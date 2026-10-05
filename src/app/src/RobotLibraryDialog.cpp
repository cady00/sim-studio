#include "sim/app/RobotLibraryDialog.hpp"

#include <QAbstractItemView>
#include <QComboBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPixmap>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QSplitter>
#include <QVBoxLayout>

namespace sim {
namespace {

const char* kSettingsKey = "library/cojtRoot";

QString kindLabel(const RobotLibraryEntry& entry) {
    if (entry.kind == "directory") {
        return QString::fromUtf8("目录");
    }
    if (entry.kind == "nested-zip") {
        return QString::fromUtf8("嵌套 zip");
    }
    return QString::fromUtf8("zip");
}

}  // namespace

RobotLibraryDialog::RobotLibraryDialog(Translate translate, QWidget* parent)
    : QDialog(parent), translate_(std::move(translate)) {
    setObjectName("robotLibrary");
    setWindowTitle(translate_("library.title"));
    resize(980, 640);

    auto* column = new QVBoxLayout(this);
    auto* pathRow = new QHBoxLayout;
    pathEdit_ = new QLineEdit;
    pathEdit_->setPlaceholderText(translate_("library.path"));
    auto* browse = new QPushButton(translate_("library.browse"));
    pathRow->addWidget(pathEdit_, 1);
    pathRow->addWidget(browse);
    column->addLayout(pathRow);

    auto* filterRow = new QHBoxLayout;
    filterEdit_ = new QLineEdit;
    filterEdit_->setPlaceholderText(translate_("library.filter"));
    brandBox_ = new QComboBox;
    filterRow->addWidget(filterEdit_, 1);
    filterRow->addWidget(brandBox_);
    column->addLayout(filterRow);

    auto* split = new QSplitter;
    list_ = new QListWidget;
    list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    preview_ = new QLabel(translate_("library.previewMissing"));
    preview_->setAlignment(Qt::AlignCenter);
    preview_->setMinimumSize(280, 240);
    preview_->setStyleSheet("background:#1c2433;color:#9aa6bd;");
    split->addWidget(list_);
    split->addWidget(preview_);
    split->setStretchFactor(0, 3);
    split->setStretchFactor(1, 2);
    column->addWidget(split, 1);

    auto* buttons = new QHBoxLayout;
    auto* import = new QPushButton(translate_("library.import"));
    status_ = new QLabel;
    status_->setWordWrap(true);
    buttons->addWidget(import);
    buttons->addWidget(status_, 1);
    column->addLayout(buttons);

    connect(browse, &QPushButton::clicked, this, &RobotLibraryDialog::browse);
    connect(pathEdit_, &QLineEdit::editingFinished, this, [this] {
        setRoot(pathEdit_->text(), true);
    });
    connect(filterEdit_, &QLineEdit::textChanged, this, [this] { refill(); });
    connect(brandBox_, &QComboBox::currentTextChanged, this, [this] { refill(); });
    connect(list_, &QListWidget::currentRowChanged, this, [this] { showPreview(); });
    connect(import, &QPushButton::clicked, this, &RobotLibraryDialog::importSelected);

    QSettings settings;
    const QString saved = settings.value(QString::fromLatin1(kSettingsKey)).toString();
    if (!saved.isEmpty()) {
        setRoot(saved, false);
    }
}

RobotLibraryDialog::~RobotLibraryDialog() {
    // Hiding the dialog in ~QDialog runs after these members are gone. A focus-out
    // on the path box would otherwise scan again and free the catalog twice.
    if (pathEdit_ != nullptr) {
        pathEdit_->disconnect(this);
    }
    if (filterEdit_ != nullptr) {
        filterEdit_->disconnect(this);
    }
    if (brandBox_ != nullptr) {
        brandBox_->disconnect(this);
    }
    if (list_ != nullptr) {
        list_->disconnect(this);
    }
}

void RobotLibraryDialog::setImporter(Importer importer) { importer_ = std::move(importer); }

void RobotLibraryDialog::setRoot(const QString& path, bool persist) {
    const QSignalBlocker block(pathEdit_);
    pathEdit_->setText(path);
    if (persist) {
        QSettings settings;
        settings.setValue(QString::fromLatin1(kSettingsKey), path);
    }
    rescan();
}

void RobotLibraryDialog::setFilter(const QString& text) { filterEdit_->setText(text); }

void RobotLibraryDialog::selectName(const QString& name) {
    if (name.isEmpty()) {
        showPreview();
        return;
    }
    for (int row = 0; row < list_->count(); ++row) {
        if (list_->item(row)->text().contains(name, Qt::CaseInsensitive)) {
            list_->setCurrentRow(row);
            showPreview();
            return;
        }
    }
    showPreview();
}

QString RobotLibraryDialog::root() const { return pathEdit_->text(); }

void RobotLibraryDialog::showEvent(QShowEvent* event) {
    QDialog::showEvent(event);
    showPreview();
}

void RobotLibraryDialog::browse() {
    const QString path = QFileDialog::getExistingDirectory(this, translate_("library.title"), pathEdit_->text());
    if (!path.isEmpty()) {
        setRoot(path, true);
    }
}

void RobotLibraryDialog::rescan() {
    entries_.clear();
    brandBox_->blockSignals(true);
    const QString previous = brandBox_->currentData().toString();
    brandBox_->clear();
    brandBox_->addItem(translate_("library.all"), QString());
    std::string error;
    entries_ = scanRobotLibrary(pathEdit_->text().toStdString(), error);
    QStringList brands;
    for (const RobotLibraryEntry& entry : entries_) {
        const QString brand = QString::fromStdString(entry.brand);
        if (!brands.contains(brand)) {
            brands << brand;
        }
    }
    brands.sort(Qt::CaseInsensitive);
    for (const QString& brand : brands) {
        brandBox_->addItem(brand, brand);
    }
    const int restore = brandBox_->findData(previous);
    brandBox_->setCurrentIndex(restore >= 0 ? restore : 0);
    brandBox_->blockSignals(false);
    refill();
    if (!error.empty() && entries_.empty()) {
        status_->setText(QString::fromStdString(error));
    } else {
        status_->setText(translate_("library.count").arg(entries_.size()));
    }
}

void RobotLibraryDialog::refill() {
    const QString filter = filterEdit_->text().trimmed();
    const QString brand = brandBox_->currentData().toString();
    list_->clear();
    for (int index = 0; index < static_cast<int>(entries_.size()); ++index) {
        const RobotLibraryEntry& entry = entries_[static_cast<std::size_t>(index)];
        const QString brandText = QString::fromStdString(entry.brand);
        const QString model = QString::fromStdString(entry.model);
        if (!brand.isEmpty() && brandText.compare(brand, Qt::CaseInsensitive) != 0) {
            continue;
        }
        const QString haystack = brandText + QLatin1Char(' ') + model + QLatin1Char(' ') + QString::fromStdString(entry.relative);
        if (!filter.isEmpty() && !haystack.contains(filter, Qt::CaseInsensitive)) {
            continue;
        }
        auto* item = new QListWidgetItem(brandText + QString::fromUtf8("  ") + model + QString::fromUtf8("  (") +
                                          kindLabel(entry) + QStringLiteral(")"));
        item->setData(Qt::UserRole, index);
        list_->addItem(item);
    }
    if (list_->count() > 0 && list_->currentRow() < 0) {
        list_->setCurrentRow(0);
    } else {
        showPreview();
    }
}

void RobotLibraryDialog::showPreview() {
    preview_->setPixmap(QPixmap());
    preview_->setText(translate_("library.previewMissing"));
    const auto* item = list_->currentItem();
    if (item == nullptr) {
        return;
    }
    const int index = item->data(Qt::UserRole).toInt();
    if (index < 0 || index >= static_cast<int>(entries_.size())) {
        return;
    }
    const RobotLibraryEntry& entry = entries_[static_cast<std::size_t>(index)];
    if (!entry.preview) {
        return;
    }
    std::string error;
    const std::vector<std::uint8_t> bytes = robotLibraryPreview(entry, error);
    QImage image;
    if (!bytes.empty()) {
        image.loadFromData(bytes.data(), static_cast<int>(bytes.size()), "BMP");
    }
    if (image.isNull()) {
        preview_->setText(error.empty() ? translate_("library.previewMissing") : QString::fromStdString(error));
        return;
    }
    preview_->setText(QString());
    preview_->setPixmap(QPixmap::fromImage(image).scaled(preview_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void RobotLibraryDialog::importSelected() {
    if (!importer_) {
        return;
    }
    std::vector<RobotLibraryEntry> chosen;
    for (QListWidgetItem* item : list_->selectedItems()) {
        const int index = item->data(Qt::UserRole).toInt();
        if (index >= 0 && index < static_cast<int>(entries_.size())) {
            chosen.push_back(entries_[static_cast<std::size_t>(index)]);
        }
    }
    if (chosen.empty()) {
        status_->setText(translate_("library.none"));
        return;
    }
    status_->setText(importer_(chosen));
}

}  // namespace sim
