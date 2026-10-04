#include "sim/app/CollisionPanel.hpp"

#include "sim/collision/Query.hpp"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QStringList>
#include <QPushButton>
#include <QVBoxLayout>

namespace sim {
namespace {

std::vector<std::string> splitComma(const QString& text) {
    std::vector<std::string> out;
    for (const QString& part : text.split(',', Qt::SkipEmptyParts)) {
        const QString trimmed = part.trimmed();
        if (!trimmed.isEmpty()) {
            out.push_back(trimmed.toStdString());
        }
    }
    return out;
}

QString joinComma(const std::vector<std::string>& names) {
    QStringList list;
    for (const std::string& name : names) {
        list.push_back(QString::fromStdString(name));
    }
    return list.join(", ");
}

}  // namespace

CollisionPanel::CollisionPanel(Scene& scene, I18nCatalog& i18n, QWidget* parent) : QWidget(parent), scene_(scene), i18n_(i18n) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    auto* form = new QFormLayout;
    clearance_ = new QDoubleSpinBox;
    clearance_->setRange(0, 1.0e6);
    clearance_->setDecimals(2);
    clearance_->setSuffix(QStringLiteral(" mm"));
    setAName_ = new QLineEdit;
    setAObjects_ = new QLineEdit;
    setBName_ = new QLineEdit;
    setBObjects_ = new QLineEdit;
    form->addRow(trk("collision.clearance"), clearance_);
    form->addRow(trk("collision.setA"), setAName_);
    form->addRow(trk("collision.objectsA"), setAObjects_);
    form->addRow(trk("collision.setB"), setBName_);
    form->addRow(trk("collision.objectsB"), setBObjects_);
    layout->addLayout(form);
    auto* buttons = new QHBoxLayout;
    auto* save = new QPushButton(trk("collision.save"));
    auto* run = new QPushButton(trk("collision.run"));
    save->setProperty("i18n", QStringLiteral("collision.save"));
    run->setProperty("i18n", QStringLiteral("collision.run"));
    buttons->addWidget(save);
    buttons->addWidget(run);
    layout->addLayout(buttons);
    summary_ = new QLabel;
    results_ = new QListWidget;
    results_->setMinimumHeight(120);
    layout->addWidget(summary_);
    layout->addWidget(results_, 1);
    connect(save, &QPushButton::clicked, this, [this] {
        readFields();
        runQuery();
    });
    connect(run, &QPushButton::clicked, this, [this] { runQuery(); });
}

QString CollisionPanel::trk(const char* key) const { return QString::fromStdString(i18n_.translate(key)); }

void CollisionPanel::retranslate() {
    const auto retitle = [this](const char* key) {
        if (auto* widget = findChild<QWidget*>(QString())) {
            (void)widget;
            (void)key;
        }
    };
    (void)retitle;
    if (auto* save = findChild<QPushButton*>()) {
        (void)save;
    }
    const auto buttons = findChildren<QPushButton*>();
    for (QPushButton* button : buttons) {
        const QByteArray key = button->property("i18n").toString().toLatin1();
        if (!key.isEmpty()) {
            button->setText(trk(key.constData()));
        }
    }
}

void CollisionPanel::readFields() {
    scene_.proximity.clearanceMm = clearance_->value();
    CollisionSet a;
    a.name = setAName_->text().trimmed().toStdString();
    a.objects = splitComma(setAObjects_->text());
    CollisionSet b;
    b.name = setBName_->text().trimmed().toStdString();
    b.objects = splitComma(setBObjects_->text());
    scene_.proximity.sets.clear();
    scene_.proximity.pairs.clear();
    if (!a.name.empty()) {
        scene_.proximity.sets.push_back(std::move(a));
    }
    if (!b.name.empty()) {
        scene_.proximity.sets.push_back(std::move(b));
    }
    if (scene_.proximity.sets.size() == 2) {
        scene_.proximity.pairs.emplace_back(scene_.proximity.sets[0].name, scene_.proximity.sets[1].name);
    }
}

void CollisionPanel::reload() {
    loading_ = true;
    clearance_->setValue(scene_.proximity.clearanceMm);
    if (!scene_.proximity.sets.empty()) {
        setAName_->setText(QString::fromStdString(scene_.proximity.sets[0].name));
        setAObjects_->setText(joinComma(scene_.proximity.sets[0].objects));
    } else {
        setAName_->clear();
        setAObjects_->clear();
    }
    if (scene_.proximity.sets.size() > 1) {
        setBName_->setText(QString::fromStdString(scene_.proximity.sets[1].name));
        setBObjects_->setText(joinComma(scene_.proximity.sets[1].objects));
    } else {
        setBName_->clear();
        setBObjects_->clear();
    }
    loading_ = false;
    runQuery();
}

void CollisionPanel::runQuery() {
    if (loading_) {
        return;
    }
    scene_.proximity.clearanceMm = clearance_->value();
    auto query = createCollisionQuery();
    query->rebuild(scene_);
    const std::vector<ContactHit> hits = query->query(scene_, scene_.proximity.clearanceMm);
    query->commitCache(scene_);
    paintProximity(scene_, hits);
    results_->clear();
    int overlaps = 0;
    int near = 0;
    for (const ContactHit& hit : hits) {
        const bool overlap = hit.kind == HitKind::Overlap;
        if (overlap) {
            ++overlaps;
        } else {
            ++near;
        }
        QString line = trk(overlap ? "collision.overlap" : "collision.near");
        line += QStringLiteral("  ") + QString::fromStdString(hit.a + " / " + hit.b);
        line += QStringLiteral("  ") + QString::number(overlap ? hit.penetration : hit.distance, 'f', 2) + QStringLiteral(" mm");
        if (overlap) {
            line += QStringLiteral("  V ") + QString::number(hit.volume, 'f', 1);
            if (hit.volumeEstimated) {
                line += QStringLiteral(" ") + trk("collision.estimated");
            }
        }
        results_->addItem(line);
    }
    summary_->setText(trk("collision.summary").arg(overlaps).arg(near));
    if (viewport_ != nullptr) {
        viewport_->update();
    }
}

QString CollisionPanel::summaryText() const { return summary_ == nullptr ? QString() : summary_->text(); }

}  // namespace sim
