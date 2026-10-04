#include "sim/app/WeldPanel.hpp"

#include "sim/motion/WeldPlan.hpp"

#include <QLabel>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace sim {

WeldPanel::WeldPanel(Scene& scene, I18nCatalog& i18n, QWidget* parent) : QWidget(parent), scene_(scene), i18n_(i18n) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    summary_ = new QLabel;
    summary_->setWordWrap(true);
    layout->addWidget(summary_);
    source_ = new QPlainTextEdit;
    source_->setPlaceholderText("id,x,y,z,nx,ny,nz,layers,electrode,example");
    source_->setMaximumHeight(90);
    layout->addWidget(source_);
    auto* importButton = new QPushButton;
    importButton->setObjectName("weldImport");
    auto* projectButton = new QPushButton;
    projectButton->setObjectName("weldProject");
    auto* alignButton = new QPushButton;
    alignButton->setObjectName("weldAlign");
    layout->addWidget(importButton);
    layout->addWidget(projectButton);
    layout->addWidget(alignButton);
    list_ = new QListWidget;
    layout->addWidget(list_, 1);
    connect(importButton, &QPushButton::clicked, this, [this] { importText(); });
    connect(projectButton, &QPushButton::clicked, this, [this] { project(); });
    connect(alignButton, &QPushButton::clicked, this, [this] { align(); });
    retranslate();
}

QString WeldPanel::trk(const char* key) const { return QString::fromStdString(i18n_.translate(key)); }

void WeldPanel::retranslate() {
    if (auto* button = findChild<QPushButton*>("weldImport")) {
        button->setText(trk("weld.import"));
    }
    if (auto* button = findChild<QPushButton*>("weldProject")) {
        button->setText(trk("weld.project"));
    }
    if (auto* button = findChild<QPushButton*>("weldAlign")) {
        button->setText(trk("weld.align"));
    }
    reload();
}

void WeldPanel::reload() {
    if (list_ == nullptr) {
        return;
    }
    list_->clear();
    int onSurface = 0;
    int reachable = 0;
    for (const WeldSpot& spot : scene_.welds) {
        if (spot.onSurface) {
            ++onSurface;
        }
        if (spot.reachable) {
            ++reachable;
        }
        const QString mark = spot.example ? trk("weld.example") : QString();
        const QString state = !spot.checked ? trk("weld.unchecked") : (spot.reachable ? trk("weld.reachable") : trk("weld.blocked"));
        list_->addItem(QString::fromStdString(spot.id) + QStringLiteral("  ") + state + QStringLiteral("  ") +
                       trk("weld.spin").arg(spot.spinDeg, 0, 'f', 0) + QStringLiteral("  ") +
                       QString::fromStdString(spot.electrode) + QStringLiteral("  ") + mark);
    }
    summary_->setText(trk("weld.summary").arg(static_cast<int>(scene_.welds.size())).arg(onSurface).arg(reachable));
}

void WeldPanel::importText() {
    std::string error;
    const std::string text = source_->toPlainText().toStdString();
    const bool xml = text.find('<') != std::string::npos;
    const bool ok = xml ? importWeldXml(text, scene_.welds, error) : importWeldCsv(text, scene_.welds, error);
    if (!ok) {
        summary_->setText(QString::fromStdString(error));
        return;
    }
    scene_.notify(ChangeKind::Structure);
    reload();
}

void WeldPanel::project() {
    const int count = projectWeldSpots(scene_);
    summary_->setText(trk("weld.projected").arg(count));
    scene_.notify(ChangeKind::Structure);
    reload();
}

void WeldPanel::align() {
    KinematicResource* robot = findResource(scene_.resources, "kr420");
    if (robot == nullptr && !scene_.resources.empty()) {
        robot = &scene_.resources.front();
    }
    if (robot == nullptr) {
        summary_->setText(trk("weld.noRobot"));
        return;
    }
    for (WeldSpot& spot : scene_.welds) {
        const SpinSearch aligned = searchWeldSpin(&scene_, *robot, spot, 12, false);
        spot.checked = true;
        spot.reachable = aligned.ok && !aligned.singular && aligned.margin > 0.02;
        spot.margin = aligned.margin;
        spot.spinDeg = aligned.spinDeg;
        spot.manipulability = aligned.manipulability;
        spot.singular = aligned.singular;
        spot.collision = aligned.collision;
    }
    scene_.notify(ChangeKind::Transform);
    reload();
}

}  // namespace sim
