#include "sim/app/WeldPanel.hpp"

#include "sim/motion/Envelope.hpp"
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
    auto* reachButton = new QPushButton;
    reachButton->setObjectName("weldReach");
    auto* baseButton = new QPushButton;
    baseButton->setObjectName("weldBase");
    auto* planButton = new QPushButton;
    planButton->setObjectName("weldPlan");
    auto* envelopeButton = new QPushButton;
    envelopeButton->setObjectName("weldEnvelope");
    auto* interlockButton = new QPushButton;
    interlockButton->setObjectName("weldInterlock");
    layout->addWidget(importButton);
    layout->addWidget(projectButton);
    layout->addWidget(alignButton);
    layout->addWidget(reachButton);
    layout->addWidget(baseButton);
    layout->addWidget(planButton);
    layout->addWidget(envelopeButton);
    layout->addWidget(interlockButton);
    list_ = new QListWidget;
    layout->addWidget(list_, 1);
    connect(importButton, &QPushButton::clicked, this, [this] { importText(); });
    connect(projectButton, &QPushButton::clicked, this, [this] { project(); });
    connect(alignButton, &QPushButton::clicked, this, [this] { align(); });
    connect(reachButton, &QPushButton::clicked, this, [this] { reach(); });
    connect(baseButton, &QPushButton::clicked, this, [this] { baseRegion(); });
    connect(planButton, &QPushButton::clicked, this, [this] { planTour(); });
    connect(envelopeButton, &QPushButton::clicked, this, [this] { envelope(); });
    connect(interlockButton, &QPushButton::clicked, this, [this] { interlock(); });
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
    if (auto* button = findChild<QPushButton*>("weldReach")) {
        button->setText(trk("weld.reach"));
    }
    if (auto* button = findChild<QPushButton*>("weldBase")) {
        button->setText(trk("weld.base"));
    }
    if (auto* button = findChild<QPushButton*>("weldPlan")) {
        button->setText(trk("weld.plan"));
    }
    if (auto* button = findChild<QPushButton*>("weldEnvelope")) {
        button->setText(trk("weld.envelope"));
    }
    if (auto* button = findChild<QPushButton*>("weldInterlock")) {
        button->setText(trk("weld.interlock"));
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
        QString flags;
        if (spot.checked) {
            flags = QStringLiteral("  ") + trk("weld.margin").arg(spot.margin, 0, 'f', 2);
            if (spot.singular) {
                flags += QStringLiteral("  ") + trk("weld.singular");
            }
            if (spot.collision) {
                flags += QStringLiteral("  ") + trk("weld.collision");
            }
        }
        list_->addItem(QString::fromStdString(spot.id) + QStringLiteral("  ") + state + QStringLiteral("  ") +
                       trk("weld.spin").arg(spot.spinDeg, 0, 'f', 0) + flags + QStringLiteral("  ") +
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

KinematicResource* WeldPanel::robot() {
    KinematicResource* resource = findResource(scene_.resources, "kr420");
    if (resource == nullptr && !scene_.resources.empty()) {
        resource = &scene_.resources.front();
    }
    return resource;
}

void WeldPanel::align() {
    KinematicResource* resource = robot();
    if (resource == nullptr) {
        summary_->setText(trk("weld.noRobot"));
        return;
    }
    for (WeldSpot& spot : scene_.welds) {
        const SpinSearch aligned = searchWeldSpin(&scene_, *resource, spot, 12, false);
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

void WeldPanel::reach() {
    KinematicResource* resource = robot();
    if (resource == nullptr) {
        summary_->setText(trk("weld.noRobot"));
        return;
    }
    const std::vector<SpotReach> report = checkWeldReach(&scene_, *resource, scene_.welds, true);
    int reachable = 0;
    for (const SpotReach& row : report) {
        if (row.reachable) {
            ++reachable;
        }
    }
    scene_.notify(ChangeKind::Transform);
    reload();
    summary_->setText(trk("weld.reachSummary").arg(reachable).arg(static_cast<int>(report.size())));
}

void WeldPanel::baseRegion() {
    KinematicResource* resource = robot();
    if (resource == nullptr || scene_.welds.empty()) {
        summary_->setText(trk("weld.noRobot"));
        return;
    }
    const BaseRegion region = optimizeRobotBase(*resource, scene_.welds, 400, 5);
    reload();
    if (!region.found) {
        summary_->setText(trk("weld.baseMiss").arg(region.bestCovered).arg(static_cast<int>(scene_.welds.size())));
    } else {
        summary_->setText(trk("weld.baseHit")
                              .arg(region.minX, 0, 'f', 0)
                              .arg(region.maxX, 0, 'f', 0)
                              .arg(region.minY, 0, 'f', 0)
                              .arg(region.maxY, 0, 'f', 0));
    }
}

void WeldPanel::planTour() {
    const int points = planExampleWeldTour(scene_);
    scene_.notify(ChangeKind::Structure);
    reload();
    summary_->setText(trk("weld.tour").arg(points));
}

void WeldPanel::envelope() {
    KinematicResource* resource = robot();
    if (resource == nullptr) {
        summary_->setText(trk("weld.noRobot"));
        return;
    }
    std::vector<std::vector<double>> joints;
    for (const RobotPath& path : scene_.robotPaths) {
        if (path.name != "weld_tour") {
            continue;
        }
        for (const PathPoint& point : path.points) {
            if (!point.joints.empty()) {
                joints.push_back(point.joints);
            }
        }
    }
    const MotionEnvelope built = buildTcpEnvelope(scene_, *resource, joints, "weld_envelope", 45);
    scene_.notify(ChangeKind::Structure);
    reload();
    summary_->setText(trk("weld.envelopeSummary").arg(static_cast<int>(built.nodes.size())));
}

void WeldPanel::interlock() {
    if (scene_.interlocks.empty() && !scene_.welds.empty()) {
        InterlockZone zone;
        zone.name = "weld_cell";
        zone.radius = 900;
        zone.resources = {"kr420"};
        for (const WeldSpot& spot : scene_.welds) {
            zone.center = zone.center + spot.position;
        }
        zone.center = zone.center / static_cast<double>(scene_.welds.size());
        scene_.interlocks.push_back(zone);
    }
    const std::vector<InterlockHit> hits = evaluateInterlocks(scene_);
    scene_.notify(ChangeKind::Structure);
    reload();
    summary_->setText(trk("weld.interlockSummary").arg(static_cast<int>(hits.size())));
}

}  // namespace sim
