#include "sim/app/MotionPanel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFrame>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

QDoubleSpinBox* spinBox(double minValue, double maxValue) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(minValue, maxValue);
    spin->setDecimals(2);
    spin->setSingleStep(5);
    return spin;
}

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

}  // namespace

MotionPanel::MotionPanel(Scene& scene, I18nCatalog& i18n, LogicMachine& logic, BuiltinRobotSolver& robot, QWidget* parent)
    : QWidget(parent), scene_(scene), i18n_(i18n), logic_(logic), robot_(robot) {
    buildUi();
    refresh();
}

QString MotionPanel::trk(const char* key) const { return QString::fromStdString(i18n_.translate(key)); }

void MotionPanel::tag(QWidget* widget, const char* key) { widget->setProperty("i18n", QString::fromLatin1(key)); }

void MotionPanel::retranslate() {
    if (tabs_ != nullptr) {
        tabs_->setTabText(0, trk("motion.tab.kin"));
        tabs_->setTabText(1, trk("motion.tab.logic"));
        tabs_->setTabText(2, trk("motion.tab.robot"));
    }
    if (solidLabel_ != nullptr) {
        solidLabel_->setText(trk("motion.solids"));
    }
    auto refill = [this](QComboBox* box, std::initializer_list<std::pair<const char*, const char*>> items) {
        if (box == nullptr) {
            return;
        }
        const QVariant current = box->currentData();
        const QSignalBlocker blocker(box);
        box->clear();
        for (const auto& item : items) {
            box->addItem(trk(item.first), QString::fromLatin1(item.second));
        }
        const int index = box->findData(current);
        if (index >= 0) {
            box->setCurrentIndex(index);
        }
    };
    refill(externalKindCombo_, {{"motion.kind.track", "track"}, {"motion.kind.positioner", "positioner"}});
    refill(redundancyCombo_, {{"motion.redundancy.fixed", "fixed"},
                              {"motion.redundancy.travel", "travel"},
                              {"motion.redundancy.midrange", "midrange"},
                              {"motion.redundancy.manipulability", "manipulability"}});
    refill(pathExternalCombo_, {{"motion.external.locked", "locked"},
                                {"motion.external.sync", "sync"},
                                {"motion.external.positioner", "positioner"}});
    if (externalTargets_ != nullptr) {
        externalTargets_->setPlaceholderText(trk("motion.externalValues"));
    }
    const auto buttons = findChildren<QPushButton*>();
    for (QPushButton* button : buttons) {
        const QString key = button->property("i18n").toString();
        if (!key.isEmpty()) {
            button->setText(trk(key.toLatin1().constData()));
        }
    }
}

KinematicResource* MotionPanel::resource() {
    if (resourceCombo_ == nullptr || resourceCombo_->currentIndex() < 0) {
        return nullptr;
    }
    return findResource(scene_.resources, resourceCombo_->currentText().toStdString());
}

void MotionPanel::buildUi() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    tabs_ = new QTabWidget;
    layout->addWidget(tabs_);
    liveLabel_ = new QLabel;
    liveLabel_->setWordWrap(true);
    layout->addWidget(liveLabel_);

    auto* kin = new QWidget;
    auto* kinLayout = new QVBoxLayout(kin);
    solidLabel_ = new QLabel(trk("motion.solids"));
    solidList_ = new QListWidget;
    solidList_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    solidList_->setMaximumHeight(140);
    kinLayout->addWidget(solidLabel_);
    kinLayout->addWidget(solidList_);
    connect(solidList_, &QListWidget::itemSelectionChanged, this, [this] {
        if (refreshing_ || solidList_ == nullptr) {
            return;
        }
        std::vector<std::string> ids;
        const auto selected = solidList_->selectedItems();
        ids.reserve(static_cast<std::size_t>(selected.size()));
        for (QListWidgetItem* item : selected) {
            ids.push_back(item->data(Qt::UserRole).toString().toStdString());
        }
        scene_.setSelection(ids);
        if (viewport_ != nullptr) {
            viewport_->update();
        }
    });
    auto* kinForm = new QFormLayout;
    resourceCombo_ = new QComboBox;
    resourceName_ = new QLineEdit;
    linkName_ = new QLineEdit;
    parentCombo_ = new QComboBox;
    childCombo_ = new QComboBox;
    motionCombo_ = new QComboBox;
    motionCombo_->addItems({"revolute", "prismatic"});
    jointCombo_ = new QComboBox;
    for (QDoubleSpinBox*& spin : axisSpin_) {
        spin = spinBox(-1e6, 1e6);
    }
    axisSpin_[2]->setValue(1);
    for (QDoubleSpinBox*& spin : anchorSpin_) {
        spin = spinBox(-1e6, 1e6);
    }
    lowerSpin_ = spinBox(-1e6, 1e6);
    upperSpin_ = spinBox(-1e6, 1e6);
    upperSpin_->setValue(180);
    speedSpin_ = spinBox(0, 1e6);
    speedSpin_->setValue(90);
    accelSpin_ = spinBox(0, 1e7);
    accelSpin_->setValue(360);
    coupleCombo_ = new QComboBox;
    ratioSpin_ = spinBox(-100, 100);
    ratioSpin_->setValue(1);
    poseName_ = new QLineEdit("home");
    poseCombo_ = new QComboBox;
    profileCombo_ = new QComboBox;
    profileCombo_->addItems({"trapezoid", "scurve"});
    pickLabel_ = new QLabel("-");
    kinForm->addRow(trk("motion.resource"), resourceCombo_);
    kinForm->addRow(resourceName_);
    kinForm->addRow(trk("motion.link"), linkName_);
    kinForm->addRow(trk("motion.parent"), parentCombo_);
    kinForm->addRow(trk("motion.child"), childCombo_);
    kinForm->addRow(motionCombo_);
    kinForm->addRow(jointCombo_);
    kinForm->addRow("ax", axisSpin_[0]);
    kinForm->addRow("ay", axisSpin_[1]);
    kinForm->addRow("az", axisSpin_[2]);
    kinForm->addRow("ox", anchorSpin_[0]);
    kinForm->addRow("oy", anchorSpin_[1]);
    kinForm->addRow("oz", anchorSpin_[2]);
    kinForm->addRow(lowerSpin_);
    kinForm->addRow(upperSpin_);
    kinForm->addRow(speedSpin_);
    kinForm->addRow(accelSpin_);
    kinForm->addRow(trk("motion.couple"), coupleCombo_);
    kinForm->addRow(trk("motion.ratio"), ratioSpin_);
    carrierName_ = new QLineEdit("kl");
    mountLinkName_ = new QLineEdit("carriage");
    externalKindCombo_ = new QComboBox;
    externalKindCombo_->addItem(trk("motion.kind.track"), QStringLiteral("track"));
    externalKindCombo_->addItem(trk("motion.kind.positioner"), QStringLiteral("positioner"));
    redundancyCombo_ = new QComboBox;
    redundancyCombo_->addItem(trk("motion.redundancy.fixed"), QStringLiteral("fixed"));
    redundancyCombo_->addItem(trk("motion.redundancy.travel"), QStringLiteral("travel"));
    redundancyCombo_->addItem(trk("motion.redundancy.midrange"), QStringLiteral("midrange"));
    redundancyCombo_->addItem(trk("motion.redundancy.manipulability"), QStringLiteral("manipulability"));
    weightTravel_ = spinBox(0, 100);
    weightTravel_->setValue(1);
    weightMid_ = spinBox(0, 100);
    weightMid_->setValue(0.2);
    weightManip_ = spinBox(0, 100);
    weightManip_->setValue(0.2);
    kinForm->addRow(trk("motion.carrier"), carrierName_);
    kinForm->addRow(trk("motion.mountLink"), mountLinkName_);
    kinForm->addRow(trk("motion.externalKind"), externalKindCombo_);
    kinForm->addRow(trk("motion.redundancy"), redundancyCombo_);
    kinForm->addRow(trk("motion.weightTravel"), weightTravel_);
    kinForm->addRow(trk("motion.weightMid"), weightMid_);
    kinForm->addRow(trk("motion.weightManip"), weightManip_);
    kinForm->addRow(poseName_);
    kinForm->addRow(poseCombo_);
    kinForm->addRow(profileCombo_);
    kinForm->addRow(pickLabel_);
    kinLayout->addLayout(kinForm);
    auto* kinButtons = new QHBoxLayout;
    auto addButton = [this, kinButtons](const char* key, auto slot) {
        auto* button = new QPushButton;
        tag(button, key);
        button->setObjectName(QString::fromLatin1(key));
        button->setText(trk(key));
        connect(button, &QPushButton::clicked, this, slot);
        kinButtons->addWidget(button);
        return button;
    };
    addButton("motion.newResource", [this] {
        const QString name = resourceName_->text().trimmed().isEmpty() ? QStringLiteral("resource") : resourceName_->text().trimmed();
        ensureResource(scene_, name.toStdString());
        refresh();
    });
    addButton("motion.suggest", [this] {
        const SuggestResult suggestion = suggestLinkGroups(scene_);
        suggestNote_ = suggestion.note;
        const QString name = resourceName_->text().trimmed().isEmpty() ? QStringLiteral("robot") : resourceName_->text().trimmed();
        std::string error;
        std::size_t jointCount = 0;
        for (std::size_t i = 1; i < suggestion.links.size(); ++i) {
            if (suggestion.links[i].hasJoint) {
                ++jointCount;
            }
        }
        const QString lowered = name.toLower();
        const bool kr420 = jointCount == 6 && lowered.contains(QStringLiteral("kr420"));
        const std::vector<RevoluteLimit> limits =
            kr420 ? kr420DatasheetLimits() : std::vector<RevoluteLimit>(std::max<std::size_t>(jointCount, 1), RevoluteLimit{});
        if (!applyLinkSuggestions(scene_, name.toStdString(), suggestion, limits, error)) {
            liveLabel_->setText(QString::fromStdString(error.empty() ? suggestion.note : error));
            return;
        }
        refresh();
        const int index = resourceCombo_->findText(name);
        if (index >= 0) {
            resourceCombo_->setCurrentIndex(index);
            fillCombos();
        }
        refreshLive();
    });
    addButton("motion.group", [this] {
        KinematicResource* current = resource();
        if (current == nullptr || scene_.selection().empty()) {
            return;
        }
        const std::string link = linkName_->text().trimmed().isEmpty() ? std::string("link") : linkName_->text().trimmed().toStdString();
        if (findLink(*current, link) == nullptr) {
            const std::string parent = parentCombo_->currentText().toStdString();
            addLink(scene_, *current, link, parent == link ? std::string{} : parent, {});
        }
        std::string error;
        if (!adoptParts(scene_, *current, link, scene_.selection(), error)) {
            liveLabel_->setText(QString::fromStdString(error));
        }
        refresh();
    });
    addButton("motion.addJoint", [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        const std::string parent = parentCombo_->currentText().toStdString();
        const std::string child = childCombo_->currentText().toStdString();
        if (parent.empty() || child.empty() || parent == child) {
            return;
        }
        const JointMotion motion = motionCombo_->currentText() == "prismatic" ? JointMotion::Prismatic : JointMotion::Revolute;
        const Vec3 axis{axisSpin_[0]->value(), axisSpin_[1]->value(), axisSpin_[2]->value()};
        const Vec3 anchor{anchorSpin_[0]->value(), anchorSpin_[1]->value(), anchorSpin_[2]->value()};
        KinJoint& joint = addJoint(*current, scene_, parent + "_" + child, parent, child, motion, axis, anchor);
        joint.lower = lowerSpin_->value();
        joint.upper = upperSpin_->value();
        joint.maxSpeed = speedSpin_->value();
        joint.maxAccel = accelSpin_->value();
        applyResource(scene_, *current);
        refresh();
    });
    addButton("motion.attachExternal", [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        std::string error;
        const std::string kind = externalKindCombo_->currentData().toString().toStdString();
        if (!attachExternalAxes(scene_, *current, carrierName_->text().trimmed().toStdString(),
                                mountLinkName_->text().trimmed().toStdString(), kind, error)) {
            liveLabel_->setText(QString::fromStdString(error));
            return;
        }
        applyResource(scene_, *current);
        refresh();
        if (viewport_ != nullptr) {
            viewport_->update();
        }
    });
    addButton("motion.axisFromPoints", [this] {
        if (pickCount_ < 2) {
            return;
        }
        KinematicResource* current = resource();
        if (current == nullptr || jointCombo_->currentIndex() < 0) {
            axisSpin_[0]->setValue(picks_[1].x - picks_[0].x);
            axisSpin_[1]->setValue(picks_[1].y - picks_[0].y);
            axisSpin_[2]->setValue(picks_[1].z - picks_[0].z);
            anchorSpin_[0]->setValue(picks_[0].x);
            anchorSpin_[1]->setValue(picks_[0].y);
            anchorSpin_[2]->setValue(picks_[0].z);
            return;
        }
        KinJoint* joint = findJoint(*current, jointCombo_->currentText().toStdString());
        if (joint == nullptr) {
            return;
        }
        setAxisFromPoints(*joint, picks_[0], picks_[1]);
        applyResource(scene_, *current);
        if (viewport_ != nullptr) {
            viewport_->update();
        }
    });
    faceAxisButton_ = new QPushButton;
    faceAxisButton_->setCheckable(true);
    tag(faceAxisButton_, "motion.faceAxis");
    faceAxisButton_->setText(trk("motion.faceAxis"));
    kinButtons->addWidget(faceAxisButton_);
    kinLayout->addLayout(kinButtons);
    auto* jogRow = new QHBoxLayout;
    auto* minus = new QPushButton;
    tag(minus, "motion.jogMinus");
    minus->setText(trk("motion.jogMinus"));
    auto* plus = new QPushButton;
    tag(plus, "motion.jogPlus");
    plus->setText(trk("motion.jogPlus"));
    connect(minus, &QPushButton::clicked, this, [this] { jog(-5); });
    connect(plus, &QPushButton::clicked, this, [this] { jog(5); });
    jogRow->addWidget(minus);
    jogRow->addWidget(plus);
    auto* couple = new QPushButton;
    tag(couple, "motion.applyCouple");
    couple->setText(trk("motion.applyCouple"));
    connect(couple, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        KinJoint* joint = findJoint(*current, jointCombo_->currentText().toStdString());
        if (joint == nullptr) {
            return;
        }
        joint->coupledTo = coupleCombo_->currentText().toStdString();
        joint->coupleRatio = ratioSpin_->value();
        applyResource(scene_, *current);
        refreshLive();
    });
    jogRow->addWidget(couple);
    kinLayout->addLayout(jogRow);
    auto* poseRow = new QHBoxLayout;
    auto* capture = new QPushButton;
    tag(capture, "motion.capturePose");
    capture->setText(trk("motion.capturePose"));
    auto* goPose = new QPushButton;
    tag(goPose, "motion.movePose");
    goPose->setText(trk("motion.movePose"));
    connect(capture, &QPushButton::clicked, this, [this] {
        if (KinematicResource* current = resource()) {
            capturePose(*current, poseName_->text().toStdString());
            refresh();
        }
    });
    connect(goPose, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        std::string error;
        const ProfileKind profile = profileCombo_->currentText() == "scurve" ? ProfileKind::SCurve : ProfileKind::Trapezoid;
        if (!beginPoseMove(*current, poseCombo_->currentText().toStdString(), profile, move_, error)) {
            liveLabel_->setText(QString::fromStdString(error));
            return;
        }
        ensureMoveTimer();
        moveTimer_->start();
    });
    poseRow->addWidget(capture);
    poseRow->addWidget(goPose);
    kinLayout->addLayout(poseRow);
    auto* kinScroll = new QScrollArea;
    kinScroll->setWidgetResizable(true);
    kinScroll->setFrameShape(QFrame::NoFrame);
    kinScroll->setWidget(kin);
    tabs_->addTab(kinScroll, trk("motion.tab.kin"));

    auto* logicPage = new QWidget;
    auto* logicLayout = new QVBoxLayout(logicPage);
    blockCombo_ = new QComboBox;
    blockName_ = new QLineEdit("cycle");
    runSignal_ = new QLineEdit;
    inputs_ = new QLineEdit;
    outputs_ = new QLineEdit;
    actionKind_ = new QComboBox;
    actionKind_->addItems({"move_pose", "jog", "set_output", "reset_output", "set_value", "wait", "wait_signal"});
    actionText_ = new QLineEdit;
    actionSeconds_ = spinBox(0, 3600);
    actionList_ = new QListWidget;
    python_ = new QPlainTextEdit;
    python_->setPlaceholderText("def on_step(dt):\n    pass");
    python_->setMaximumHeight(90);
    logicLayout->addWidget(blockCombo_);
    logicLayout->addWidget(blockName_);
    logicLayout->addWidget(runSignal_);
    logicLayout->addWidget(inputs_);
    logicLayout->addWidget(outputs_);
    logicLayout->addWidget(actionKind_);
    logicLayout->addWidget(actionText_);
    logicLayout->addWidget(actionSeconds_);
    logicLayout->addWidget(actionList_);
    logicLayout->addWidget(python_);
    auto* logicButtons = new QHBoxLayout;
    auto* addAction = new QPushButton;
    tag(addAction, "motion.addAction");
    addAction->setText(trk("motion.addAction"));
    auto* runLogic = new QPushButton;
    tag(runLogic, "motion.runLogic");
    runLogic->setText(trk("motion.runLogic"));
    auto* usePython = new QPushButton;
    tag(usePython, "motion.usePython");
    usePython->setText(trk("motion.usePython"));
    auto* saveLib = new QPushButton;
    tag(saveLib, "motion.saveLib");
    saveLib->setText(trk("motion.saveLib"));
    auto* loadLib = new QPushButton;
    tag(loadLib, "motion.loadLib");
    loadLib->setText(trk("motion.loadLib"));
    connect(addAction, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        const std::string name = blockName_->text().trimmed().isEmpty() ? std::string("block") : blockName_->text().trimmed().toStdString();
        LogicBlock* block = nullptr;
        for (LogicBlock& candidate : scene_.logicBlocks) {
            if (candidate.name == name) {
                block = &candidate;
            }
        }
        if (block == nullptr) {
            LogicBlock created;
            created.name = name;
            scene_.logicBlocks.push_back(std::move(created));
            block = &scene_.logicBlocks.back();
        }
        block->resource = current == nullptr ? std::string{} : current->name;
        block->runSignal = runSignal_->text().trimmed().toStdString();
        block->inputs = splitComma(inputs_->text());
        block->outputs = splitComma(outputs_->text());
        block->python = python_->toPlainText().toStdString();
        if (block->steps.empty()) {
            block->steps.push_back({});
        }
        LogicAction action;
        action.kind = actionKind_->currentText().toStdString();
        action.resource = block->resource;
        action.pose = actionText_->text().toStdString();
        action.joint = actionText_->text().toStdString();
        action.signal = actionText_->text().toStdString();
        action.seconds = actionSeconds_->value();
        action.speed = speedSpin_->value();
        action.profile = profileCombo_->currentText().toStdString();
        block->steps.back().actions.push_back(action);
        actionList_->addItem(QString::fromStdString(action.kind + " " + action.pose));
        refresh();
    });
    connect(runLogic, &QPushButton::clicked, this, [this] {
        const std::string name = blockCombo_->currentText().isEmpty() ? blockName_->text().toStdString() : blockCombo_->currentText().toStdString();
        std::string error;
        if (!logic_.start(scene_, name, error)) {
            liveLabel_->setText(QString::fromStdString(error));
            return;
        }
        if (play_) {
            play_("");
        }
    });
    connect(usePython, &QPushButton::clicked, this, [this] { emit pythonRequested(python_->toPlainText()); });
    connect(saveLib, &QPushButton::clicked, this, [this] {
        if (resource() == nullptr) {
            return;
        }
        const QString path = QFileDialog::getSaveFileName(this, trk("motion.saveLib"), QString(), trk("file.filter.library"));
        if (path.isEmpty()) {
            return;
        }
        std::string error;
        if (!saveLibrary(scene_, resource()->name, path.toStdString(), error)) {
            liveLabel_->setText(QString::fromStdString(error));
        }
    });
    connect(loadLib, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, trk("motion.loadLib"), QString(), trk("file.filter.library"));
        if (path.isEmpty()) {
            return;
        }
        std::string error;
        if (!loadLibrary(scene_, path.toStdString(), error)) {
            liveLabel_->setText(QString::fromStdString(error));
            return;
        }
        refresh();
    });
    logicButtons->addWidget(addAction);
    logicButtons->addWidget(runLogic);
    logicButtons->addWidget(usePython);
    logicButtons->addWidget(saveLib);
    logicButtons->addWidget(loadLib);
    logicLayout->addLayout(logicButtons);
    tabs_->addTab(logicPage, trk("motion.tab.logic"));

    auto* robotPage = new QWidget;
    auto* robotLayout = new QVBoxLayout(robotPage);
    ikCombo_ = new QComboBox;
    ikCombo_->addItems({"none", "scara", "spherical_wrist", "numerical"});
    for (QDoubleSpinBox*& spin : tcpSpin_) {
        spin = spinBox(-1e5, 1e5);
    }
    moveKind_ = new QComboBox;
    moveKind_->addItems({"ptp", "lin", "circ"});
    pathSpeed_ = spinBox(1, 1e5);
    pathSpeed_->setValue(200);
    pathZone_ = spinBox(0, 1e4);
    for (QDoubleSpinBox*& spin : goalSpin_) {
        spin = spinBox(-1e5, 1e5);
    }
    pathName_ = new QLineEdit("path");
    pathExternalCombo_ = new QComboBox;
    pathExternalCombo_->addItem(trk("motion.external.locked"), QStringLiteral("locked"));
    pathExternalCombo_->addItem(trk("motion.external.sync"), QStringLiteral("sync"));
    pathExternalCombo_->addItem(trk("motion.external.positioner"), QStringLiteral("positioner"));
    externalTargets_ = new QLineEdit;
    externalTargets_->setPlaceholderText(trk("motion.externalValues"));
    pathList_ = new QListWidget;
    planLabel_ = new QLabel;
    planLabel_->setWordWrap(true);
    robotLayout->addWidget(ikCombo_);
    auto* tcpRow = new QHBoxLayout;
    tcpRow->addWidget(tcpSpin_[0]);
    tcpRow->addWidget(tcpSpin_[1]);
    tcpRow->addWidget(tcpSpin_[2]);
    robotLayout->addLayout(tcpRow);
    robotLayout->addWidget(moveKind_);
    robotLayout->addWidget(pathSpeed_);
    robotLayout->addWidget(pathZone_);
    auto* goalRow = new QHBoxLayout;
    goalRow->addWidget(goalSpin_[0]);
    goalRow->addWidget(goalSpin_[1]);
    goalRow->addWidget(goalSpin_[2]);
    robotLayout->addLayout(goalRow);
    robotLayout->addWidget(pathName_);
    robotLayout->addWidget(pathExternalCombo_);
    robotLayout->addWidget(externalTargets_);
    robotLayout->addWidget(pathList_);
    robotLayout->addWidget(planLabel_);
    auto* robotButtons = new QHBoxLayout;
    auto* addPoint = new QPushButton;
    tag(addPoint, "motion.addPoint");
    addPoint->setText(trk("motion.addPoint"));
    auto* plan = new QPushButton;
    tag(plan, "motion.plan");
    plan->setText(trk("motion.plan"));
    auto* playPath = new QPushButton;
    tag(playPath, "motion.playPath");
    playPath->setText(trk("motion.playPath"));
    auto* reach = new QPushButton;
    tag(reach, "motion.reach");
    reach->setText(trk("motion.reach"));
    auto* attach = new QPushButton;
    tag(attach, "motion.attach");
    attach->setText(trk("motion.attach"));
    auto* detach = new QPushButton;
    tag(detach, "motion.detach");
    detach->setText(trk("motion.detach"));
    auto writeRedundancy = [this] {
        if (refreshing_) {
            return;
        }
        KinematicResource* current = resource();
        if (current == nullptr || redundancyCombo_ == nullptr) {
            return;
        }
        current->redundancy.mode = redundancyCombo_->currentData().toString().toStdString();
        current->redundancy.travel = weightTravel_->value();
        current->redundancy.midrange = weightMid_->value();
        current->redundancy.manipulability = weightManip_->value();
    };
    connect(redundancyCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, writeRedundancy] {
        if (refreshing_ || redundancyCombo_ == nullptr) {
            return;
        }
        const QString mode = redundancyCombo_->currentData().toString();
        refreshing_ = true;
        if (mode == QStringLiteral("travel")) {
            weightTravel_->setValue(1);
            weightMid_->setValue(0.05);
            weightManip_->setValue(0);
        } else if (mode == QStringLiteral("midrange")) {
            weightTravel_->setValue(0.05);
            weightMid_->setValue(1);
            weightManip_->setValue(0);
        } else if (mode == QStringLiteral("manipulability")) {
            weightTravel_->setValue(0.05);
            weightMid_->setValue(0.05);
            weightManip_->setValue(1);
        }
        refreshing_ = false;
        writeRedundancy();
    });
    connect(weightTravel_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [writeRedundancy](double) { writeRedundancy(); });
    connect(weightMid_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [writeRedundancy](double) { writeRedundancy(); });
    connect(weightManip_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [writeRedundancy](double) { writeRedundancy(); });
    connect(ikCombo_, &QComboBox::currentTextChanged, this, [this](const QString& text) {
        if (refreshing_) {
            return;
        }
        if (KinematicResource* current = resource()) {
            current->ik = text.toStdString();
        }
    });
    connect(addPoint, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        RobotPath* path = nullptr;
        const std::string name = pathName_->text().toStdString();
        for (RobotPath& candidate : scene_.robotPaths) {
            if (candidate.name == name) {
                path = &candidate;
            }
        }
        if (path == nullptr) {
            RobotPath created;
            created.name = name;
            created.resource = current->name;
            scene_.robotPaths.push_back(std::move(created));
            path = &scene_.robotPaths.back();
        }
        path->external = pathExternalCombo_->currentData().toString().toStdString();
        PathPoint point;
        point.name = "p" + std::to_string(path->points.size() + 1);
        pathMoveFromName(moveKind_->currentText().toStdString(), point.move);
        point.speed = pathSpeed_->value();
        point.zone = pathZone_->value();
        point.joints = jointVector(*current);
        point.external = path->external;
        for (const std::string& token : splitComma(externalTargets_->text())) {
            try {
                point.externalValues.push_back(std::stod(token));
            } catch (const std::exception&) {
                liveLabel_->setText(trk("motion.externalValues"));
                return;
            }
        }
        if (point.move != PathMove::Ptp) {
            point.cartesian = true;
            const CartesianPose pose = forwardModel(*current);
            point.position = pose.position + Vec3{goalSpin_[0]->value(), goalSpin_[1]->value(), goalSpin_[2]->value()};
            point.orientation = pose.rotation;
            if (point.move == PathMove::Circ) {
                point.via = (pose.position + point.position) * 0.5 + Vec3{0, 40, 0};
            }
        }
        path->points.push_back(point);
        pathList_->addItem(QString::fromStdString(point.name + " " + pathMoveName(point.move)));
    });
    connect(plan, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        const RobotPath* path = nullptr;
        for (const RobotPath& candidate : scene_.robotPaths) {
            if (candidate.name == pathName_->text().toStdString()) {
                path = &candidate;
            }
        }
        if (path == nullptr) {
            return;
        }
        const MotionPlan result = planRobotPath(scene_, *current, *path);
        QString text = QString::number(result.seconds, 'f', 2) + QStringLiteral(" s");
        for (const std::string& warning : result.warnings) {
            text += QStringLiteral("\n") + QString::fromStdString(warning);
        }
        for (const auto& [a, b] : result.collisions) {
            text += QStringLiteral("\n") + QString::fromStdString(a + " / " + b);
        }
        planLabel_->setText(text);
        if (!result.collisions.empty()) {
            highlightCollisions(scene_, *current, result.collisions);
        }
        if (viewport_ != nullptr) {
            viewport_->update();
        }
    });
    connect(playPath, &QPushButton::clicked, this, [this] {
        if (play_) {
            play_(pathName_->text().toStdString());
        }
    });
    connect(reach, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr) {
            return;
        }
        CartesianPose target = forwardModel(*current);
        target.position = target.position + Vec3{goalSpin_[0]->value(), goalSpin_[1]->value(), goalSpin_[2]->value()};
        const ReachReport report = analyzeReach(*current, target, 9);
        liveLabel_->setText(trk("motion.reachResult").arg(report.reachable).arg(report.samples));
    });
    connect(attach, &QPushButton::clicked, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr || scene_.selectedId().empty()) {
            return;
        }
        std::string error;
        if (!attachTool(scene_, *current, scene_.selectedId(), error)) {
            liveLabel_->setText(QString::fromStdString(error));
        }
    });
    connect(detach, &QPushButton::clicked, this, [this] {
        if (scene_.selectedId().empty()) {
            return;
        }
        std::string error;
        if (!detachTool(scene_, scene_.selectedId(), error)) {
            liveLabel_->setText(QString::fromStdString(error));
        }
    });
    robotButtons->addWidget(addPoint);
    robotButtons->addWidget(plan);
    robotButtons->addWidget(playPath);
    robotButtons->addWidget(reach);
    robotButtons->addWidget(attach);
    robotButtons->addWidget(detach);
    robotLayout->addLayout(robotButtons);
    tabs_->addTab(robotPage, trk("motion.tab.robot"));

    connect(resourceCombo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { fillCombos(); });
}

void MotionPanel::ensureMoveTimer() {
    if (moveTimer_ != nullptr) {
        return;
    }
    moveTimer_ = new QTimer(this);
    moveTimer_->setInterval(16);
    connect(moveTimer_, &QTimer::timeout, this, [this] {
        KinematicResource* current = resource();
        if (current == nullptr || !move_.active) {
            moveTimer_->stop();
            return;
        }
        advanceMove(*current, move_, 1.0 / 60.0);
        applyResource(scene_, *current);
        refreshLive();
        if (viewport_ != nullptr) {
            viewport_->update();
        }
        if (!move_.active) {
            moveTimer_->stop();
        }
    });
}

void MotionPanel::jog(double delta) {
    KinematicResource* current = resource();
    if (current == nullptr) {
        return;
    }
    std::string error;
    if (!jogJoint(*current, jointCombo_->currentText().toStdString(), delta, error)) {
        liveLabel_->setText(QString::fromStdString(error));
        return;
    }
    applyResource(scene_, *current);
    refreshLive();
    if (viewport_ != nullptr) {
        viewport_->update();
    }
}

void MotionPanel::notePick(Vec3 point, Vec3 normal, const std::string& nodeId) {
    if (faceAxisButton_ != nullptr && faceAxisButton_->isChecked() && !nodeId.empty()) {
        Vec3 origin;
        Vec3 direction;
        double radius = 0;
        if (axisFromCylindricalFace(scene_, nodeId, point, normal, origin, direction, radius)) {
            axisSpin_[0]->setValue(direction.x);
            axisSpin_[1]->setValue(direction.y);
            axisSpin_[2]->setValue(direction.z);
            anchorSpin_[0]->setValue(origin.x);
            anchorSpin_[1]->setValue(origin.y);
            anchorSpin_[2]->setValue(origin.z);
            if (KinematicResource* current = resource()) {
                if (KinJoint* joint = findJoint(*current, jointCombo_->currentText().toStdString())) {
                    joint->axis = normalize(direction);
                    joint->anchor = origin;
                    applyResource(scene_, *current);
                    if (viewport_ != nullptr) {
                        viewport_->update();
                    }
                }
            }
            pickLabel_->setText(trk("motion.faceAxis") +
                                QStringLiteral("  r %1  (%2, %3, %4)")
                                    .arg(radius, 0, 'f', 1)
                                    .arg(origin.x, 0, 'f', 1)
                                    .arg(origin.y, 0, 'f', 1)
                                    .arg(origin.z, 0, 'f', 1));
            return;
        }
    }
    if (pickCount_ < 2) {
        picks_[pickCount_++] = point;
    } else {
        picks_[0] = picks_[1];
        picks_[1] = point;
    }
    pickLabel_->setText(QStringLiteral("A %1 %2 %3   B %4 %5 %6")
                            .arg(picks_[0].x, 0, 'f', 1)
                            .arg(picks_[0].y, 0, 'f', 1)
                            .arg(picks_[0].z, 0, 'f', 1)
                            .arg(pickCount_ > 1 ? picks_[1].x : 0, 0, 'f', 1)
                            .arg(pickCount_ > 1 ? picks_[1].y : 0, 0, 'f', 1)
                            .arg(pickCount_ > 1 ? picks_[1].z : 0, 0, 'f', 1));
}

void MotionPanel::fillSolids() {
    if (solidList_ == nullptr || scene_.root() == nullptr) {
        return;
    }
    solidList_->blockSignals(true);
    solidList_->clear();
    std::vector<std::shared_ptr<SceneNode>> parts;
    std::vector<std::shared_ptr<SceneNode>> stack{scene_.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->kind == NodeKind::Part && !node->meshId.empty()) {
            parts.push_back(node);
        }
    }
    std::sort(parts.begin(), parts.end(), [](const std::shared_ptr<SceneNode>& a, const std::shared_ptr<SceneNode>& b) {
        return a->name < b->name;
    });
    for (const auto& part : parts) {
        auto* item = new QListWidgetItem(QString::fromStdString(part->name));
        item->setData(Qt::UserRole, QString::fromStdString(part->id));
        solidList_->addItem(item);
    }
    solidList_->blockSignals(false);
    syncSelection();
}

void MotionPanel::syncSelection() {
    if (solidList_ == nullptr) {
        return;
    }
    const auto& ids = scene_.selection();
    solidList_->blockSignals(true);
    for (int i = 0; i < solidList_->count(); ++i) {
        QListWidgetItem* item = solidList_->item(i);
        const std::string id = item->data(Qt::UserRole).toString().toStdString();
        item->setSelected(std::find(ids.begin(), ids.end(), id) != ids.end());
    }
    solidList_->blockSignals(false);
}

void MotionPanel::fillCombos() {
    parentCombo_->clear();
    childCombo_->clear();
    jointCombo_->clear();
    coupleCombo_->clear();
    poseCombo_->clear();
    coupleCombo_->addItem(QString());
    KinematicResource* current = resource();
    if (current == nullptr) {
        return;
    }
    for (const KinLink& link : current->links) {
        parentCombo_->addItem(QString::fromStdString(link.name));
        childCombo_->addItem(QString::fromStdString(link.name));
    }
    for (const KinJoint& joint : current->joints) {
        jointCombo_->addItem(QString::fromStdString(joint.name));
        coupleCombo_->addItem(QString::fromStdString(joint.name));
    }
    for (const KinPose& pose : current->poses) {
        poseCombo_->addItem(QString::fromStdString(pose.name));
    }
    const QString ik = QString::fromStdString(current->ik);
    const int ikIndex = ikCombo_->findText(ik);
    if (ikIndex >= 0) {
        ikCombo_->setCurrentIndex(ikIndex);
    }
    if (redundancyCombo_ != nullptr) {
        const QSignalBlocker blockCombo(redundancyCombo_);
        const QSignalBlocker blockTravel(weightTravel_);
        const QSignalBlocker blockMid(weightMid_);
        const QSignalBlocker blockManip(weightManip_);
        const int modeIndex = redundancyCombo_->findData(QString::fromStdString(current->redundancy.mode));
        redundancyCombo_->setCurrentIndex(modeIndex >= 0 ? modeIndex : 0);
        weightTravel_->setValue(current->redundancy.travel);
        weightMid_->setValue(current->redundancy.midrange);
        weightManip_->setValue(current->redundancy.manipulability);
    }
    refreshLive();
}

void MotionPanel::refresh() {
    refreshing_ = true;
    const QString previous = resourceCombo_->currentText();
    resourceCombo_->clear();
    for (const KinematicResource& item : scene_.resources) {
        resourceCombo_->addItem(QString::fromStdString(item.name));
    }
    const int index = resourceCombo_->findText(previous);
    if (index >= 0) {
        resourceCombo_->setCurrentIndex(index);
    }
    blockCombo_->clear();
    for (const LogicBlock& block : scene_.logicBlocks) {
        blockCombo_->addItem(QString::fromStdString(block.name));
    }
    if (blockCombo_->count() > 0 && python_->toPlainText().isEmpty()) {
        python_->setPlainText(QString::fromStdString(scene_.logicBlocks.front().python));
    }
    fillCombos();
    fillSolids();
    refreshing_ = false;
}

void MotionPanel::refreshLive() {
    KinematicResource* current = resource();
    if (current == nullptr) {
        liveLabel_->setText(QString());
        return;
    }
    QString text;
    for (const KinJoint& joint : current->joints) {
        if (!text.isEmpty()) {
            text += QStringLiteral("  ");
        }
        text += QString::fromStdString(joint.name) + QStringLiteral("=") + QString::number(jointCommand(*current, joint), 'f', 1);
    }
    if (!suggestNote_.empty()) {
        if (!text.isEmpty()) {
            text += QStringLiteral("\n");
        }
        text += QString::fromStdString(suggestNote_);
    }
    liveLabel_->setText(text);
}

}  // namespace sim
