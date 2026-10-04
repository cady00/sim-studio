#include "sim/app/PhysicsShelf.hpp"

#include "sim/core/Scene.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QTableWidget>
#include <QVBoxLayout>

namespace sim {
namespace {

QDoubleSpinBox* spin(double minValue, double maxValue, int decimals) {
    auto* box = new QDoubleSpinBox;
    box->setRange(minValue, maxValue);
    box->setDecimals(decimals);
    box->setSingleStep(decimals >= 3 ? 0.01 : 1);
    return box;
}

}  // namespace

PhysicsShelf::PhysicsShelf(Scene& scene, std::function<QString(const char*)> translate, QWidget* parent)
    : QWidget(parent), scene_(scene), translate_(std::move(translate)) {
    auto* host = new QWidget;
    auto* form = new QFormLayout(host);
    form->setContentsMargins(8, 8, 8, 8);
    enabled_ = new QCheckBox;
    timestep_ = spin(0.001, 0.05, 4);
    engine_ = new QComboBox;
    engine_->addItem("jolt");
    engine_->addItem("builtin");
    reference_ = spin(0.01, 5, 3);
    stats_ = new QLabel;
    stats_->setWordWrap(true);
    motion_ = new QComboBox;
    surface_ = new QLineEdit;
    density_ = spin(1, 20000, 1);
    margin_ = spin(0, 20, 3);
    linearDamp_ = spin(0, 10, 3);
    angularDamp_ = spin(0, 10, 3);
    continuous_ = new QCheckBox;
    sleep_ = new QCheckBox;
    ride_ = new QCheckBox;
    join_ = new QCheckBox;
    handoff_ = new QCheckBox;
    grip_ = spin(0, 5, 3);
    guide_ = spin(0, 1.0e5, 1);
    next_ = new QLineEdit;
    motorName_ = new QLineEdit;
    target_ = spin(-1.0e6, 1.0e6, 1);
    accel_ = spin(0, 1.0e7, 1);
    decel_ = spin(0, 1.0e7, 1);
    running_ = new QCheckBox;
    pulse_ = spin(0.01, 1.0e6, 2);
    gray_ = new QCheckBox;
    auto* applyEngine = new QPushButton;
    applyEngine->setObjectName("phys.apply");
    grades_ = new QTableWidget(0, 3);
    grades_->horizontalHeader()->setStretchLastSection(true);
    pairs_ = new QTableWidget(0, 4);
    pairs_->horizontalHeader()->setStretchLastSection(true);
    auto* addGrade = new QPushButton;
    addGrade->setObjectName("phys.addGrade");
    auto* addPair = new QPushButton;
    addPair->setObjectName("phys.addPair");

    form->addRow("enabled", enabled_);
    form->addRow("step", timestep_);
    form->addRow("engine", engine_);
    form->addRow("apply", applyEngine);
    form->addRow("reference", reference_);
    form->addRow("stats", stats_);
    form->addRow("motion", motion_);
    form->addRow("surface", surface_);
    form->addRow("density", density_);
    form->addRow("margin", margin_);
    form->addRow("linear", linearDamp_);
    form->addRow("angular", angularDamp_);
    form->addRow("ccd", continuous_);
    form->addRow("sleep", sleep_);
    form->addRow("ride", ride_);
    form->addRow("join", join_);
    form->addRow("handoff", handoff_);
    form->addRow("grip", grip_);
    form->addRow("guide", guide_);
    form->addRow("next", next_);
    form->addRow("motor", motorName_);
    form->addRow("target", target_);
    form->addRow("accel", accel_);
    form->addRow("decel", decel_);
    form->addRow("running", running_);
    form->addRow("pulse", pulse_);
    form->addRow("gray", gray_);
    form->addRow("grades", grades_);
    form->addRow(addGrade);
    form->addRow("pairs", pairs_);
    form->addRow(addPair);

    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(host);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(scroll);

    const auto touch = [this] {
        if (!refreshing_) {
            writeDocument();
            writeSelection();
        }
    };
    connect(enabled_, &QCheckBox::toggled, this, touch);
    connect(timestep_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(reference_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(motion_, qOverload<int>(&QComboBox::currentIndexChanged), this, touch);
    connect(surface_, &QLineEdit::editingFinished, this, touch);
    connect(density_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(margin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(linearDamp_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(angularDamp_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(continuous_, &QCheckBox::toggled, this, touch);
    connect(sleep_, &QCheckBox::toggled, this, touch);
    connect(ride_, &QCheckBox::toggled, this, touch);
    connect(join_, &QCheckBox::toggled, this, touch);
    connect(handoff_, &QCheckBox::toggled, this, touch);
    connect(grip_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(guide_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(next_, &QLineEdit::editingFinished, this, touch);
    connect(motorName_, &QLineEdit::editingFinished, this, touch);
    connect(target_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(accel_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(decel_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(running_, &QCheckBox::toggled, this, touch);
    connect(pulse_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, touch);
    connect(gray_, &QCheckBox::toggled, this, touch);
    connect(grades_, &QTableWidget::itemChanged, this, [this] { if (!refreshing_) writeTables(); });
    connect(pairs_, &QTableWidget::itemChanged, this, [this] { if (!refreshing_) writeTables(); });
    connect(addGrade, &QPushButton::clicked, this, [this] {
        SurfaceGrade grade;
        grade.name = "grade" + std::to_string(scene_.physicsDoc.grades.size() + 1);
        scene_.physicsDoc.grades.push_back(grade);
        reload();
    });
    connect(addPair, &QPushButton::clicked, this, [this] {
        GradePair pair;
        pair.a = "a";
        pair.b = "b";
        scene_.physicsDoc.pairs.push_back(pair);
        reload();
    });
    connect(applyEngine, &QPushButton::clicked, this, [this] {
        scene_.physicsDoc.engine = engine_->currentText().toStdString();
        if (onEngine_) {
            onEngine_(scene_.physicsDoc.engine);
        }
    });
    reload();
}

QString PhysicsShelf::trk(const char* key) const {
    return translate_ ? translate_(key) : QString::fromUtf8(key);
}

void PhysicsShelf::setStats(const PhysicsStats& stats) {
    stats_->setText(trk("phys.statsLine")
                        .arg(stats.contacts)
                        .arg(stats.detectMs, 0, 'f', 2)
                        .arg(stats.solveMs, 0, 'f', 2)
                        .arg(stats.continuous));
}

MotorAspect* PhysicsShelf::selectedMotor() {
    auto node = scene_.find(scene_.selectedId());
    if (!node) {
        return nullptr;
    }
    const std::string name = motorName_->text().trimmed().toUtf8().toStdString();
    if (name.empty()) {
        return nullptr;
    }
    for (MotorAspect& motor : scene_.motors) {
        if (motor.name == name) {
            return &motor;
        }
    }
    MotorAspect created;
    created.name = name;
    created.owner = node->name;
    scene_.motors.push_back(std::move(created));
    return &scene_.motors.back();
}

void PhysicsShelf::writeDocument() {
    scene_.physicsDoc.enabled = enabled_->isChecked();
    scene_.physicsDoc.timestep = timestep_->value();
    scene_.physicsDoc.engine = engine_->currentText().toStdString();
    scene_.physicsDoc.referenceFriction = reference_->value();
}

void PhysicsShelf::writeSelection() {
    auto node = scene_.find(scene_.selectedId());
    if (!node || node->id == "root") {
        return;
    }
    BodyMotion motion = BodyMotion::Free;
    if (motion_->currentData().isValid()) {
        bodyMotionFromName(motion_->currentData().toString().toStdString(), motion);
    }
    node->mech.motion = motion;
    syncMotionFlags(node->mech);
    node->mech.surface = surface_->text().trimmed().toUtf8().toStdString();
    node->mech.density = density_->value();
    node->mech.margin = margin_->value();
    node->mech.linearDamp = linearDamp_->value();
    node->mech.angularDamp = angularDamp_->value();
    node->mech.continuous = continuous_->isChecked();
    node->mech.allowSleep = sleep_->isChecked();
    node->mech.surfaceRide = ride_->isChecked();
    node->mech.joinAncestor = join_->isChecked();
    node->mech.handoff = handoff_->isChecked();
    node->mech.grip = grip_->value();
    node->mech.guide = guide_->value();
    node->mech.nextName = next_->text().trimmed().toUtf8().toStdString();
    node->mech.motor = motorName_->text().trimmed().toUtf8().toStdString();
    if (MotorAspect* motor = selectedMotor()) {
        motor->owner = node->name;
        motor->targetSpeed = target_->value();
        motor->accel = accel_->value();
        motor->decel = decel_->value();
        motor->running = running_->isChecked();
        motor->pulseDistance = pulse_->value();
        motor->grayCode = gray_->isChecked();
    }
}

void PhysicsShelf::writeTables() {
    scene_.physicsDoc.grades.clear();
    for (int row = 0; row < grades_->rowCount(); ++row) {
        SurfaceGrade grade;
        auto cell = [&](int column) {
            auto* item = grades_->item(row, column);
            return item == nullptr ? std::string{} : item->text().trimmed().toUtf8().toStdString();
        };
        grade.name = cell(0);
        grade.friction = grades_->item(row, 1) ? grades_->item(row, 1)->text().toDouble() : 0.4;
        grade.restitution = grades_->item(row, 2) ? grades_->item(row, 2)->text().toDouble() : 0.05;
        if (!grade.name.empty()) {
            scene_.physicsDoc.grades.push_back(std::move(grade));
        }
    }
    scene_.physicsDoc.pairs.clear();
    for (int row = 0; row < pairs_->rowCount(); ++row) {
        GradePair pair;
        auto text = [&](int column) {
            auto* item = pairs_->item(row, column);
            return item == nullptr ? QString{} : item->text().trimmed();
        };
        pair.a = text(0).toUtf8().toStdString();
        pair.b = text(1).toUtf8().toStdString();
        pair.friction = text(2).toDouble();
        pair.restitution = text(3).toDouble();
        if (!pair.a.empty() && !pair.b.empty()) {
            scene_.physicsDoc.pairs.push_back(std::move(pair));
        }
    }
}

void PhysicsShelf::reload() {
    if (enabled_ == nullptr) {
        return;
    }
    QSignalBlocker blockGrades(grades_);
    QSignalBlocker blockPairs(pairs_);
    refreshing_ = true;
    const PhysicsDocument& document = scene_.physicsDoc;
    enabled_->setChecked(document.enabled);
    timestep_->setValue(document.timestep);
    const int engineIndex = engine_->findText(QString::fromStdString(document.engine));
    engine_->setCurrentIndex(engineIndex < 0 ? 0 : engineIndex);
    reference_->setValue(document.referenceFriction);
    setStats(document.stats);
    motion_->clear();
    const BodyMotion motions[] = {BodyMotion::Anchored, BodyMotion::Driven, BodyMotion::Free, BodyMotion::Surface};
    const char* motionKeys[] = {"phys.anchored", "phys.driven", "phys.free", "phys.surface"};
    for (int i = 0; i < 4; ++i) {
        motion_->addItem(trk(motionKeys[i]), QString::fromUtf8(bodyMotionName(motions[i])));
    }
    auto node = scene_.find(scene_.selectedId());
    const bool has = static_cast<bool>(node) && node->id != "root";
    for (QWidget* widget : {static_cast<QWidget*>(motion_), static_cast<QWidget*>(surface_), static_cast<QWidget*>(density_),
                            static_cast<QWidget*>(margin_), static_cast<QWidget*>(linearDamp_), static_cast<QWidget*>(angularDamp_),
                            static_cast<QWidget*>(continuous_), static_cast<QWidget*>(sleep_), static_cast<QWidget*>(ride_),
                            static_cast<QWidget*>(join_), static_cast<QWidget*>(handoff_), static_cast<QWidget*>(grip_),
                            static_cast<QWidget*>(guide_), static_cast<QWidget*>(next_), static_cast<QWidget*>(motorName_)}) {
        widget->setEnabled(has);
    }
    if (has) {
        const int index = motion_->findData(QString::fromUtf8(bodyMotionName(node->mech.motion)));
        motion_->setCurrentIndex(index < 0 ? 2 : index);
        surface_->setText(QString::fromUtf8(node->mech.surface.c_str()));
        density_->setValue(node->mech.density);
        margin_->setValue(node->mech.margin);
        linearDamp_->setValue(node->mech.linearDamp);
        angularDamp_->setValue(node->mech.angularDamp);
        continuous_->setChecked(node->mech.continuous);
        sleep_->setChecked(node->mech.allowSleep);
        ride_->setChecked(node->mech.surfaceRide || node->mech.motion == BodyMotion::Surface);
        join_->setChecked(node->mech.joinAncestor);
        handoff_->setChecked(node->mech.handoff);
        grip_->setValue(node->mech.grip);
        guide_->setValue(node->mech.guide);
        next_->setText(QString::fromUtf8(node->mech.nextName.c_str()));
        motorName_->setText(QString::fromUtf8(node->mech.motor.c_str()));
    }
    const MotorAspect* motor = nullptr;
    if (has) {
        for (const MotorAspect& candidate : scene_.motors) {
            if (candidate.name == node->mech.motor) {
                motor = &candidate;
                break;
            }
        }
    }
    const bool hasMotor = motor != nullptr;
    for (QWidget* widget : {static_cast<QWidget*>(target_), static_cast<QWidget*>(accel_), static_cast<QWidget*>(decel_),
                            static_cast<QWidget*>(running_), static_cast<QWidget*>(pulse_), static_cast<QWidget*>(gray_)}) {
        widget->setEnabled(hasMotor);
    }
    if (hasMotor) {
        target_->setValue(motor->targetSpeed);
        accel_->setValue(motor->accel);
        decel_->setValue(motor->decel);
        running_->setChecked(motor->running);
        pulse_->setValue(motor->pulseDistance);
        gray_->setChecked(motor->grayCode);
    }
    grades_->setRowCount(static_cast<int>(document.grades.size()));
    grades_->setHorizontalHeaderLabels({trk("phys.grade"), trk("phys.friction"), trk("phys.restitution")});
    for (int row = 0; row < grades_->rowCount(); ++row) {
        const SurfaceGrade& grade = document.grades[static_cast<std::size_t>(row)];
        grades_->setItem(row, 0, new QTableWidgetItem(QString::fromUtf8(grade.name.c_str())));
        grades_->setItem(row, 1, new QTableWidgetItem(QString::number(grade.friction, 'f', 3)));
        grades_->setItem(row, 2, new QTableWidgetItem(QString::number(grade.restitution, 'f', 3)));
    }
    pairs_->setRowCount(static_cast<int>(document.pairs.size()));
    pairs_->setHorizontalHeaderLabels({trk("phys.a"), trk("phys.b"), trk("phys.friction"), trk("phys.restitution")});
    for (int row = 0; row < pairs_->rowCount(); ++row) {
        const GradePair& pair = document.pairs[static_cast<std::size_t>(row)];
        pairs_->setItem(row, 0, new QTableWidgetItem(QString::fromUtf8(pair.a.c_str())));
        pairs_->setItem(row, 1, new QTableWidgetItem(QString::fromUtf8(pair.b.c_str())));
        pairs_->setItem(row, 2, new QTableWidgetItem(QString::number(pair.friction, 'f', 3)));
        pairs_->setItem(row, 3, new QTableWidgetItem(QString::number(pair.restitution, 'f', 3)));
    }
    if (auto* form = findChild<QFormLayout*>()) {
        const char* labels[] = {"phys.enabled", "phys.step", "phys.engine", "phys.applyEngine", "phys.reference", "phys.stats",
                                "phys.motion", "phys.gradeName", "phys.density", "phys.margin", "phys.linear", "phys.angular",
                                "phys.ccd", "phys.sleep", "phys.ride", "phys.join", "phys.handoff", "phys.grip", "phys.guide",
                                "phys.next", "phys.motor", "phys.target", "phys.accel", "phys.decel", "phys.running", "phys.pulse",
                                "phys.gray", "phys.grades"};
        for (int i = 0; i < form->rowCount() && i < 28; ++i) {
            auto* item = form->itemAt(i, QFormLayout::FieldRole);
            if (item == nullptr || item->widget() == nullptr) {
                continue;
            }
            if (auto* label = qobject_cast<QLabel*>(form->labelForField(item->widget()))) {
                label->setText(trk(labels[i]));
            }
        }
    }
    if (auto* button = findChild<QPushButton*>("phys.apply")) {
        button->setText(trk("phys.applyEngine"));
    }
    if (auto* button = findChild<QPushButton*>("phys.addGrade")) {
        button->setText(trk("phys.addGrade"));
    }
    if (auto* button = findChild<QPushButton*>("phys.addPair")) {
        button->setText(trk("phys.addPair"));
    }
    refreshing_ = false;
}

}  // namespace sim
