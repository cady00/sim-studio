#pragma once

#include "sim/core/PhysicsModel.hpp"

#include <QWidget>

#include <functional>
#include <string>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QTableWidget;

namespace sim {

class Scene;
struct PhysicsStats;

// Document grades, the selected body's motion, and one motor. Strings come from the app catalog.
class PhysicsShelf : public QWidget {
    Q_OBJECT

public:
    explicit PhysicsShelf(Scene& scene, std::function<QString(const char*)> translate, QWidget* parent = nullptr);
    void reload();
    void setStats(const PhysicsStats& stats);
    void setEngineRequested(std::function<void(const std::string&)> handler) { onEngine_ = std::move(handler); }

private:
    QString trk(const char* key) const;
    void writeDocument();
    void writeSelection();
    void writeTables();
    MotorAspect* selectedMotor();

    Scene& scene_;
    std::function<QString(const char*)> translate_;
    std::function<void(const std::string&)> onEngine_;
    bool refreshing_ = false;
    QCheckBox* enabled_ = nullptr;
    QDoubleSpinBox* timestep_ = nullptr;
    QComboBox* engine_ = nullptr;
    QDoubleSpinBox* reference_ = nullptr;
    QLabel* stats_ = nullptr;
    QComboBox* motion_ = nullptr;
    QLineEdit* surface_ = nullptr;
    QDoubleSpinBox* density_ = nullptr;
    QDoubleSpinBox* margin_ = nullptr;
    QDoubleSpinBox* linearDamp_ = nullptr;
    QDoubleSpinBox* angularDamp_ = nullptr;
    QCheckBox* continuous_ = nullptr;
    QCheckBox* sleep_ = nullptr;
    QCheckBox* ride_ = nullptr;
    QCheckBox* join_ = nullptr;
    QCheckBox* handoff_ = nullptr;
    QDoubleSpinBox* grip_ = nullptr;
    QDoubleSpinBox* guide_ = nullptr;
    QLineEdit* next_ = nullptr;
    QLineEdit* motorName_ = nullptr;
    QDoubleSpinBox* target_ = nullptr;
    QDoubleSpinBox* accel_ = nullptr;
    QDoubleSpinBox* decel_ = nullptr;
    QCheckBox* running_ = nullptr;
    QDoubleSpinBox* pulse_ = nullptr;
    QCheckBox* gray_ = nullptr;
    QTableWidget* grades_ = nullptr;
    QTableWidget* pairs_ = nullptr;
};

}  // namespace sim
