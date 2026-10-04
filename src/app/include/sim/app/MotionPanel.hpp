#pragma once

#include "sim/core/I18n.hpp"
#include "sim/core/Math.hpp"
#include "sim/motion/Motion.hpp"

#include <QWidget>

#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QSlider;
class QTabWidget;
class QTimer;

namespace sim {

class MotionPanel : public QWidget {
    Q_OBJECT

public:
    MotionPanel(Scene& scene, I18nCatalog& i18n, LogicMachine& logic, BuiltinRobotSolver& robot, QWidget* parent = nullptr);

    void retranslate();
    void refresh();
    void refreshLive();
    void syncSelection();
    void notePick(Vec3 point, Vec3 normal = {}, const std::string& nodeId = {});
    void setViewport(QWidget* viewport) { viewport_ = viewport; }
    void setPlayRequest(std::function<void(const std::string& path)> play) { play_ = std::move(play); }

signals:
    void pythonRequested(const QString& source);

private:
    void buildUi();
    QString trk(const char* key) const;
    void tag(QWidget* widget, const char* key);
    KinematicResource* resource();
    void fillCombos();
    void fillSolids();
    void jog(double delta);
    void ensureMoveTimer();
    void syncJointSlider();
    std::string selectPickedLink(const std::string& nodeId);

    Scene& scene_;
    I18nCatalog& i18n_;
    LogicMachine& logic_;
    BuiltinRobotSolver& robot_;
    QWidget* viewport_ = nullptr;
    std::function<void(const std::string&)> play_;
    bool refreshing_ = false;
    Vec3 picks_[2];
    int pickCount_ = 0;
    ActiveMove move_;
    QTimer* moveTimer_ = nullptr;

    QTabWidget* tabs_ = nullptr;
    QComboBox* resourceCombo_ = nullptr;
    QComboBox* catalogCombo_ = nullptr;
    QLineEdit* resourceName_ = nullptr;
    QLineEdit* linkName_ = nullptr;
    QComboBox* parentCombo_ = nullptr;
    QComboBox* childCombo_ = nullptr;
    QComboBox* motionCombo_ = nullptr;
    QComboBox* jointCombo_ = nullptr;
    QSlider* jointSlider_ = nullptr;
    QListWidget* poseList_ = nullptr;
    QDoubleSpinBox* axisSpin_[3] = {};
    QDoubleSpinBox* anchorSpin_[3] = {};
    QDoubleSpinBox* lowerSpin_ = nullptr;
    QDoubleSpinBox* upperSpin_ = nullptr;
    QDoubleSpinBox* speedSpin_ = nullptr;
    QDoubleSpinBox* accelSpin_ = nullptr;
    QComboBox* coupleCombo_ = nullptr;
    QDoubleSpinBox* ratioSpin_ = nullptr;
    QLineEdit* poseName_ = nullptr;
    QComboBox* poseCombo_ = nullptr;
    QLineEdit* carrierName_ = nullptr;
    QLineEdit* mountLinkName_ = nullptr;
    QComboBox* externalKindCombo_ = nullptr;
    QComboBox* redundancyCombo_ = nullptr;
    QDoubleSpinBox* weightTravel_ = nullptr;
    QDoubleSpinBox* weightMid_ = nullptr;
    QDoubleSpinBox* weightManip_ = nullptr;
    QComboBox* profileCombo_ = nullptr;
    QLabel* pickLabel_ = nullptr;
    QLabel* liveLabel_ = nullptr;
    QLabel* solidLabel_ = nullptr;
    QListWidget* solidList_ = nullptr;
    QPushButton* faceAxisButton_ = nullptr;
    std::string suggestNote_;

    QLineEdit* blockName_ = nullptr;
    QLineEdit* runSignal_ = nullptr;
    QLineEdit* inputs_ = nullptr;
    QLineEdit* outputs_ = nullptr;
    QComboBox* actionKind_ = nullptr;
    QLineEdit* actionText_ = nullptr;
    QDoubleSpinBox* actionSeconds_ = nullptr;
    QListWidget* actionList_ = nullptr;
    QPlainTextEdit* python_ = nullptr;
    QComboBox* blockCombo_ = nullptr;

    QComboBox* ikCombo_ = nullptr;
    QDoubleSpinBox* tcpSpin_[3] = {};
    QComboBox* moveKind_ = nullptr;
    QDoubleSpinBox* pathSpeed_ = nullptr;
    QDoubleSpinBox* pathZone_ = nullptr;
    QDoubleSpinBox* goalSpin_[3] = {};
    QLineEdit* pathName_ = nullptr;
    QComboBox* pathExternalCombo_ = nullptr;
    QLineEdit* externalTargets_ = nullptr;
    QListWidget* pathList_ = nullptr;
    QLabel* planLabel_ = nullptr;
};

}  // namespace sim
