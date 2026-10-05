#pragma once

#include <QPointF>
#include <QWidget>

#include "sim/motion/Scope.hpp"

#include <map>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGraphicsScene;
class QGraphicsView;
class QLabel;
class QListWidget;
class QPlainTextEdit;
class QScrollArea;
class QPushButton;
class QTableWidget;

namespace sim {
class Scene;
class OpcUaClient;
class OpcUaServer;
}

namespace sim {

class Scene;
struct MotionPlan;

// Node graph over the logic blocks already stored on the scene.
// Layout pins live in the widget. The project file stays format 6.
class LogicCanvas : public QWidget {
    Q_OBJECT

public:
    explicit LogicCanvas(Scene& scene, QWidget* parent = nullptr);
    void reload();
    void setDark(bool dark);
    void remember(const std::string& key, const QPointF& position);
    void refreshEdges();
    void finishWire(const std::string& fromKey, const QPointF& scenePos);
    void editKey(const std::string& key);
    // keys look like "0/s1". hotSignals are bool signals that are currently true.
    void setLive(const std::vector<std::string>& stepKeys, const std::vector<std::string>& hotSignals);
    bool arranging() const { return arranging_; }

private:
    void addBlock();
    void addInput();
    void addStep();
    void addOutput();
    void renameSelected();
    void applyText();
    void showText();
    void restyle();

    Scene& scene_;
    QGraphicsView* view_ = nullptr;
    QGraphicsScene* graph_ = nullptr;
    QPlainTextEdit* source_ = nullptr;
    int selected_ = 0;
    bool arranging_ = false;
    bool dark_ = true;
    std::map<std::string, QPointF> pins_;
    std::vector<std::string> liveKeys_;
    std::vector<std::string> hotSignals_;
};

// Planner cycle time and joint samples for the first robot path.
class PathTimeline : public QWidget {
    Q_OBJECT

public:
    explicit PathTimeline(Scene& scene, QWidget* parent = nullptr);
    void reload();
    void setDark(bool dark);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    Scene& scene_;
    std::string signature_;
    bool planned_ = false;
    bool ok_ = false;
    double seconds_ = 0;
    std::vector<double> segmentSeconds_;
    std::vector<std::vector<double>> samples_;
    double dt_ = 1.0 / 60.0;
    std::string note_;
    bool dark_ = true;
};

class ScopePlot;

// Plots watched signals and joint commands against simulation time.
// Joint rows can also draw backward-difference velocity and acceleration.
class SignalScope : public QWidget {
    Q_OBJECT

public:
    explicit SignalScope(Scene& scene, QWidget* parent = nullptr);
    void sample(double timeSeconds);
    void clearTrace();
    void reload();
    void setDark(bool dark);
    void setTexts(const QString& velocity, const QString& acceleration, const QString& exportCsv, const QString& clearText,
                  const QString& waiting);

private:
    void syncWatch();
    void exportCsv();
    void refreshReadout();

    Scene& scene_;
    ScopeTrace trace_;
    QListWidget* list_ = nullptr;
    QCheckBox* velocityBox_ = nullptr;
    QCheckBox* accelerationBox_ = nullptr;
    QPushButton* exportButton_ = nullptr;
    QPushButton* clearButton_ = nullptr;
    QLabel* readout_ = nullptr;
    ScopePlot* plot_ = nullptr;
    bool userPicked_ = false;
    bool filling_ = false;
    QString waiting_;
};

// OPC UA opens SecurityPolicy None or Basic256Sha256 on the loopback address.
// PLCSIM Advanced stays an address card until that runtime is installed.
class PlcCards : public QWidget {
    Q_OBJECT

public:
    explicit PlcCards(Scene& scene, QWidget* parent = nullptr);
    ~PlcCards() override;
    bool connectOpcUa();
    // Loopback Basic256Sha256 session with a username, plus a sample signal table.
    bool demonstrateSecure();
    // Copies the mapping table, forces, scan period, and backend into the scene.
    void commit();
    // Fills the debug cards from the scene document.
    void reload();
    // Scrolls the mapping table into view after a robot import.
    void revealMappings();
    // Scrolls the tag table into view.
    void revealTags();
    // Refreshes the monitor column and the scan log without rewriting edits.
    void showRuntime(const std::vector<std::string>& log);
    // Decimal node id, bool only. False when the client has no session.
    bool exchangeOpc(const std::string& nodeId, bool write, bool& value, std::string& error);

signals:
    void signalsImported();

private:
    bool importText(const QString& text);
    void commitBindings();
    void commitForces();
    Scene& scene_;
    QLabel* opcState_ = nullptr;
    QLabel* plcsimState_ = nullptr;
    QLabel* ladderLabel_ = nullptr;
    QCheckBox* enabledBox_ = nullptr;
    QDoubleSpinBox* scanSpin_ = nullptr;
    QComboBox* backendBox_ = nullptr;
    QScrollArea* scroll_ = nullptr;
    QTableWidget* mapTable_ = nullptr;
    QTableWidget* forceTable_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;
    OpcUaClient* client_ = nullptr;
    OpcUaServer* secureServer_ = nullptr;
    bool filling_ = false;
    bool opcReady_ = false;
};

}  // namespace sim
