#pragma once

#include <QPointF>
#include <QWidget>

#include <map>
#include <string>
#include <vector>

class QGraphicsScene;
class QGraphicsView;
class QLabel;

namespace sim {
class Scene;
class OpcUaClient;
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
    bool arranging() const { return arranging_; }

private:
    void addBlock();
    void addInput();
    void addStep();
    void addOutput();
    void renameSelected();

    Scene& scene_;
    QGraphicsView* view_ = nullptr;
    QGraphicsScene* graph_ = nullptr;
    int selected_ = 0;
    bool arranging_ = false;
    bool dark_ = true;
    std::map<std::string, QPointF> pins_;
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

// OPC UA opens a SecurityPolicy None session on the loopback address.
// PLCSIM Advanced stays an address card until that runtime is installed.
class PlcCards : public QWidget {
    Q_OBJECT

public:
    explicit PlcCards(Scene& scene, QWidget* parent = nullptr);
    ~PlcCards() override;
    bool connectOpcUa();

private:
    Scene& scene_;
    QLabel* opcState_ = nullptr;
    OpcUaClient* client_ = nullptr;
};

}  // namespace sim
