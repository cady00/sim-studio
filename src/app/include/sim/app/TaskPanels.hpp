#pragma once

#include <QWidget>

namespace sim {

class Scene;

// Reads the logic blocks already stored on the scene. Nodes are not a second model.
class LogicCanvas : public QWidget {
    Q_OBJECT

public:
    explicit LogicCanvas(Scene& scene, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    Scene& scene_;
};

// Path timeline for the first robot path: points, move types, and a time ruler.
class PathTimeline : public QWidget {
    Q_OBJECT

public:
    explicit PathTimeline(Scene& scene, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    Scene& scene_;
};

}  // namespace sim
