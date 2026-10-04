#pragma once

#include <QWidget>

class QTreeWidget;
class QLabel;

namespace sim {

class Scene;

class FloorPlanView : public QWidget {
public:
    explicit FloorPlanView(Scene& scene, QWidget* parent = nullptr);
    void setDark(bool dark);
    void setZh(bool zh);
    void reload();

private:
    Scene& scene_;
    QWidget* canvas_ = nullptr;
};

class OperationTreeView : public QWidget {
public:
    explicit OperationTreeView(Scene& scene, QWidget* parent = nullptr);
    void setDark(bool dark);
    void setZh(bool zh);
    void reload();

private:
    Scene& scene_;
    QTreeWidget* tree_ = nullptr;
    QLabel* badge_ = nullptr;
    bool zh_ = true;
};

class GanttView : public QWidget {
public:
    explicit GanttView(Scene& scene, QWidget* parent = nullptr);
    void setDark(bool dark);
    void setZh(bool zh);
    void reload();

private:
    Scene& scene_;
    QWidget* canvas_ = nullptr;
    bool zh_ = true;
};

}  // namespace sim
