#pragma once

#include "sim/core/Scene.hpp"
#include "sim/render/Camera.hpp"

#include <QOpenGLWidget>
#include <QImage>
#include <QPoint>
#include <QString>

#include <cstdint>
#include <memory>
#include <vector>

class QMouseEvent;
class QWheelEvent;

namespace sim {

enum class GizmoMode { Translate, Rotate };

class IRenderer;

class ViewportWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

    void setScene(Scene* scene);
    OrbitCamera& camera() { return camera_; }
    void fitAll();
    void setStandardView(StandardView view);
    void setGizmoMode(GizmoMode mode);
    GizmoMode gizmoMode() const { return gizmoMode_; }
    // Reads the widget framebuffer through IRenderer::captureRgba.
    QImage captureView();

signals:
    void selectionChanged(const QString& id);
    void transformEdited();
    void glError(const QString& message);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void syncMeshes();
    void drawGrid();
    bool gizmoAxes(Vec3& origin, Vec3& axisX, Vec3& axisY, Vec3& axisZ, double& length) const;
    int pickGizmo(double x, double y) const;
    void applyGizmoDrag(double x, double y);

    Scene* scene_ = nullptr;
    OrbitCamera camera_;
    GizmoMode gizmoMode_ = GizmoMode::Translate;
    std::unique_ptr<IRenderer> renderer_;
    std::uint64_t syncedRevision_ = 0;
    QString glError_;
    bool draggingCamera_ = false;
    bool panning_ = false;
    bool draggingGizmo_ = false;
    int gizmoAxis_ = -1;
    QPoint lastPos_;
    Vec3 gizmoDragOrigin_;
    Vec3 gizmoDragDirection_;
    double gizmoDragParameter_ = 0;
    Vec3 gizmoRotateStart_;
    Quat gizmoRotateBasis_;
    Vec3 gizmoRotateTranslation_;
};

}  // namespace sim
