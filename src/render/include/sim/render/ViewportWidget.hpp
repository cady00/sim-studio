#pragma once

#include "sim/core/Scene.hpp"
#include "sim/render/Camera.hpp"

#include <QOpenGLWidget>
#include <QImage>
#include <QPoint>
#include <QRect>
#include <QString>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
    void setSimTime(double seconds);
    void setPhysicsLine(const QString& text);
    void setCallout(const QString& text);
    void setMeasureMode(bool enabled);
    void setPreviewMode(bool enabled);
    // Translucent copy of a mesh. The scene itself is not moved.
    void setGhost(bool enabled);
    void setDarkChrome(bool dark);
    // Reads the widget framebuffer through IRenderer::captureRgba.
    QImage captureView();
    void setForceDetail(bool force) { forceDetail_ = force; }
    double lastDrawMs() const { return drawMs_; }
    int lastSubmitted() const;
    int lastCulled() const { return culledFrame_; }
    int lastDraws() const;
    int residentMeshes() const { return static_cast<int>(resident_.size()); }
    int lastDrawnTriangles() const { return drawnTriangles_; }

signals:
    void selectionChanged(const QString& id);
    void pointPicked(double x, double y, double z, double nx, double ny, double nz, const QString& nodeId);
    void transformEdited();
    void glError(const QString& message);
    void overlayCommand(const QString& id);

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
    void paintOverlay();
    QRect cubeRect_;
    QRect faceTop_;
    QRect faceFront_;
    QRect faceRight_;
    QRect toolbarRect_;
    QString callout_;
    bool measureMode_ = false;
    bool previewMode_ = false;
    bool ghost_ = false;
    bool darkChrome_ = true;
    bool cubePress_ = false;
    bool cubeDrag_ = false;
    QPoint cubePressPos_;
    QString ghostCaption_;
    double simTime_ = 0;
    QString physicsLine_;
    double fps_ = 0;
    int triangles_ = 0;
    double drawMs_ = 0;
    int culledFrame_ = 0;
    int drawnTriangles_ = 0;
    bool forceDetail_ = false;
    std::unordered_set<std::string> resident_;
    std::unordered_map<std::string, std::shared_ptr<MeshData>> lodCache_;
    qint64 lastFrameMs_ = 0;
    std::vector<Vec3> measurePoints_;
};

}  // namespace sim
