#include "sim/render/ViewportWidget.hpp"

#include "sim/render/Renderer.hpp"

#include <QMouseEvent>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace sim {
namespace {

bool selectedBranch(const SceneNode& node, const std::string& selected) {
    if (selected.empty()) {
        return false;
    }
    if (node.id == selected) {
        return true;
    }
    auto parent = node.parent.lock();
    while (parent) {
        if (parent->id == selected) {
            return true;
        }
        parent = parent->parent.lock();
    }
    return false;
}

double closestParameter(Vec3 origin, Vec3 direction, const Ray& ray) {
    const Vec3 between = origin - ray.origin;
    const double b = dot(direction, ray.direction);
    const double d = dot(direction, between);
    const double e = dot(ray.direction, between);
    const double denom = 1.0 - b * b;
    if (std::abs(denom) < 1e-8) {
        return 0;
    }
    return (b * e - d) / denom;
}

void pushLine(LineBatch& batch, Vec3 position, Vec3 color) {
    batch.vertices.push_back(LineVertex{static_cast<float>(position.x), static_cast<float>(position.y),
                                         static_cast<float>(position.z), static_cast<float>(color.x),
                                         static_cast<float>(color.y), static_cast<float>(color.z)});
}

}  // namespace

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    camera_.setStandardView(StandardView::Iso);
}

ViewportWidget::~ViewportWidget() {
    makeCurrent();
    if (renderer_) {
        renderer_->shutdown();
    }
    doneCurrent();
}

void ViewportWidget::setScene(Scene* scene) {
    scene_ = scene;
    syncedRevision_ = 0;
    update();
}

void ViewportWidget::fitAll() {
    if (scene_ != nullptr) {
        const double aspect = height() > 0 ? static_cast<double>(width()) / height() : 1.0;
        camera_.fit(scene_->worldBounds(), aspect);
    }
    update();
}

void ViewportWidget::setStandardView(StandardView view) {
    camera_.setStandardView(view);
    update();
}

void ViewportWidget::setGizmoMode(GizmoMode mode) {
    gizmoMode_ = mode;
    update();
}

void ViewportWidget::initializeGL() {
    renderer_ = createRenderer();
    std::string error;
    if (!renderer_->initialize(error)) {
        glError_ = QString::fromStdString(error);
        emit glError(glError_);
    }
}

void ViewportWidget::resizeGL(int width, int height) {
    if (renderer_) {
        renderer_->resize(width, height);
    }
}

void ViewportWidget::syncMeshes() {
    if (scene_ == nullptr || renderer_ == nullptr || scene_->structureRevision() == syncedRevision_) {
        return;
    }
    renderer_->clearMeshes();
    for (const auto& [id, mesh] : scene_->meshes()) {
        if (mesh) {
            renderer_->uploadMesh(id, *mesh);
        }
    }
    syncedRevision_ = scene_->structureRevision();
}

void ViewportWidget::drawGrid() {
    Aabb bounds;
    if (scene_ != nullptr) {
        bounds = scene_->worldBounds();
    }
    double step = 100.0;
    double half = 1000.0;
    if (bounds.valid) {
        const double extent = std::max({std::abs(bounds.min.x), std::abs(bounds.max.x), std::abs(bounds.min.y),
                                        std::abs(bounds.max.y), bounds.diagonal(), 10.0});
        const double raw = std::pow(10.0, std::floor(std::log10(extent / 8.0)));
        step = std::clamp(raw, 1.0, 1000.0);
        half = step * 16.0;
    }
    LineBatch lines;
    const Vec3 minor{0.22f, 0.24f, 0.28f};
    const Vec3 major{0.34f, 0.37f, 0.44f};
    const int count = static_cast<int>(std::round(half / step));
    for (int i = -count; i <= count; ++i) {
        const double coordinate = i * step;
        const Vec3 color = (i % 5 == 0) ? major : minor;
        pushLine(lines, {coordinate, -half, 0}, color);
        pushLine(lines, {coordinate, half, 0}, color);
        pushLine(lines, {-half, coordinate, 0}, color);
        pushLine(lines, {half, coordinate, 0}, color);
    }
    const double axis = half * 0.35;
    pushLine(lines, {0, 0, 0}, {0.85, 0.28, 0.30});
    pushLine(lines, {axis, 0, 0}, {0.85, 0.28, 0.30});
    pushLine(lines, {0, 0, 0}, {0.30, 0.75, 0.42});
    pushLine(lines, {0, axis, 0}, {0.30, 0.75, 0.42});
    pushLine(lines, {0, 0, 0}, {0.35, 0.55, 0.90});
    pushLine(lines, {0, 0, axis}, {0.35, 0.55, 0.90});
    lines.width = 1.0f;
    renderer_->submitLines(lines);
}

bool ViewportWidget::gizmoAxes(Vec3& origin, Vec3& axisX, Vec3& axisY, Vec3& axisZ, double& axisLength) const {
    if (scene_ == nullptr || scene_->selectedId().empty()) {
        return false;
    }
    const auto node = scene_->find(scene_->selectedId());
    if (!node) {
        return false;
    }
    const Mat4 world = node->worldMatrix();
    origin = transformPoint(world, {});
    axisX = normalize(transformVector(world, {1, 0, 0}));
    axisY = normalize(transformVector(world, {0, 1, 0}));
    axisZ = normalize(transformVector(world, {0, 0, 1}));
    axisLength = std::max(camera_.distance * 0.16, 20.0);
    return length(axisX) > 0.5 && length(axisY) > 0.5 && length(axisZ) > 0.5;
}

int ViewportWidget::pickGizmo(double x, double y) const {
    Vec3 origin, axisX, axisY, axisZ;
    double axisLength = 0;
    if (!gizmoAxes(origin, axisX, axisY, axisZ, axisLength)) {
        return -1;
    }
    const double aspect = height() > 0 ? static_cast<double>(width()) / height() : 1.0;
    const Vec3 axes[3] = {axisX, axisY, axisZ};
    int best = -1;
    double bestDistance = 10.0;
    for (int i = 0; i < 3; ++i) {
        const Vec3 a = camera_.project(origin, aspect, width(), height());
        const Vec3 b = camera_.project(origin + axes[i] * axisLength, aspect, width(), height());
        const double distance = distancePointToSegment2D(x, y, a.x, a.y, b.x, b.y);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = i;
        }
    }
    return best;
}

void ViewportWidget::applyGizmoDrag(double x, double y) {
    auto node = scene_->find(scene_->selectedId());
    if (!node) {
        draggingGizmo_ = false;
        return;
    }
    const Ray ray = camera_.rayFromPixel(x, y, width(), height());
    const Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    const Vec3 localAxis = axes[gizmoAxis_];
    if (gizmoMode_ == GizmoMode::Translate) {
        const double parameter = closestParameter(gizmoDragOrigin_, gizmoDragDirection_, ray);
        const double delta = parameter - gizmoDragParameter_;
        gizmoDragParameter_ = parameter;
        const Vec3 worldDelta = gizmoDragDirection_ * delta;
        const auto parent = node->parent.lock();
        const Mat4 parentWorld = parent ? parent->worldMatrix() : Mat4::identity();
        node->local.translation = node->local.translation + transformVector(inverseAffine(parentWorld), worldDelta);
    } else {
        const double denom = dot(gizmoDragDirection_, ray.direction);
        if (std::abs(denom) < 1e-8) {
            return;
        }
        const double t = dot(gizmoDragDirection_, gizmoDragOrigin_ - ray.origin) / denom;
        const Vec3 hit = ray.origin + ray.direction * t;
        const Vec3 current = hit - gizmoDragOrigin_;
        if (length(current) < 1e-6 || length(gizmoRotateStart_) < 1e-6) {
            return;
        }
        const double angle = std::atan2(dot(gizmoDragDirection_, cross(gizmoRotateStart_, current)),
                                        dot(gizmoRotateStart_, current));
        node->local.rotation = (gizmoRotateBasis_ * Quat::fromAxisAngle(localAxis, angle)).normalized();
        node->local.translation = gizmoRotateTranslation_;
    }
    scene_->notify(ChangeKind::Transform);
    emit transformEdited();
}

void ViewportWidget::paintGL() {
    if (!glError_.isEmpty() || renderer_ == nullptr) {
        return;
    }
    syncMeshes();
    const double aspect = height() > 0 ? static_cast<double>(width()) / height() : 1.0;
    FrameParams frame;
    frame.viewProjection = mul(camera_.projectionMatrix(aspect), camera_.viewMatrix());
    frame.eye = camera_.eye();
    frame.width = std::max(width(), 1);
    frame.height = std::max(height(), 1);
    frame.frustumCull = true;
    renderer_->beginFrame(frame);
    drawGrid();

    if (scene_ != nullptr && scene_->root()) {
        std::vector<RenderInstance> instances;
        std::vector<std::shared_ptr<SceneNode>> stack{scene_->root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (!node->effectivelyVisible() || node->meshId.empty()) {
                continue;
            }
            const auto mesh = scene_->mesh(node->meshId);
            RenderInstance instance;
            instance.meshKey = node->meshId;
            instance.world = node->worldMatrix();
            instance.material.color = node->color;
            instance.material.highlight = selectedBranch(*node, scene_->selectedId()) ? 1.0f : 0.0f;
            instance.worldBounds = mesh ? transformedBounds(mesh->bounds, instance.world) : Aabb{};
            instance.distanceMm = static_cast<float>(length(instance.worldBounds.center() - frame.eye));
            instance.lodLevels = 1;
            instances.push_back(std::move(instance));
        }
        renderer_->submitInstances(instances);
    }

    Vec3 origin, axisX, axisY, axisZ;
    double axisLength = 0;
    if (gizmoAxes(origin, axisX, axisY, axisZ, axisLength)) {
        LineBatch gizmo;
        gizmo.width = 2.5f;
        gizmo.depthTest = false;
        const Vec3 colors[3] = {{0.93, 0.32, 0.32}, {0.30, 0.82, 0.42}, {0.36, 0.58, 0.95}};
        const Vec3 axes[3] = {axisX, axisY, axisZ};
        for (int i = 0; i < 3; ++i) {
            pushLine(gizmo, origin, colors[i]);
            pushLine(gizmo, origin + axes[i] * axisLength, colors[i]);
        }
        if (gizmoMode_ == GizmoMode::Rotate) {
            constexpr int kSegments = 48;
            for (int axis = 0; axis < 3; ++axis) {
                const Vec3 normal = axes[axis];
                Vec3 tangent = std::abs(normal.z) < 0.9 ? cross(normal, Vec3{0, 0, 1}) : cross(normal, Vec3{0, 1, 0});
                tangent = normalize(tangent);
                const Vec3 bitangent = normalize(cross(normal, tangent));
                const double radius = axisLength * 0.65;
                for (int i = 0; i < kSegments; ++i) {
                    const double a0 = kPi * 2.0 * i / kSegments;
                    const double a1 = kPi * 2.0 * (i + 1) / kSegments;
                    pushLine(gizmo, origin + (tangent * std::cos(a0) + bitangent * std::sin(a0)) * radius, colors[axis]);
                    pushLine(gizmo, origin + (tangent * std::cos(a1) + bitangent * std::sin(a1)) * radius, colors[axis]);
                }
            }
        }
        renderer_->submitLines(gizmo);
    }
    renderer_->endFrame();
}

QImage ViewportWidget::captureView() {
    if (renderer_ == nullptr) {
        return grabFramebuffer();
    }
    makeCurrent();
    const unsigned framebuffer = defaultFramebufferObject();
    if (framebuffer != 0) {
        // QOpenGLFunctions is not on this widget. Bind through the context's functions.
    }
    std::vector<std::uint8_t> rgba;
    int imageWidth = 0;
    int imageHeight = 0;
    // paintGL already ran for grab; read the widget FBO the renderer last drew into.
    QOpenGLContext* context = QOpenGLContext::currentContext();
    if (context != nullptr && framebuffer != 0) {
        auto* functions = context->functions();
        if (functions != nullptr) {
            functions->glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        }
    }
    const bool ok = renderer_->captureRgba(rgba, imageWidth, imageHeight);
    doneCurrent();
    if (!ok || imageWidth <= 0 || imageHeight <= 0) {
        return grabFramebuffer();
    }
    QImage image(imageWidth, imageHeight, QImage::Format_RGBA8888);
    const int stride = imageWidth * 4;
    for (int y = 0; y < imageHeight; ++y) {
        std::memcpy(image.scanLine(imageHeight - 1 - y), rgba.data() + static_cast<std::size_t>(y) * stride,
                    static_cast<std::size_t>(stride));
    }
    return image;
}

void ViewportWidget::mousePressEvent(QMouseEvent* event) {
    lastPos_ = event->position().toPoint();
    if (event->button() == Qt::MiddleButton) {
        panning_ = event->modifiers().testFlag(Qt::ShiftModifier);
        draggingCamera_ = !panning_;
        return;
    }
    if (event->button() != Qt::LeftButton || scene_ == nullptr) {
        return;
    }
    const int axis = pickGizmo(event->position().x(), event->position().y());
    if (axis >= 0 && !scene_->selectedId().empty()) {
        Vec3 origin, axisX, axisY, axisZ;
        double axisLength = 0;
        gizmoAxes(origin, axisX, axisY, axisZ, axisLength);
        const Vec3 axes[3] = {axisX, axisY, axisZ};
        gizmoAxis_ = axis;
        draggingGizmo_ = true;
        gizmoDragOrigin_ = origin;
        gizmoDragDirection_ = axes[axis];
        auto node = scene_->find(scene_->selectedId());
        gizmoRotateBasis_ = node ? node->local.rotation : Quat::identity();
        gizmoRotateTranslation_ = node ? node->local.translation : Vec3{};
        const Ray ray = camera_.rayFromPixel(event->position().x(), event->position().y(), width(), height());
        if (gizmoMode_ == GizmoMode::Translate) {
            gizmoDragParameter_ = closestParameter(origin, axes[axis], ray);
        } else {
            const double denom = dot(axes[axis], ray.direction);
            if (std::abs(denom) > 1e-8) {
                const double t = dot(axes[axis], origin - ray.origin) / denom;
                gizmoRotateStart_ = ray.origin + ray.direction * t - origin;
            }
        }
        return;
    }
    const Ray ray = camera_.rayFromPixel(event->position().x(), event->position().y(), width(), height());
    const auto hit = renderer_ ? renderer_->pick(*scene_, ray) : pickScene(*scene_, ray);
    const QString id = hit ? QString::fromStdString(hit->nodeId) : QString();
    scene_->setSelected(hit ? hit->nodeId : std::string{});
    emit selectionChanged(id);
    if (hit) {
        emit pointPicked(hit->point.x, hit->point.y, hit->point.z);
    }
    update();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPoint pos = event->position().toPoint();
    const double dx = pos.x() - lastPos_.x();
    const double dy = pos.y() - lastPos_.y();
    lastPos_ = pos;
    if (draggingCamera_) {
        camera_.orbit(dx, dy);
        update();
        return;
    }
    if (panning_) {
        camera_.pan(dx, dy);
        update();
        return;
    }
    if (draggingGizmo_) {
        applyGizmoDrag(event->position().x(), event->position().y());
        update();
    }
}

void ViewportWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton) {
        draggingCamera_ = false;
        panning_ = false;
    }
    if (event->button() == Qt::LeftButton) {
        draggingGizmo_ = false;
        gizmoAxis_ = -1;
    }
}

void ViewportWidget::wheelEvent(QWheelEvent* event) {
    const double notches = event->angleDelta().y() / 120.0;
    camera_.zoomWheel(notches);
    update();
}

}  // namespace sim
