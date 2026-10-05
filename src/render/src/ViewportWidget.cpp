#include "sim/render/ViewportWidget.hpp"

#include "sim/render/Renderer.hpp"

#include <QDateTime>
#include <QElapsedTimer>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace sim {
namespace {

bool selectedBranch(const SceneNode& node, const Scene& scene) {
    const auto& ids = scene.selection();
    if (ids.empty()) {
        return false;
    }
    const auto listed = [&ids](const std::string& id) {
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
    if (listed(node.id)) {
        return true;
    }
    auto parent = node.parent.lock();
    while (parent) {
        if (listed(parent->id)) {
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

void ViewportWidget::setShadeStrength(float ao, float shadow) {
    aoStrength_ = std::clamp(ao, 0.0f, 1.0f);
    shadowStrength_ = std::clamp(shadow, 0.0f, 1.0f);
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
    resident_.clear();
    lodCache_.clear();
    syncedRevision_ = scene_->structureRevision();
}

int ViewportWidget::gpuFrustumCull(const std::vector<Aabb>& bounds) {
    if (renderer_ == nullptr || !isValid()) {
        return -1;
    }
    makeCurrent();
    const double aspect = height() > 0 ? static_cast<double>(width()) / static_cast<double>(height()) : 1.0;
    const Mat4 viewProjection = mul(camera_.projectionMatrix(aspect), camera_.viewMatrix());
    const int kept = renderer_->gpuFrustumCull(viewProjection, bounds);
    doneCurrent();
    return kept;
}

int ViewportWidget::lastSubmitted() const { return renderer_ ? renderer_->lastSubmitted() : 0; }

int ViewportWidget::lastDraws() const { return renderer_ ? renderer_->lastDraws() : 0; }

void ViewportWidget::drawGrid() {
    Aabb bounds;
    if (scene_ != nullptr) {
        bounds = scene_->worldBounds();
    }
    double step = 100.0;
    double centerX = 0;
    double centerY = 0;
    double halfX = step * 16.0;
    double halfY = step * 16.0;
    double z = 0;
    if (bounds.valid) {
        const double spanX = std::max(bounds.max.x - bounds.min.x, 10.0);
        const double spanY = std::max(bounds.max.y - bounds.min.y, 10.0);
        const double extent = std::max({spanX, spanY, 10.0});
        const double raw = std::pow(10.0, std::floor(std::log10(std::max(extent / 8.0, 1e-6))));
        step = std::clamp(raw, 1.0, 1000.0);
        centerX = std::round(((bounds.min.x + bounds.max.x) * 0.5) / step) * step;
        centerY = std::round(((bounds.min.y + bounds.max.y) * 0.5) / step) * step;
        const double margin = step * 2.0;
        halfX = std::max(std::abs(bounds.max.x - centerX), std::abs(bounds.min.x - centerX)) + margin;
        halfY = std::max(std::abs(bounds.max.y - centerY), std::abs(bounds.min.y - centerY)) + margin;
        halfX = std::ceil(halfX / step) * step;
        halfY = std::ceil(halfY / step) * step;
        // A cell whose lowest solid sits clearly above z=0 keeps file coordinates.
        // The grid moves to that floor so the machine is not drawn over an origin patch.
        if (bounds.min.z > step) {
            z = bounds.min.z - std::min(step * 0.02, 2.0);
        }
    }
    LineBatch lines;
    const Vec3 minor{0.28f, 0.32f, 0.38f};
    const Vec3 major{0.48f, 0.54f, 0.62f};
    const auto majorAt = [](double coordinate, double spacing) {
        long index = std::lround(coordinate / spacing) % 5;
        if (index < 0) {
            index += 5;
        }
        return index == 0;
    };
    const int countX = static_cast<int>(std::round(halfX / step));
    const int countY = static_cast<int>(std::round(halfY / step));
    for (int i = -countX; i <= countX; ++i) {
        const double x = centerX + static_cast<double>(i) * step;
        const Vec3 color = majorAt(x, step) ? major : minor;
        pushLine(lines, {x, centerY - halfY, z}, color);
        pushLine(lines, {x, centerY + halfY, z}, color);
    }
    for (int j = -countY; j <= countY; ++j) {
        const double y = centerY + static_cast<double>(j) * step;
        const Vec3 color = majorAt(y, step) ? major : minor;
        pushLine(lines, {centerX - halfX, y, z}, color);
        pushLine(lines, {centerX + halfX, y, z}, color);
    }
    const double axis = std::clamp(std::min(halfX, halfY) * 0.28, step * 3.0, step * 12.0);
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
    frame.projection = camera_.projectionMatrix(aspect);
    frame.view = camera_.viewMatrix();
    frame.viewProjection = mul(frame.projection, frame.view);
    frame.eye = camera_.eye();
    frame.aoStrength = aoStrength_;
    frame.shadowStrength = shadowStrength_;
    frame.width = std::max(width(), 1);
    frame.height = std::max(height(), 1);
    frame.frustumCull = true;
    frame.clearColor = darkChrome_ ? Vec3{0.07, 0.09, 0.14} : Vec3{0.94, 0.94, 0.97};
    QElapsedTimer drawTimer;
    drawTimer.start();
    culledFrame_ = 0;
    occludedFrame_ = 0;
    drawnTriangles_ = 0;
    renderer_->beginFrame(frame);
    drawGrid();

    if (scene_ != nullptr && scene_->root()) {
        std::vector<RenderInstance> instances;
        int uploads = 0;
        auto ensureMesh = [&](const std::string& key, const std::shared_ptr<MeshData>& mesh) {
            if (!mesh || mesh->empty() || resident_.count(key) != 0) {
                return resident_.count(key) != 0;
            }
            if (!resident_.empty() && uploads >= 32) {
                return false;
            }
            renderer_->uploadMesh(key, *mesh);
            resident_.insert(key);
            ++uploads;
            return true;
        };
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
            const Mat4 world = node->worldMatrix();
            const Aabb bounds = mesh ? transformedBounds(mesh->bounds, world) : Aabb{};
            if (frame.frustumCull && !intersectsClipFrustum(frame.viewProjection, bounds)) {
                ++culledFrame_;
                continue;
            }
            if (mesh) {
                triangles_ += static_cast<int>(mesh->triangleCount());
            }
            std::string key = node->meshId;
            const int levels = mesh && mesh->triangleCount() > 8 ? 3 : 1;
            const float distance = static_cast<float>(length(bounds.center() - frame.eye));
            const int lod = forceDetail_ ? 0 : selectLodLevel(distance, levels);
            const MeshData* drawn = mesh.get();
            if (lod > 0 && mesh) {
                const std::string lodKey = node->meshId + "#lod" + std::to_string(lod);
                if (lodCache_.count(lodKey) == 0) {
                    const double cell = std::max(mesh->bounds.diagonal() * (lod == 1 ? 0.2 : 0.55), 1.0);
                    lodCache_[lodKey] = simplifyMesh(*mesh, cell);
                }
                if (ensureMesh(lodKey, lodCache_[lodKey])) {
                    key = lodKey;
                    drawn = lodCache_[lodKey].get();
                }
            }
            if (key == node->meshId) {
                ensureMesh(node->meshId, mesh);
            }
            if (resident_.count(key) == 0) {
                continue;
            }
            RenderInstance instance;
            instance.meshKey = key;
            instance.world = world;
            instance.material.color = node->color;
            instance.material.highlight = selectedBranch(*node, *scene_) ? 1.0f : 0.0f;
            instance.material.metallic = node->metallic;
            instance.material.roughness = node->roughness;
            instance.worldBounds = bounds;
            instance.distanceMm = distance;
            instance.lodLevels = 1;
            if (drawn != nullptr) {
                instance.triangles = static_cast<int>(drawn->triangleCount());
            }
            instances.push_back(std::move(instance));
        }
        if (ghost_) {
            std::shared_ptr<SceneNode> ghostNode;
            if (auto selected = scene_->find(scene_->selectedId()); selected && !selected->meshId.empty()) {
                ghostNode = selected;
            }
            if (!ghostNode) {
                std::vector<std::shared_ptr<SceneNode>> walk{scene_->root()};
                while (!walk.empty() && !ghostNode) {
                    auto node = walk.back();
                    walk.pop_back();
                    if (!node) {
                        continue;
                    }
                    for (const auto& child : node->children) {
                        walk.push_back(child);
                    }
                    if (node->effectivelyVisible() && !node->meshId.empty()) {
                        ghostNode = node;
                    }
                }
            }
            if (ghostNode) {
                const Mat4 home = ghostNode->worldMatrix();
                Mat4 world = home;
                bool onPath = false;
                for (const RobotPath& path : scene_->robotPaths) {
                    for (auto point = path.points.rbegin(); point != path.points.rend(); ++point) {
                        if (point->cartesian) {
                            world(3, 0) = point->position.x;
                            world(3, 1) = point->position.y;
                            world(3, 2) = point->position.z;
                            onPath = true;
                            break;
                        }
                    }
                    if (onPath) {
                        break;
                    }
                }
                if (!onPath) {
                    Vec3 right;
                    Vec3 up;
                    Vec3 forward;
                    camera_.basis(right, up, forward);
                    const double shift = std::max(120.0, camera_.distance * 0.08);
                    world(3, 0) += right.x * shift;
                    world(3, 1) += right.y * shift;
                    world(3, 2) += right.z * shift + shift * 0.15;
                }
                const auto mesh = scene_->mesh(ghostNode->meshId);
                RenderInstance ghost;
                ghost.meshKey = ghostNode->meshId;
                ghost.world = world;
                ghost.material.color = {0.239, 0.525, 0.910};
                ghost.material.highlight = 0;
                ghost.material.opacity = 0.38f;
                ghost.worldBounds = mesh ? transformedBounds(mesh->bounds, ghost.world) : Aabb{};
                ghost.distanceMm = static_cast<float>(length(ghost.worldBounds.center() - frame.eye));
                instances.push_back(std::move(ghost));
                ghostCaption_ = onPath ? QStringLiteral("幽灵在路径末点 · 场景未改 · 不是模型输出")
                                       : QStringLiteral("幽灵是零件的平移副本 · 场景未改 · 不是模型输出");
                LineBatch link;
                link.width = 1.5f;
                link.depthTest = false;
                pushLine(link, Vec3{home(3, 0), home(3, 1), home(3, 2)}, Vec3{0.239, 0.525, 0.910});
                pushLine(link, Vec3{world(3, 0), world(3, 1), world(3, 2)}, Vec3{0.114, 0.333, 0.702});
                renderer_->submitLines(link);
            } else {
                ghostCaption_.clear();
            }
        } else {
            ghostCaption_.clear();
        }
        if (!instances.empty()) {
            OcclusionBuffer occlusion;
            occlusion.reset(160, 90);
            std::vector<int> order(instances.size());
            for (int i = 0; i < static_cast<int>(instances.size()); ++i) {
                order[static_cast<std::size_t>(i)] = i;
            }
            std::sort(order.begin(), order.end(), [&](int left, int right) {
                return instances[static_cast<std::size_t>(left)].distanceMm < instances[static_cast<std::size_t>(right)].distanceMm;
            });
            std::vector<RenderInstance> visible;
            visible.reserve(instances.size());
            for (int index : order) {
                RenderInstance& instance = instances[static_cast<std::size_t>(index)];
                ScreenRect rect;
                if (projectBounds(frame.viewProjection, instance.worldBounds, occlusion.width, occlusion.height, rect) &&
                    hiddenByOcclusion(occlusion, rect)) {
                    ++occludedFrame_;
                    continue;
                }
                if (rect.valid) {
                    rasterizeOccluder(occlusion, rect);
                }
                visible.push_back(std::move(instance));
            }
            instances.swap(visible);
            drawnTriangles_ = 0;
            for (const RenderInstance& instance : instances) {
                drawnTriangles_ += instance.triangles;
            }
        }
        static const std::shared_ptr<MeshData> zoneSphere = makeSphereMesh(1.0, 12);
        static const std::shared_ptr<MeshData> zoneBox = makeBoxMesh({1, 1, 1});
        static const std::shared_ptr<MeshData> zoneCylinder = makeCylinderMesh(1.0, 1.0, 16);
        for (const PlcZoneWatch& zone : scene_->softPlc.zones) {
            const std::string shape = zone.shape == "box" || zone.shape == "cylinder" ? zone.shape : "sphere";
            double sx = 0;
            double sy = 0;
            double sz = 0;
            std::string key;
            const std::shared_ptr<MeshData>* mesh = nullptr;
            if (shape == "box") {
                sx = std::max(zone.size.x, 0.0);
                sy = std::max(zone.size.y, 0.0);
                sz = std::max(zone.size.z, 0.0);
                key = "plc-zone-box";
                mesh = &zoneBox;
            } else if (shape == "cylinder") {
                sx = std::max(zone.size.x, 0.0);
                sy = sx;
                sz = std::max(zone.size.z, 0.0);
                key = "plc-zone-cylinder";
                mesh = &zoneCylinder;
            } else {
                sx = std::max(zone.radius, 0.0);
                sy = sx;
                sz = sx;
                key = "plc-zone-sphere";
                mesh = &zoneSphere;
            }
            if (sx < 1e-4 || sy < 1e-4 || sz < 1e-4 || mesh == nullptr || !ensureMesh(key, *mesh)) {
                continue;
            }
            Mat4 world = Mat4::identity();
            world(0, 0) = sx;
            world(1, 1) = sy;
            world(2, 2) = sz;
            world(3, 0) = zone.center.x;
            world(3, 1) = zone.center.y;
            world(3, 2) = zone.center.z;
            RenderInstance volume;
            volume.meshKey = key;
            volume.world = world;
            volume.material.color = {0.93, 0.32, 0.26};
            volume.material.opacity = 0.38f;
            volume.material.roughness = 0.6f;
            volume.worldBounds = transformedBounds((*mesh)->bounds, world);
            volume.distanceMm = static_cast<float>(length(volume.worldBounds.center() - frame.eye));
            volume.triangles = static_cast<int>((*mesh)->triangleCount());
            instances.push_back(std::move(volume));
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
    if (scene_ != nullptr && !scene_->welds.empty()) {
        LineBatch welds;
        welds.width = 2.2f;
        welds.depthTest = false;
        for (const WeldSpot& spot : scene_->welds) {
            const Vec3 color = !spot.checked ? Vec3{0.35, 0.55, 0.95}
                                             : (spot.reachable ? Vec3{0.22, 0.78, 0.38} : Vec3{0.86, 0.28, 0.22});
            const Vec3 normal = length(spot.normal) > 0.5 ? normalize(spot.normal) : Vec3{0, 0, 1};
            Vec3 tangent = std::abs(normal.z) < 0.9 ? cross(normal, Vec3{0, 0, 1}) : cross(normal, Vec3{1, 0, 0});
            tangent = normalize(tangent);
            const Vec3 bitangent = normalize(cross(normal, tangent));
            const double mark = 28;
            pushLine(welds, spot.position - tangent * mark, color);
            pushLine(welds, spot.position + tangent * mark, color);
            pushLine(welds, spot.position - bitangent * mark, color);
            pushLine(welds, spot.position + bitangent * mark, color);
            pushLine(welds, spot.position, color);
            pushLine(welds, spot.position + normal * 70.0, {0.95, 0.78, 0.25});
        }
        renderer_->submitLines(welds);
    }
    if (scene_ != nullptr) {
        LineBatch tour;
        tour.width = 2.4f;
        tour.depthTest = false;
        const Vec3 ink{0.24, 0.62, 0.95};
        for (const RobotPath& path : scene_->robotPaths) {
            if (path.name != "weld_tour" || path.points.size() < 2) {
                continue;
            }
            for (std::size_t i = 1; i < path.points.size(); ++i) {
                pushLine(tour, path.points[i - 1].position, ink);
                pushLine(tour, path.points[i].position, ink);
            }
        }
        if (!tour.vertices.empty()) {
            renderer_->submitLines(tour);
        }
    }
    renderer_->endFrame();
    drawMs_ = static_cast<double>(drawTimer.nsecsElapsed()) / 1.0e6;
    paintOverlay();
}

void ViewportWidget::setSimTime(double seconds) { simTime_ = seconds; }

void ViewportWidget::setPhysicsLine(const QString& text) {
    physicsLine_ = text;
    update();
}

void ViewportWidget::setCallout(const QString& text) {
    callout_ = text;
    update();
}

void ViewportWidget::setMeasureMode(bool enabled) {
    measureMode_ = enabled;
    if (!enabled) {
        measurePoints_.clear();
    }
    update();
}

void ViewportWidget::setPreviewMode(bool enabled) {
    previewMode_ = enabled;
    update();
}

void ViewportWidget::setGhost(bool enabled) {
    ghost_ = enabled;
    if (!enabled) {
        ghostCaption_.clear();
    }
    update();
}

void ViewportWidget::setDarkChrome(bool dark) {
    darkChrome_ = dark;
    update();
}

void ViewportWidget::paintOverlay() {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (lastFrameMs_ != 0) {
        const double dt = static_cast<double>(now - lastFrameMs_) / 1000.0;
        if (dt > 0.001) {
            const double instant = 1.0 / dt;
            fps_ = fps_ <= 0 ? instant : fps_ * 0.8 + instant * 0.2;
        }
    }
    lastFrameMs_ = now;

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const QColor ink = darkChrome_ ? QColor(231, 233, 247) : QColor(23, 29, 56);
    const QColor muted = darkChrome_ ? QColor(166, 172, 203) : QColor(76, 83, 115);
    const QColor glass = darkChrome_ ? QColor(19, 25, 48, 190) : QColor(255, 255, 255, 220);
    const QColor line = darkChrome_ ? QColor(61, 134, 232, 110) : QColor(44, 110, 206, 140);
    auto capsule = [&](const QRect& rect) {
        painter.setPen(line);
        painter.setBrush(glass);
        painter.drawRoundedRect(rect, 10, 10);
    };

    QString hud = QString("透视  ·  t %1 s  ·  %2 fps  ·  %3 三角")
                      .arg(simTime_, 0, 'f', 2)
                      .arg(fps_, 0, 'f', 0)
                      .arg(triangles_);
    if (previewMode_) {
        hud += "  ·  预览模式 · 场景未修改";
    }
    if (!ghostCaption_.isEmpty()) {
        hud += "  ·  " + ghostCaption_;
    }
    if (!physicsLine_.isEmpty()) {
        hud += "\n" + physicsLine_;
    }
    if (fps_ > 0 && fps_ < 30) {
        hud += "  ·  建议检查显示负载";
    }
    const bool wideHud = !ghostCaption_.isEmpty() || !physicsLine_.isEmpty();
    const QRect hudRect(16, 16, std::min(width() - 140, wideHud ? std::max(360, width() - 150) : 520), wideHud ? 52 : 28);
    capsule(hudRect);
    painter.setPen(muted);
    painter.drawText(hudRect.adjusted(10, 4, -10, -4), Qt::AlignVCenter | Qt::AlignLeft | Qt::TextWordWrap, hud);

    cubeRect_ = QRect(width() - 112, 16, 96, 104);
    capsule(cubeRect_);
    faceTop_ = QRect(cubeRect_.left() + 28, cubeRect_.top() + 8, 40, 26);
    faceFront_ = QRect(cubeRect_.left() + 8, cubeRect_.top() + 40, 40, 28);
    faceRight_ = QRect(cubeRect_.left() + 48, cubeRect_.top() + 40, 40, 28);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(91, 140, 255, 180));
    painter.drawPolygon(QPolygon() << faceTop_.topLeft() << faceTop_.topRight() << faceRight_.bottomRight()
                                   << faceFront_.bottomLeft());
    painter.setBrush(QColor(42, 54, 94, 220));
    painter.drawRect(faceFront_);
    painter.setBrush(QColor(54, 69, 122, 220));
    painter.drawRect(faceRight_);
    painter.setBrush(QColor(44, 110, 206, 230));
    painter.drawRect(faceTop_);
    painter.setPen(ink);
    painter.drawText(faceTop_, Qt::AlignCenter, "上");
    painter.drawText(faceFront_, Qt::AlignCenter, "前");
    painter.drawText(faceRight_, Qt::AlignCenter, "右");

    if (!callout_.isEmpty()) {
        const int textWidth = QFontMetrics(painter.font()).horizontalAdvance(callout_) + 24;
        const int cap = std::max(180, width() - 48);
        const QRect box(24, 56, std::clamp(textWidth, 180, cap), 32);
        painter.setPen(QColor(255, 178, 36));
        painter.setBrush(QColor(42, 32, 8, 210));
        painter.drawRoundedRect(box, 8, 8);
        painter.drawText(box.adjusted(8, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft, callout_);
    }
    if (measurePoints_.size() == 2) {
        const double distance = length(measurePoints_[1] - measurePoints_[0]);
        const QRect box(24, height() - 48, 220, 28);
        painter.setPen(QColor(98, 210, 242));
        painter.setBrush(glass);
        painter.drawRoundedRect(box, 8, 8);
        painter.drawText(box, Qt::AlignCenter, QString("测量  %1 mm").arg(distance, 0, 'f', 1));
    } else if (measureMode_) {
        painter.setPen(QColor(98, 210, 242));
        painter.drawText(24, height() - 28, "测量：在模型上点两个点");
    }

    toolbarRect_ = QRect();
    if (scene_ != nullptr && !scene_->selectedId().empty()) {
        toolbarRect_ = QRect(std::max(16, width() / 2 - 140), std::max(56, height() - 78), 280, 34);
        capsule(toolbarRect_);
        painter.setPen(ink);
        const QString labels[] = {"聚焦", "移动", "测量"};
        for (int i = 0; i < 3; ++i) {
            const QRect slot(toolbarRect_.left() + i * 92, toolbarRect_.top(), 92, toolbarRect_.height());
            painter.drawText(slot, Qt::AlignCenter, labels[i]);
        }
    }
    triangles_ = 0;
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
    const QPoint click = event->position().toPoint();
    if (cubeRect_.contains(click)) {
        cubePress_ = true;
        cubeDrag_ = false;
        cubePressPos_ = click;
        return;
    }
    if (toolbarRect_.contains(click)) {
        const int slot = std::clamp((click.x() - toolbarRect_.left()) / 92, 0, 2);
        const char* ids[] = {"ui.fit", "ui.move", "ui.measure"};
        emit overlayCommand(QString::fromLatin1(ids[slot]));
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
    const bool extend = event->modifiers().testFlag(Qt::ControlModifier);
    if (extend && hit) {
        scene_->toggleSelected(hit->nodeId);
    } else if (!extend) {
        scene_->setSelected(hit ? hit->nodeId : std::string{});
    }
    emit selectionChanged(id);
    if (hit) {
        emit pointPicked(hit->point.x, hit->point.y, hit->point.z, hit->normal.x, hit->normal.y, hit->normal.z, id);
        if (measureMode_) {
            measurePoints_.push_back(hit->point);
            if (measurePoints_.size() > 2) {
                measurePoints_.erase(measurePoints_.begin());
            }
        }
    }
    update();
}

void ViewportWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPoint pos = event->position().toPoint();
    const double dx = pos.x() - lastPos_.x();
    const double dy = pos.y() - lastPos_.y();
    lastPos_ = pos;
    if (cubePress_) {
        if ((pos - cubePressPos_).manhattanLength() > 4) {
            cubeDrag_ = true;
        }
        if (cubeDrag_) {
            camera_.orbit(dx, dy);
            update();
        }
        return;
    }
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
        if (cubePress_ && !cubeDrag_) {
            const QPoint click = event->position().toPoint();
            if (faceTop_.contains(click) || faceTop_.contains(cubePressPos_)) {
                setStandardView(StandardView::Top);
            } else if (faceFront_.contains(click) || faceFront_.contains(cubePressPos_)) {
                setStandardView(StandardView::Front);
            } else if (faceRight_.contains(click) || faceRight_.contains(cubePressPos_)) {
                setStandardView(StandardView::Right);
            }
        }
        cubePress_ = false;
        cubeDrag_ = false;
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
