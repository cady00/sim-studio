#include "sim/render/ViewportWidget.hpp"

#include "sim/core/Pick.hpp"

#include <QMouseEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {
namespace {

void toGl(const Mat4& matrix, float out[16]) {
    for (int i = 0; i < 16; ++i) {
        out[i] = static_cast<float>(matrix.m[i]);
    }
}


const char* kMeshVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMvp;
uniform mat4 uModel;
uniform mat3 uNormalMat;
out vec3 vNormal;
out vec3 vWorld;
void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorld = world.xyz;
    vNormal = normalize(uNormalMat * aNormal);
    gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

const char* kMeshFragment = R"(#version 330 core
in vec3 vNormal;
in vec3 vWorld;
uniform vec3 uColor;
uniform vec3 uEye;
uniform float uHighlight;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormal);
    vec3 lightA = normalize(vec3(0.35, -0.45, 0.82));
    vec3 lightB = normalize(vec3(-0.55, -0.15, 0.35));
    float diffuse = max(dot(n, lightA), 0.0) * 0.72 + max(dot(n, lightB), 0.0) * 0.22;
    vec3 viewDir = normalize(uEye - vWorld);
    float spec = pow(max(dot(n, normalize(lightA + viewDir)), 0.0), 32.0) * 0.22;
    vec3 color = uColor * (0.28 + diffuse) + vec3(spec);
    if (uHighlight > 0.5) {
        color = mix(color, vec3(0.58, 0.60, 0.98), 0.42);
    }
    fragColor = vec4(color, 1.0);
}
)";

const char* kLineVertex = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)";

const char* kLineFragment = R"(#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

void appendVertex(std::vector<float>& data, Vec3 position, Vec3 color) {
    data.push_back(static_cast<float>(position.x));
    data.push_back(static_cast<float>(position.y));
    data.push_back(static_cast<float>(position.z));
    data.push_back(static_cast<float>(color.x));
    data.push_back(static_cast<float>(color.y));
    data.push_back(static_cast<float>(color.z));
}

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

}  // namespace

ViewportWidget::ViewportWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    camera_.setStandardView(StandardView::Iso);
}

ViewportWidget::~ViewportWidget() {
    makeCurrent();
    destroyGpu();
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

bool ViewportWidget::compileProgram(GLuint& program, const char* vertexSource, const char* fragmentSource, QString& error) {
    const auto compile = [&](GLenum type, const char* source) -> GLuint {
        const GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        GLint ok = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[1024];
            glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
            error = QString::fromUtf8(log);
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    };
    const GLuint vertex = compile(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragment = compile(GL_FRAGMENT_SHADER, fragmentSource);
    if (vertex == 0 || fragment == 0) {
        return false;
    }
    program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    GLint ok = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        error = QString::fromUtf8(log);
        glDeleteProgram(program);
        program = 0;
        return false;
    }
    return true;
}

void ViewportWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(0.10f, 0.11f, 0.14f, 1.0f);
    QString error;
    if (!compileProgram(meshProgram_, kMeshVertex, kMeshFragment, error) ||
        !compileProgram(lineProgram_, kLineVertex, kLineFragment, error)) {
        glError_ = error;
        emit glError(error);
        return;
    }
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);
}

void ViewportWidget::resizeGL(int width, int height) { glViewport(0, 0, width, height); }

void ViewportWidget::clearGpuMeshes() {
    for (auto& [id, mesh] : gpu_) {
        (void)id;
        glDeleteBuffers(1, &mesh.vbo);
        glDeleteBuffers(1, &mesh.ebo);
        glDeleteVertexArrays(1, &mesh.vao);
    }
    gpu_.clear();
}

void ViewportWidget::destroyGpu() {
    clearGpuMeshes();
    if (lineVbo_ != 0) {
        glDeleteBuffers(1, &lineVbo_);
        lineVbo_ = 0;
    }
    if (lineVao_ != 0) {
        glDeleteVertexArrays(1, &lineVao_);
        lineVao_ = 0;
    }
    if (meshProgram_ != 0) {
        glDeleteProgram(meshProgram_);
        meshProgram_ = 0;
    }
    if (lineProgram_ != 0) {
        glDeleteProgram(lineProgram_);
        lineProgram_ = 0;
    }
}

void ViewportWidget::syncMeshes() {
    if (scene_ == nullptr || scene_->structureRevision() == syncedRevision_ || meshProgram_ == 0) {
        return;
    }
    clearGpuMeshes();
    for (const auto& [id, mesh] : scene_->meshes()) {
        if (!mesh || mesh->empty()) {
            continue;
        }
        std::vector<float> interleaved(mesh->vertexCount() * 6);
        for (std::size_t i = 0; i < mesh->vertexCount(); ++i) {
            interleaved[i * 6 + 0] = mesh->positions[i * 3 + 0];
            interleaved[i * 6 + 1] = mesh->positions[i * 3 + 1];
            interleaved[i * 6 + 2] = mesh->positions[i * 3 + 2];
            interleaved[i * 6 + 3] = mesh->normals[i * 3 + 0];
            interleaved[i * 6 + 4] = mesh->normals[i * 3 + 1];
            interleaved[i * 6 + 5] = mesh->normals[i * 3 + 2];
        }
        GpuMesh gpu;
        glGenVertexArrays(1, &gpu.vao);
        glGenBuffers(1, &gpu.vbo);
        glGenBuffers(1, &gpu.ebo);
        glBindVertexArray(gpu.vao);
        glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)), interleaved.data(),
                     GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh->indices.size() * sizeof(std::uint32_t)),
                     mesh->indices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
        gpu.indexCount = static_cast<GLsizei>(mesh->indices.size());
        glBindVertexArray(0);
        gpu_.emplace(id, gpu);
    }
    syncedRevision_ = scene_->structureRevision();
}

void ViewportWidget::drawLines(const std::vector<float>& vertices, float width) {
    if (vertices.empty() || lineProgram_ == 0) {
        return;
    }
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)), vertices.data(),
                 GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glLineWidth(width);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size() / 6));
    glBindVertexArray(0);
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
    std::vector<float> lines;
    const Vec3 minor{0.22f, 0.24f, 0.28f};
    const Vec3 major{0.34f, 0.37f, 0.44f};
    const int count = static_cast<int>(std::round(half / step));
    for (int i = -count; i <= count; ++i) {
        const double coordinate = i * step;
        const Vec3 color = (i % 5 == 0) ? major : minor;
        appendVertex(lines, {coordinate, -half, 0}, color);
        appendVertex(lines, {coordinate, half, 0}, color);
        appendVertex(lines, {-half, coordinate, 0}, color);
        appendVertex(lines, {half, coordinate, 0}, color);
    }
    const double axis = half * 0.35;
    appendVertex(lines, {0, 0, 0}, {0.85, 0.28, 0.30});
    appendVertex(lines, {axis, 0, 0}, {0.85, 0.28, 0.30});
    appendVertex(lines, {0, 0, 0}, {0.30, 0.75, 0.42});
    appendVertex(lines, {0, axis, 0}, {0.30, 0.75, 0.42});
    appendVertex(lines, {0, 0, 0}, {0.35, 0.55, 0.90});
    appendVertex(lines, {0, 0, axis}, {0.35, 0.55, 0.90});
    drawLines(lines, 1.0f);
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
    if (!glError_.isEmpty() || meshProgram_ == 0) {
        glClearColor(0.10f, 0.11f, 0.14f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    syncMeshes();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const double aspect = height() > 0 ? static_cast<double>(width()) / height() : 1.0;
    const Mat4 viewProj = mul(camera_.projectionMatrix(aspect), camera_.viewMatrix());

    float viewProjGl[16];
    toGl(viewProj, viewProjGl);
    glUseProgram(lineProgram_);
    glUniformMatrix4fv(glGetUniformLocation(lineProgram_, "uViewProj"), 1, GL_FALSE, viewProjGl);
    drawGrid();

    glUseProgram(meshProgram_);
    const Vec3 eye = camera_.eye();
    glUniform3f(glGetUniformLocation(meshProgram_, "uEye"), static_cast<float>(eye.x), static_cast<float>(eye.y),
                static_cast<float>(eye.z));
    if (scene_ != nullptr && scene_->root()) {
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
            const auto gpu = gpu_.find(node->meshId);
            if (gpu == gpu_.end()) {
                continue;
            }
            const Mat4 model = node->worldMatrix();
            const Mat4 mvp = mul(viewProj, model);
            float normalMat[9] = {static_cast<float>(model(0, 0)), static_cast<float>(model(0, 1)),
                                  static_cast<float>(model(0, 2)), static_cast<float>(model(1, 0)),
                                  static_cast<float>(model(1, 1)), static_cast<float>(model(1, 2)),
                                  static_cast<float>(model(2, 0)), static_cast<float>(model(2, 1)),
                                  static_cast<float>(model(2, 2))};
            float modelGl[16];
            float mvpGl[16];
            toGl(model, modelGl);
            toGl(mvp, mvpGl);
            glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uModel"), 1, GL_FALSE, modelGl);
            glUniformMatrix4fv(glGetUniformLocation(meshProgram_, "uMvp"), 1, GL_FALSE, mvpGl);
            glUniformMatrix3fv(glGetUniformLocation(meshProgram_, "uNormalMat"), 1, GL_FALSE, normalMat);
            glUniform3f(glGetUniformLocation(meshProgram_, "uColor"), static_cast<float>(node->color.x),
                        static_cast<float>(node->color.y), static_cast<float>(node->color.z));
            glUniform1f(glGetUniformLocation(meshProgram_, "uHighlight"),
                        selectedBranch(*node, scene_->selectedId()) ? 1.0f : 0.0f);
            glBindVertexArray(gpu->second.vao);
            glDrawElements(GL_TRIANGLES, gpu->second.indexCount, GL_UNSIGNED_INT, nullptr);
        }
        glBindVertexArray(0);
    }

    Vec3 origin, axisX, axisY, axisZ;
    double axisLength = 0;
    if (gizmoAxes(origin, axisX, axisY, axisZ, axisLength)) {
        std::vector<float> gizmo;
        const Vec3 colors[3] = {{0.93, 0.32, 0.32}, {0.30, 0.82, 0.42}, {0.36, 0.58, 0.95}};
        const Vec3 axes[3] = {axisX, axisY, axisZ};
        for (int i = 0; i < 3; ++i) {
            appendVertex(gizmo, origin, colors[i]);
            appendVertex(gizmo, origin + axes[i] * axisLength, colors[i]);
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
                    appendVertex(gizmo, origin + (tangent * std::cos(a0) + bitangent * std::sin(a0)) * radius, colors[axis]);
                    appendVertex(gizmo, origin + (tangent * std::cos(a1) + bitangent * std::sin(a1)) * radius, colors[axis]);
                }
            }
        }
        glDisable(GL_DEPTH_TEST);
        glUseProgram(lineProgram_);
        glUniformMatrix4fv(glGetUniformLocation(lineProgram_, "uViewProj"), 1, GL_FALSE, viewProjGl);
        drawLines(gizmo, 2.5f);
        glEnable(GL_DEPTH_TEST);
    }
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
        const Vec3 local[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
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
        (void)local;
        return;
    }
    const Ray ray = camera_.rayFromPixel(event->position().x(), event->position().y(), width(), height());
    const auto hit = pickScene(*scene_, ray);
    const QString id = hit ? QString::fromStdString(hit->nodeId) : QString();
    scene_->setSelected(hit ? hit->nodeId : std::string{});
    emit selectionChanged(id);
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
