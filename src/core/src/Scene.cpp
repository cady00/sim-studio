#include "sim/core/Scene.hpp"

#include <algorithm>
#include <cctype>

namespace sim {
namespace {

void expandMeshBounds(Aabb& box, const Mat4& world, const MeshData& mesh) {
    if (!mesh.bounds.valid) {
        return;
    }
    const Vec3 corners[8] = {
        {mesh.bounds.min.x, mesh.bounds.min.y, mesh.bounds.min.z},
        {mesh.bounds.max.x, mesh.bounds.min.y, mesh.bounds.min.z},
        {mesh.bounds.min.x, mesh.bounds.max.y, mesh.bounds.min.z},
        {mesh.bounds.max.x, mesh.bounds.max.y, mesh.bounds.min.z},
        {mesh.bounds.min.x, mesh.bounds.min.y, mesh.bounds.max.z},
        {mesh.bounds.max.x, mesh.bounds.min.y, mesh.bounds.max.z},
        {mesh.bounds.min.x, mesh.bounds.max.y, mesh.bounds.max.z},
        {mesh.bounds.max.x, mesh.bounds.max.y, mesh.bounds.max.z},
    };
    for (const Vec3& corner : corners) {
        box.expand(transformPoint(world, corner));
    }
}

int countNodes(const SceneNode& node) {
    int count = 1;
    for (const auto& child : node.children) {
        if (child) {
            count += countNodes(*child);
        }
    }
    return count;
}

int trailingNumber(const std::string& id, char prefix) {
    if (id.size() < 2 || id[0] != prefix) {
        return 0;
    }
    int value = 0;
    for (std::size_t i = 1; i < id.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(id[i]))) {
            return 0;
        }
        value = value * 10 + (id[i] - '0');
    }
    return value;
}

void considerId(const SceneNode& node, int& nextNode) {
    nextNode = std::max(nextNode, trailingNumber(node.id, 'n') + 1);
    for (const auto& child : node.children) {
        if (child) {
            considerId(*child, nextNode);
        }
    }
}

}  // namespace

Mat4 SceneNode::worldMatrix() const {
    if (const auto parentNode = parent.lock()) {
        return mul(parentNode->worldMatrix(), local.matrix());
    }
    return local.matrix();
}

bool SceneNode::effectivelyVisible() const {
    if (!visible) {
        return false;
    }
    auto parentNode = parent.lock();
    while (parentNode) {
        if (!parentNode->visible) {
            return false;
        }
        parentNode = parentNode->parent.lock();
    }
    return true;
}

Scene::Scene() { clear(); }

void Scene::clear() {
    root_ = std::make_shared<SceneNode>();
    root_->id = "root";
    root_->name = "World";
    root_->kind = NodeKind::Assembly;
    meshes_.clear();
    selected_.clear();
    nextNode_ = 1;
    nextMesh_ = 1;
    name = "Untitled";
    units = "mm";
    sourceFiles.clear();
    signals.clear();
    scripts.clear();
    activeScript.clear();
    resources.clear();
    logicBlocks.clear();
    robotPaths.clear();
    dirty_ = false;
    ++structureRevision_;
    if (!mute_ && onChanged) {
        onChanged(ChangeKind::Structure);
    }
}

void Scene::beginEdit() { mute_ = true; }

void Scene::endEdit() {
    mute_ = false;
    ++structureRevision_;
    dirty_ = true;
    if (onChanged) {
        onChanged(ChangeKind::Structure);
    }
}

void Scene::steal(Scene& other) {
    root_ = other.root_;
    meshes_ = std::move(other.meshes_);
    name = std::move(other.name);
    units = other.units.empty() ? std::string("mm") : other.units;
    sourceFiles = std::move(other.sourceFiles);
    signals = std::move(other.signals);
    scripts = std::move(other.scripts);
    activeScript = std::move(other.activeScript);
    resources = std::move(other.resources);
    logicBlocks = std::move(other.logicBlocks);
    robotPaths = std::move(other.robotPaths);
    selected_.clear();
    other.root_.reset();
    other.meshes_.clear();
    other.sourceFiles.clear();
    other.signals.clear();
    other.scripts.clear();
    other.activeScript.clear();
    other.resources.clear();
    other.logicBlocks.clear();
    other.robotPaths.clear();
    if (!root_) {
        clear();
        return;
    }
    rebind(root_, nullptr);
    adoptIds();
    dirty_ = true;
    ++structureRevision_;
    if (onChanged) {
        onChanged(ChangeKind::Structure);
    }
}

void Scene::setSelected(const std::string& id) {
    if (selected_ == id) {
        return;
    }
    if (!id.empty() && !find(id)) {
        return;
    }
    selected_ = id;
    notify(ChangeKind::Selection);
}

std::shared_ptr<SceneNode> Scene::findByName(const std::string& nodeName) const {
    if (!root_ || nodeName.empty()) {
        return {};
    }
    std::vector<std::shared_ptr<SceneNode>> stack{root_};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        if (node->name == nodeName) {
            return node;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
    }
    return {};
}

std::shared_ptr<SceneNode> Scene::spawnClone(const std::shared_ptr<SceneNode>& prototype, const Transform& worldPose) {
    if (!prototype || !root_) {
        return {};
    }
    auto copy = std::make_shared<SceneNode>();
    copy->id = "n" + std::to_string(nextNode_++);
    copy->name = prototype->name + "_" + std::to_string(nextNode_);
    copy->kind = NodeKind::Part;
    copy->local = worldPose;
    copy->meshId = prototype->meshId;
    copy->color = prototype->color;
    copy->visible = true;
    copy->transient = true;
    copy->mech = prototype->mech;
    copy->mech.kind = MechKind::Rigid;
    copy->mech.kinematic = false;
    copy->mech.drive = DriveMode::None;
    if (!mechCreatesBody(prototype->mech.kind) && prototype->mech.kind != MechKind::Rigid) {
        copy->mech.kind = MechKind::Rigid;
        copy->mech.shape = CollisionShape::Box;
    }
    addChild(root_, copy);
    return copy;
}

bool Scene::removeNode(const std::string& id) {
    auto node = find(id);
    if (!node || node == root_) {
        return false;
    }
    auto parent = node->parent.lock();
    if (!parent) {
        return false;
    }
    auto& children = parent->children;
    const auto it = std::find(children.begin(), children.end(), node);
    if (it == children.end()) {
        return false;
    }
    children.erase(it);
    if (selected_ == id) {
        selected_.clear();
    }
    notify(ChangeKind::Structure);
    return true;
}

Signal* Scene::findSignal(const std::string& signalName) {
    for (Signal& signal : signals) {
        if (signal.name == signalName) {
            return &signal;
        }
    }
    return nullptr;
}

const Signal* Scene::findSignal(const std::string& signalName) const {
    for (const Signal& signal : signals) {
        if (signal.name == signalName) {
            return &signal;
        }
    }
    return nullptr;
}

Signal& Scene::upsertSignal(const std::string& signalName, SignalType type) {
    if (Signal* existing = findSignal(signalName)) {
        existing->type = type;
        return *existing;
    }
    Signal created;
    created.name = signalName;
    created.type = type;
    signals.push_back(created);
    return signals.back();
}

std::shared_ptr<SceneNode> Scene::find(const std::string& id) const {
    if (!root_) {
        return {};
    }
    std::vector<std::shared_ptr<SceneNode>> stack{root_};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        if (node->id == id) {
            return node;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
    }
    return {};
}

std::shared_ptr<SceneNode> Scene::makeNode(NodeKind kind, const std::string& nodeName) {
    auto node = std::make_shared<SceneNode>();
    node->id = "n" + std::to_string(nextNode_++);
    node->name = nodeName.empty() ? (kind == NodeKind::Assembly ? "Assembly" : "Part") : nodeName;
    node->kind = kind;
    return node;
}

namespace {

Quat quatFromAffine(const Mat4& m) {
    const double m00 = m(0, 0);
    const double m01 = m(1, 0);
    const double m02 = m(2, 0);
    const double m10 = m(0, 1);
    const double m11 = m(1, 1);
    const double m12 = m(2, 1);
    const double m20 = m(0, 2);
    const double m21 = m(1, 2);
    const double m22 = m(2, 2);
    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m21 - m12) / s;
        q.y = (m02 - m20) / s;
        q.z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q.w = (m21 - m12) / s;
        q.x = 0.25 * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25 * s;
        q.z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

}  // namespace

bool Scene::reparent(const std::string& id, const std::string& newParentId, bool keepWorld) {
    auto node = find(id);
    auto parent = find(newParentId);
    if (!node || !parent || node == root_ || node == parent) {
        return false;
    }
    for (auto cursor = parent; cursor; cursor = cursor->parent.lock()) {
        if (cursor == node) {
            return false;
        }
    }
    auto oldParent = node->parent.lock();
    if (!oldParent) {
        return false;
    }
    const Mat4 world = node->worldMatrix();
    auto& siblings = oldParent->children;
    const auto it = std::find(siblings.begin(), siblings.end(), node);
    if (it == siblings.end()) {
        return false;
    }
    siblings.erase(it);
    node->parent = parent;
    parent->children.push_back(node);
    if (keepWorld) {
        const Mat4 local = mul(inverseAffine(parent->worldMatrix()), world);
        node->local.translation = {local(3, 0), local(3, 1), local(3, 2)};
        node->local.rotation = quatFromAffine(local);
        node->local.scale = {1, 1, 1};
    }
    notify(ChangeKind::Structure);
    return true;
}

void Scene::addChild(const std::shared_ptr<SceneNode>& parent, const std::shared_ptr<SceneNode>& child) {
    if (!parent || !child) {
        return;
    }
    child->parent = parent;
    parent->children.push_back(child);
    notify(ChangeKind::Structure);
}

std::string Scene::addMesh(std::shared_ptr<MeshData> mesh) {
    if (!mesh) {
        return {};
    }
    mesh->recomputeBounds();
    const std::string id = "m" + std::to_string(nextMesh_++);
    meshes_[id] = std::move(mesh);
    ++structureRevision_;
    return id;
}

std::shared_ptr<MeshData> Scene::mesh(const std::string& id) const {
    const auto it = meshes_.find(id);
    if (it == meshes_.end()) {
        return {};
    }
    return it->second;
}

void Scene::reset(std::shared_ptr<SceneNode> root,
                  std::unordered_map<std::string, std::shared_ptr<MeshData>> meshes,
                  std::string projectName,
                  std::vector<std::string> sources) {
    mute_ = true;
    root_ = std::move(root);
    if (!root_) {
        root_ = std::make_shared<SceneNode>();
        root_->id = "root";
        root_->name = "World";
        root_->kind = NodeKind::Assembly;
    }
    rebind(root_, nullptr);
    meshes_ = std::move(meshes);
    for (auto& [id, mesh] : meshes_) {
        (void)id;
        if (mesh) {
            mesh->recomputeBounds();
        }
    }
    name = std::move(projectName);
    units = "mm";
    sourceFiles = std::move(sources);
    signals.clear();
    scripts.clear();
    activeScript.clear();
    resources.clear();
    logicBlocks.clear();
    robotPaths.clear();
    selected_.clear();
    adoptIds();
    dirty_ = false;
    mute_ = false;
    ++structureRevision_;
    if (onChanged) {
        onChanged(ChangeKind::Structure);
    }
}

void Scene::rebind(const std::shared_ptr<SceneNode>& node, const std::shared_ptr<SceneNode>& parent) {
    if (!node) {
        return;
    }
    node->parent = parent;
    for (const auto& child : node->children) {
        rebind(child, node);
    }
}

void Scene::adoptIds() {
    nextNode_ = 1;
    nextMesh_ = 1;
    if (root_) {
        considerId(*root_, nextNode_);
    }
    for (const auto& [id, mesh] : meshes_) {
        (void)mesh;
        nextMesh_ = std::max(nextMesh_, trailingNumber(id, 'm') + 1);
    }
}

Aabb Scene::worldBounds() const {
    Aabb box;
    if (!root_) {
        return box;
    }
    std::vector<std::shared_ptr<SceneNode>> stack{root_};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node || !node->effectivelyVisible()) {
            continue;
        }
        if (!node->meshId.empty()) {
            if (const auto meshData = mesh(node->meshId)) {
                expandMeshBounds(box, node->worldMatrix(), *meshData);
            }
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
    }
    return box;
}

int Scene::nodeCount() const { return root_ ? countNodes(*root_) : 0; }

void Scene::notify(ChangeKind kind) {
    if (kind == ChangeKind::Structure) {
        ++structureRevision_;
    }
    if (kind != ChangeKind::Selection && kind != ChangeKind::Simulation) {
        dirty_ = true;
    }
    if (!mute_ && onChanged) {
        onChanged(kind);
    }
}

const char* nodeKindName(NodeKind kind) {
    switch (kind) {
        case NodeKind::Assembly:
            return "assembly";
        case NodeKind::Part:
            return "part";
    }
    return "part";
}

bool nodeKindFromName(const std::string& name, NodeKind& out) {
    if (name == "assembly") {
        out = NodeKind::Assembly;
        return true;
    }
    if (name == "part") {
        out = NodeKind::Part;
        return true;
    }
    return false;
}

}  // namespace sim
