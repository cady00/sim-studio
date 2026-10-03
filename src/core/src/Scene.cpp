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
    selected_.clear();
    other.root_.reset();
    other.meshes_.clear();
    other.sourceFiles.clear();
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
    if (kind != ChangeKind::Selection) {
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
