#pragma once

#include "sim/core/Kinematics.hpp"
#include "sim/core/Mech.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/core/Transform.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace sim {

enum class NodeKind { Assembly, Part };

enum class ChangeKind { Structure, Transform, Selection, Simulation };

struct ProjectScript {
    std::string name;
    std::string source;
};

struct SceneNode {
    std::string id;
    std::string name;
    NodeKind kind = NodeKind::Part;
    Transform local;
    std::string meshId;
    Vec3 color{0.55, 0.58, 0.66};
    bool visible = true;
    // Runtime clones from an object source. Not written to the project file.
    bool transient = false;
    Mech mech;
    std::weak_ptr<SceneNode> parent;
    std::vector<std::shared_ptr<SceneNode>> children;

    Mat4 worldMatrix() const;
    bool effectivelyVisible() const;
};

class Scene {
public:
    Scene();

    std::shared_ptr<SceneNode> root() const { return root_; }
    const std::string& selectedId() const { return selected_; }
    void setSelected(const std::string& id);

    std::shared_ptr<SceneNode> find(const std::string& id) const;
    std::shared_ptr<SceneNode> findByName(const std::string& name) const;
    std::shared_ptr<SceneNode> makeNode(NodeKind kind, const std::string& name);
    void addChild(const std::shared_ptr<SceneNode>& parent, const std::shared_ptr<SceneNode>& child);
    // Moves a node under a new parent. keepWorld recomputes the local pose.
    bool reparent(const std::string& id, const std::string& newParentId, bool keepWorld);
    std::shared_ptr<SceneNode> spawnClone(const std::shared_ptr<SceneNode>& prototype, const Transform& worldPose);
    bool removeNode(const std::string& id);

    Signal* findSignal(const std::string& name);
    const Signal* findSignal(const std::string& name) const;
    Signal& upsertSignal(const std::string& name, SignalType type);

    std::string addMesh(std::shared_ptr<MeshData> mesh);
    std::shared_ptr<MeshData> mesh(const std::string& id) const;
    const std::unordered_map<std::string, std::shared_ptr<MeshData>>& meshes() const { return meshes_; }

    void clear();
    // Suppress onChanged while a loader inserts many nodes, then emit one Structure event.
    void beginEdit();
    void endEdit();
    // Move another scene's graph into this one and notify once. Used after a background import.
    void steal(Scene& other);
    void reset(std::shared_ptr<SceneNode> root,
               std::unordered_map<std::string, std::shared_ptr<MeshData>> meshes,
               std::string name,
               std::vector<std::string> sourceFiles);

    Aabb worldBounds() const;
    int nodeCount() const;

    std::string name = "Untitled";
    std::string units = "mm";
    std::vector<std::string> sourceFiles;
    std::vector<Signal> signals;
    std::vector<ProjectScript> scripts;
    std::string activeScript;
    std::vector<KinematicResource> resources;
    std::vector<LogicBlock> logicBlocks;
    std::vector<RobotPath> robotPaths;

    bool dirty() const { return dirty_; }
    void markClean() { dirty_ = false; }
    std::uint64_t structureRevision() const { return structureRevision_; }

    // Fired after structural, transform, or selection edits. Not fired while muted.
    std::function<void(ChangeKind)> onChanged;

    void notify(ChangeKind kind);

private:
    void rebind(const std::shared_ptr<SceneNode>& node, const std::shared_ptr<SceneNode>& parent);
    void adoptIds();

    std::shared_ptr<SceneNode> root_;
    std::unordered_map<std::string, std::shared_ptr<MeshData>> meshes_;
    std::string selected_;
    int nextNode_ = 1;
    int nextMesh_ = 1;
    bool dirty_ = false;
    bool mute_ = false;
    std::uint64_t structureRevision_ = 1;
};

const char* nodeKindName(NodeKind kind);
bool nodeKindFromName(const std::string& name, NodeKind& out);

}  // namespace sim
