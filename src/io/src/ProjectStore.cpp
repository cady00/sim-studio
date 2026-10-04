#include "sim/io/ProjectStore.hpp"

#include "sim/core/Version.hpp"
#include "sim/io/ZipArchive.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <unordered_map>

#include <signal.h>
#include <unistd.h>

namespace sim {
namespace {

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

void appendF32(std::vector<std::uint8_t>& out, float value) {
    static_assert(sizeof(float) == 4, "float must be 4 bytes");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    appendU32(out, bits);
}

std::uint32_t readU32(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
}

float readF32(const std::uint8_t* data) {
    const std::uint32_t bits = readU32(data);
    float value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

std::vector<std::uint8_t> writeMesh(const MeshData& mesh) {
    std::vector<std::uint8_t> out;
    const char magic[8] = {'S', 'I', 'M', 'M', 'E', 'S', 'H', '1'};
    out.insert(out.end(), magic, magic + 8);
    const auto vertices = static_cast<std::uint32_t>(mesh.vertexCount());
    const auto indices = static_cast<std::uint32_t>(mesh.indices.size());
    appendU32(out, 1);
    appendU32(out, vertices);
    appendU32(out, indices);
    for (float value : mesh.positions) {
        appendF32(out, value);
    }
    for (float value : mesh.normals) {
        appendF32(out, value);
    }
    for (std::uint32_t index : mesh.indices) {
        appendU32(out, index);
    }
    return out;
}

bool readMesh(const std::vector<std::uint8_t>& bytes, MeshData& mesh, std::string& error) {
    if (bytes.size() < 20 || std::memcmp(bytes.data(), "SIMMESH1", 8) != 0) {
        error = "mesh cache header is not SIMMESH1";
        return false;
    }
    const std::uint32_t version = readU32(bytes.data() + 8);
    const std::uint32_t vertices = readU32(bytes.data() + 12);
    const std::uint32_t indices = readU32(bytes.data() + 16);
    if (version != 1) {
        error = "unsupported mesh cache version";
        return false;
    }
    const std::size_t need = 20u + static_cast<std::size_t>(vertices) * 6u * 4u + static_cast<std::size_t>(indices) * 4u;
    if (bytes.size() < need) {
        error = "mesh cache is truncated";
        return false;
    }
    const std::uint8_t* cursor = bytes.data() + 20;
    mesh.positions.resize(static_cast<std::size_t>(vertices) * 3u);
    mesh.normals.resize(static_cast<std::size_t>(vertices) * 3u);
    for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
        mesh.positions[i] = readF32(cursor);
        cursor += 4;
    }
    for (std::size_t i = 0; i < mesh.normals.size(); ++i) {
        mesh.normals[i] = readF32(cursor);
        cursor += 4;
    }
    mesh.indices.resize(indices);
    for (std::uint32_t i = 0; i < indices; ++i) {
        mesh.indices[i] = readU32(cursor);
        cursor += 4;
    }
    mesh.recomputeBounds();
    error.clear();
    return true;
}

nlohmann::json writeNode(const SceneNode& node) {
    nlohmann::json json;
    json["id"] = node.id;
    json["name"] = node.name;
    json["kind"] = nodeKindName(node.kind);
    json["visible"] = node.visible;
    json["color"] = {node.color.x, node.color.y, node.color.z};
    json["metallic"] = node.metallic;
    json["roughness"] = node.roughness;
    json["transform"] = {
        {"translation", {node.local.translation.x, node.local.translation.y, node.local.translation.z}},
        {"rotation", {node.local.rotation.w, node.local.rotation.x, node.local.rotation.y, node.local.rotation.z}},
        {"scale", {node.local.scale.x, node.local.scale.y, node.local.scale.z}},
    };
    if (!node.meshId.empty()) {
        json["mesh"] = node.meshId;
    }
    if (!node.faceAxes.empty()) {
        nlohmann::json axes = nlohmann::json::array();
        for (const FaceAxis& axis : node.faceAxes) {
            axes.push_back({{"origin", {axis.origin.x, axis.origin.y, axis.origin.z}},
                            {"direction", {axis.direction.x, axis.direction.y, axis.direction.z}},
                            {"radius", axis.radius}});
        }
        json["faceAxes"] = std::move(axes);
    }
    if (node.mech.kind != MechKind::None) {
        json["mech"] = mechToJson(node.mech);
    }
    json["children"] = nlohmann::json::array();
    for (const auto& child : node.children) {
        if (child && !child->transient) {
            json["children"].push_back(writeNode(*child));
        }
    }
    return json;
}

std::shared_ptr<SceneNode> readNode(const nlohmann::json& json, std::string& error) {
    if (!json.is_object() || !json.contains("id") || !json.contains("name")) {
        error = "scene node is missing id or name";
        return {};
    }
    auto node = std::make_shared<SceneNode>();
    node->id = json.at("id").get<std::string>();
    node->name = json.at("name").get<std::string>();
    NodeKind kind = NodeKind::Part;
    if (json.contains("kind") && !nodeKindFromName(json.at("kind").get<std::string>(), kind)) {
        error = "unknown node kind";
        return {};
    }
    node->kind = kind;
    node->visible = json.value("visible", true);
    if (json.contains("color") && json.at("color").is_array() && json.at("color").size() == 3) {
        node->color = {json.at("color")[0].get<double>(), json.at("color")[1].get<double>(),
                       json.at("color")[2].get<double>()};
    }
    node->metallic = static_cast<float>(json.value("metallic", 0.0));
    node->roughness = static_cast<float>(json.value("roughness", 1.0));
    if (json.contains("transform")) {
        const auto& transform = json.at("transform");
        const auto& translation = transform.at("translation");
        const auto& rotation = transform.at("rotation");
        const auto& scale = transform.at("scale");
        node->local.translation = {translation[0].get<double>(), translation[1].get<double>(),
                                   translation[2].get<double>()};
        node->local.rotation = Quat{rotation[0].get<double>(), rotation[1].get<double>(), rotation[2].get<double>(),
                                    rotation[3].get<double>()}
                                   .normalized();
        node->local.scale = {scale[0].get<double>(), scale[1].get<double>(), scale[2].get<double>()};
    }
    if (json.contains("mesh")) {
        node->meshId = json.at("mesh").get<std::string>();
    }
    if (json.contains("faceAxes") && json.at("faceAxes").is_array()) {
        for (const auto& axisJson : json.at("faceAxes")) {
            if (!axisJson.is_object()) {
                continue;
            }
            FaceAxis axis;
            if (axisJson.contains("origin") && axisJson.at("origin").is_array() && axisJson.at("origin").size() >= 3) {
                axis.origin = {axisJson.at("origin")[0].get<double>(), axisJson.at("origin")[1].get<double>(),
                               axisJson.at("origin")[2].get<double>()};
            }
            if (axisJson.contains("direction") && axisJson.at("direction").is_array() && axisJson.at("direction").size() >= 3) {
                axis.direction = {axisJson.at("direction")[0].get<double>(), axisJson.at("direction")[1].get<double>(),
                                  axisJson.at("direction")[2].get<double>()};
            }
            axis.radius = axisJson.value("radius", 0.0);
            if (axis.radius > 0) {
                node->faceAxes.push_back(axis);
            }
        }
    }
    if (json.contains("mech")) {
        mechFromJson(json.at("mech"), node->mech);
    }
    if (json.contains("children")) {
        for (const auto& childJson : json.at("children")) {
            auto child = readNode(childJson, error);
            if (!child) {
                return {};
            }
            child->parent = node;
            node->children.push_back(std::move(child));
        }
    }
    error.clear();
    return node;
}

}  // namespace

ProjectResult ProjectStore::save(const Scene& scene, const std::string& path) {
    ProjectResult result;
    if (!scene.root()) {
        result.message = "scene has no root";
        return result;
    }
    nlohmann::json json;
    json["format"] = kProjectFormatId;
    json["version"] = kProjectFormatVersion;
    json["name"] = scene.name;
    json["units"] = scene.units;
    json["upAxis"] = "Z";
    json["sources"] = scene.sourceFiles;
    json["activeScript"] = scene.activeScript;
    json["kinematics"] = nlohmann::json::array();
    for (const KinematicResource& resource : scene.resources) {
        json["kinematics"].push_back(kinematicsToJson(resource));
    }
    json["logic"] = nlohmann::json::array();
    for (const LogicBlock& block : scene.logicBlocks) {
        json["logic"].push_back(logicBlockToJson(block));
    }
    json["welds"] = weldsToJson(scene.welds);
    json["envelopes"] = envelopesToJson(scene.envelopes);
    json["interlocks"] = interlocksToJson(scene.interlocks);
    json["paths"] = nlohmann::json::array();
    for (const RobotPath& path : scene.robotPaths) {
        json["paths"].push_back(robotPathToJson(path));
    }
    json["proximity"] = proximityToJson(scene.proximity);
    json["convexCache"] = convexCacheToJson(scene.convexCache);
    json["physics"] = physicsToJson(scene.physicsDoc);
    json["motors"] = nlohmann::json::array();
    for (const MotorAspect& motor : scene.motors) {
        json["motors"].push_back(motorToJson(motor));
    }
    json["layout"] = layoutToJson(scene.layout);
    json["operations"] = operationsToJson(scene.operations);
    json["sequence"] = sequenceToJson(scene.sequence);
    json["ppr"] = pprToJson(scene.ppr);
    json["signals"] = nlohmann::json::array();
    for (const Signal& signal : scene.signals) {
        json["signals"].push_back(signalToJson(signal));
    }
    json["scripts"] = nlohmann::json::array();
    for (const ProjectScript& script : scene.scripts) {
        json["scripts"].push_back(nlohmann::json{{"name", script.name}, {"source", script.source}});
    }
    json["root"] = writeNode(*scene.root());

    std::vector<ZipEntry> entries;
    entries.push_back(ZipEntry{"project.json", {}});
    const std::string text = json.dump(2);
    entries.back().data.assign(text.begin(), text.end());
    for (const auto& [id, mesh] : scene.meshes()) {
        if (!mesh || mesh->empty()) {
            continue;
        }
        entries.push_back(ZipEntry{"meshes/" + id + ".bin", writeMesh(*mesh)});
    }
    if (!scene.layout.underlay.embedded.empty() && !scene.layout.underlay.bytes.empty()) {
        entries.push_back(ZipEntry{scene.layout.underlay.embedded, scene.layout.underlay.bytes});
    }
    if (!ZipArchive::write(path, entries, result.message)) {
        return result;
    }
    result.ok = true;
    result.message.clear();
    return result;
}

ProjectResult ProjectStore::load(const std::string& path, Scene& scene) {
    ProjectResult result;
    std::vector<ZipEntry> entries;
    if (!ZipArchive::read(path, entries, result.message)) {
        return result;
    }
    const ZipEntry* project = nullptr;
    std::unordered_map<std::string, const ZipEntry*> meshes;
    for (const ZipEntry& entry : entries) {
        if (entry.name == "project.json") {
            project = &entry;
        } else if (entry.name.rfind("meshes/", 0) == 0) {
            meshes.emplace(entry.name, &entry);
        }
    }
    if (project == nullptr) {
        result.message = "project.json is missing";
        return result;
    }
    nlohmann::json json;
    try {
        json = nlohmann::json::parse(project->data.begin(), project->data.end());
    } catch (const std::exception& ex) {
        result.message = ex.what();
        return result;
    }
    if (json.value("format", "") != kProjectFormatId) {
        result.message = "not a Sim Studio project";
        return result;
    }
    const int version = json.value("version", 0);
    if (version < 1 || version > kProjectFormatVersion) {
        result.message = "unsupported project version";
        return result;
    }
    std::string error;
    auto root = readNode(json.at("root"), error);
    if (!root) {
        result.message = error.empty() ? "could not read the scene root" : error;
        return result;
    }
    std::unordered_map<std::string, std::shared_ptr<MeshData>> meshMap;
    std::vector<std::shared_ptr<SceneNode>> stack{root};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->meshId.empty() || meshMap.count(node->meshId) != 0) {
            continue;
        }
        const std::string meshPath = "meshes/" + node->meshId + ".bin";
        const auto it = meshes.find(meshPath);
        if (it == meshes.end()) {
            result.message = "missing mesh cache " + meshPath;
            return result;
        }
        auto mesh = std::make_shared<MeshData>();
        if (!readMesh(it->second->data, *mesh, error)) {
            result.message = error;
            return result;
        }
        meshMap.emplace(node->meshId, std::move(mesh));
    }
    std::vector<std::string> sources;
    if (json.contains("sources") && json.at("sources").is_array()) {
        for (const auto& source : json.at("sources")) {
            if (source.is_string()) {
                sources.push_back(source.get<std::string>());
            }
        }
    }
    scene.reset(std::move(root), std::move(meshMap), json.value("name", std::string("Untitled")), std::move(sources));
    if (json.contains("units") && json.at("units").is_string()) {
        scene.units = json.at("units").get<std::string>();
    }
    scene.activeScript = json.value("activeScript", std::string{});
    if (json.contains("signals") && json.at("signals").is_array()) {
        for (const auto& item : json.at("signals")) {
            Signal signal;
            signalFromJson(item, signal);
            if (!signal.name.empty()) {
                scene.signals.push_back(signal);
            }
        }
    }
    if (json.contains("scripts") && json.at("scripts").is_array()) {
        for (const auto& item : json.at("scripts")) {
            if (!item.is_object()) {
                continue;
            }
            ProjectScript script;
            script.name = item.value("name", std::string{});
            script.source = item.value("source", std::string{});
            if (!script.name.empty()) {
                scene.scripts.push_back(std::move(script));
            }
        }
    }
    if (json.contains("kinematics") && json.at("kinematics").is_array()) {
        for (const auto& item : json.at("kinematics")) {
            KinematicResource resource;
            if (!kinematicsFromJson(item, resource, error)) {
                result.message = error;
                return result;
            }
            scene.resources.push_back(std::move(resource));
        }
    }
    if (json.contains("logic") && json.at("logic").is_array()) {
        for (const auto& item : json.at("logic")) {
            LogicBlock block;
            if (!logicBlockFromJson(item, block, error)) {
                result.message = error;
                return result;
            }
            scene.logicBlocks.push_back(std::move(block));
        }
    }
    if (json.contains("paths") && json.at("paths").is_array()) {
        for (const auto& item : json.at("paths")) {
            RobotPath path;
            if (!robotPathFromJson(item, path, error)) {
                result.message = error;
                return result;
            }
            scene.robotPaths.push_back(std::move(path));
        }
    }
    if (json.contains("welds")) {
        weldsFromJson(json.at("welds"), scene.welds);
    }
    if (json.contains("envelopes")) {
        envelopesFromJson(json.at("envelopes"), scene.envelopes);
    }
    if (json.contains("interlocks")) {
        interlocksFromJson(json.at("interlocks"), scene.interlocks);
    }
    if (json.contains("proximity")) {
        proximityFromJson(json.at("proximity"), scene.proximity);
    }
    if (json.contains("convexCache")) {
        convexCacheFromJson(json.at("convexCache"), scene.convexCache);
    }
    if (json.contains("physics")) {
        physicsFromJson(json.at("physics"), scene.physicsDoc);
    }
    if (json.contains("motors") && json.at("motors").is_array()) {
        for (const auto& item : json.at("motors")) {
            MotorAspect motor;
            motorFromJson(item, motor);
            scene.motors.push_back(std::move(motor));
        }
    }
    if (json.contains("layout")) {
        layoutFromJson(json.at("layout"), scene.layout);
        if (!scene.layout.underlay.embedded.empty()) {
            for (const ZipEntry& entry : entries) {
                if (entry.name == scene.layout.underlay.embedded) {
                    scene.layout.underlay.bytes = entry.data;
                    break;
                }
            }
        }
    }
    if (json.contains("operations")) {
        operationsFromJson(json.at("operations"), scene.operations);
    }
    if (json.contains("sequence")) {
        sequenceFromJson(json.at("sequence"), scene.sequence);
    }
    if (json.contains("ppr")) {
        pprFromJson(json.at("ppr"), scene.ppr);
    }
    // reset() notifies before kinematics, welds, and operations are copied in.
    scene.notify(ChangeKind::Structure);
    result.ok = true;
    result.message.clear();
    return result;
}

namespace {

std::filesystem::path historyTempPath() {
    static std::atomic<unsigned> sequence{0};
    const unsigned id = sequence.fetch_add(1);
    return std::filesystem::temp_directory_path() /
           ("simstudio-hist-" + std::to_string(static_cast<long>(::getpid())) + "-" + std::to_string(id) + ".simproj");
}

struct TempProject {
    std::filesystem::path path;
    ~TempProject() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};

}  // namespace

ProjectResult ProjectStore::saveBytes(const Scene& scene, std::vector<std::uint8_t>& out) {
    TempProject temp{historyTempPath()};
    ProjectResult result = save(scene, temp.path.string());
    if (!result.ok) {
        return result;
    }
    std::ifstream in(temp.path, std::ios::binary);
    if (!in) {
        result.ok = false;
        result.message = "could not read the history snapshot";
        return result;
    }
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    if (out.empty()) {
        result.ok = false;
        result.message = "history snapshot is empty";
    }
    return result;
}

ProjectResult ProjectStore::loadBytes(const std::vector<std::uint8_t>& bytes, Scene& scene) {
    ProjectResult result;
    if (bytes.empty()) {
        result.message = "history snapshot is empty";
        return result;
    }
    TempProject temp{historyTempPath()};
    {
        std::ofstream file(temp.path, std::ios::binary);
        if (!file) {
            result.message = "could not write the history snapshot";
            return result;
        }
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if (!file) {
            result.message = "could not write the history snapshot";
            return result;
        }
    }
    return load(temp.path.string(), scene);
}

namespace {

std::filesystem::path recoveryFile(const std::string& directory, const char* name) {
    return std::filesystem::path(directory) / name;
}

}  // namespace

ProjectResult ProjectStore::autosave(const Scene& scene, const std::string& directory) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        ProjectResult result;
        result.message = error.message();
        return result;
    }
    return save(scene, recoveryFile(directory, "autosave.simproj").string());
}

void ProjectStore::markSession(const std::string& directory) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    std::ofstream out(recoveryFile(directory, "session.pid"));
    if (out) {
        out << static_cast<long>(getpid());
    }
}

void ProjectStore::clearSession(const std::string& directory) {
    std::error_code error;
    std::filesystem::remove(recoveryFile(directory, "session.pid"), error);
    std::filesystem::remove(recoveryFile(directory, "autosave.simproj"), error);
}

bool ProjectStore::sessionInterrupted(const std::string& directory) {
    std::ifstream in(recoveryFile(directory, "session.pid"));
    if (!in) {
        return false;
    }
    long pid = 0;
    in >> pid;
    if (pid <= 0 || pid == static_cast<long>(getpid())) {
        return false;
    }
    if (kill(static_cast<pid_t>(pid), 0) == 0) {
        return false;
    }
    return errno == ESRCH;
}

ProjectResult ProjectStore::restoreAutosave(const std::string& directory, Scene& scene) {
    const std::filesystem::path path = recoveryFile(directory, "autosave.simproj");
    if (!std::filesystem::exists(path)) {
        ProjectResult result;
        result.message = "no autosave";
        return result;
    }
    return load(path.string(), scene);
}

}  // namespace sim
