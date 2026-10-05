#include "sim/motion/Motion.hpp"

#include "sim/core/Mesh.hpp"

#include <curl/curl.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>

namespace sim {
namespace {

struct JointRow {
    const char* name;
    double x;
    double y;
    double z;
    double ax;
    double ay;
    double az;
    double lower;
    double upper;
    double speed;
};

struct Model {
    const char* name;
    const char* source;
    Vec3 color;
    Vec3 flange;
    JointRow joints[6];
};

// Link origins are the URDF joint origins, converted from metres to millimetres.
// Signed limits and speeds are the datasheet numbers named in `source`.
// Where a page only prints a stroke, the signed pair is the one in the public xacro, and the note says so.
const Model kModels[] = {
    {"KR6_R900",
     "待核实。限位和速度：KUKA《KR AGILUS sixx》4.2 节（A1 ±170°/360°/s，A2 -190°/+45°/300°/s，A3 -120°/+156°/360°/s，A4 ±185°/381°/s，A5 ±120°/388°/s，A6 ±350°/615°/s），"
     "https://www.robostav.cz/cvut/robostav/download/KR_AGILUS_sixx_en.pdf ；产品页 "
     "https://my.kuka.com/s/product/kr-6-r900-sixx/01t58000003eVQqAAM?language=en_US 。"
     "连杆原点取 ROS-Industrial xacro 的关节 origin（米已换成毫米）："
     "https://github.com/ros-industrial/kuka_experimental/blob/indigo-devel/kuka_kr6_support/urdf/kr6r900sixx_macro.xacro 。"
     "公开样本没有关节扭矩和惯量，加速度来源是速度。连杆用 ROS-Industrial kuka_kr6_support 的碰撞 STL。"
     "该包声明 BSD，没有单独的网格再分发许可，所以不入库；首次使用从公开地址下载到本地缓存，失败则该连杆仍是方盒。",
     {0.92, 0.45, 0.08},
     {0, 0, 0},
     {{"A1", 0, 0, 400, 0, 0, -1, -170, 170, 360},
      {"A2", 25, 0, 0, 0, 1, 0, -190, 45, 300},
      {"A3", 455, 0, 0, 0, 1, 0, -120, 156, 360},
      {"A4", 0, 0, 35, -1, 0, 0, -185, 185, 381},
      {"A5", 420, 0, 0, 0, 1, 0, -120, 120, 388},
      {"A6", 80, 0, 0, -1, 0, 0, -350, 350, 615}}},
    {"IRB120",
     "待核实。限位和速度：ABB Product specification IRB 120（3HAC035960）1.8.1，以及样本 "
     "https://library.e.abb.com/public/7aa0711a20fa41c49a8fdf3bbc3d5bb0/IRB120-Rev.J-ROBO149EN_D.pdf "
     "（轴1 ±165°/250°/s，轴2 ±110°/250°/s，轴3 +70°/-110°/250°/s，轴4 ±160°/320°/s，轴5 ±120°/320°/s，轴6 ±400°/420°/s）。"
     "连杆原点取 ROS-Industrial xacro："
     "https://github.com/ros-industrial/abb/blob/kinetic-devel/abb_irb120_support/urdf/irb120_3_58_macro.xacro 。"
     "公开样本没有关节扭矩和惯量，加速度来源是速度。连杆用 ROS-Industrial abb_irb120_support 的视觉 STL。"
     "包声明 Apache-2.0，说明里写惯量来自 ABB 提供的 CAD，网格再分发许可不确定，所以不入库；首次使用下载到本地缓存，失败则该连杆仍是方盒。",
     {0.86, 0.42, 0.12},
     {0, 0, 0},
     {{"A1", 0, 0, 0, 0, 0, 1, -165, 165, 250},
      {"A2", 0, 0, 290, 0, 1, 0, -110, 110, 250},
      {"A3", 0, 0, 270, 0, 1, 0, -110, 70, 250},
      {"A4", 0, 0, 70, 1, 0, 0, -160, 160, 320},
      {"A5", 302, 0, 0, 0, 1, 0, -120, 120, 320},
      {"A6", 72, 0, 0, 1, 0, 0, -400, 400, 420}}},
    {"LRMate200iD",
     "待核实。行程和速度的合计见 FANUC America LR Mate 200iD 产品页 "
     "https://www.fanucamerica.com/products/robot/lr-mate-200id "
     "（J1 340°/450°/s，J2 245°/380°/s，J3 420°/520°/s，J4 380°/550°/s，J5 250°/545°/s，J6 720°/1000°/s）。"
     "带正负号的限位和连杆原点取 ROS-Industrial xacro："
     "https://github.com/ros-industrial/fanuc/blob/noetic-devel/fanuc_lrmate200id_support/urdf/lrmate200id_macro.xacro 。"
     "该文件把 J3 写成 -70° 到 +205°，跨度 275°，和产品页的 420° 不一致，以用户核对的手册为准。"
     "公开样本没有关节扭矩和惯量，加速度来源是速度。连杆用 ROS-Industrial fanuc_lrmate200id_support 的视觉 STL。"
     "该包声明 BSD，没有单独的网格再分发许可，所以不入库；首次使用下载到本地缓存，失败则该连杆仍是方盒。",
     {0.93, 0.78, 0.12},
     {0, 0, 0},
     {{"A1", 0, 0, 330, 0, 0, 1, -170, 170, 450},
      {"A2", 50, 0, 0, 0, 1, 0, -100, 145, 380},
      {"A3", 0, 0, 330, 0, -1, 0, -70, 205, 520},
      {"A4", 0, 0, 35, -1, 0, 0, -190, 190, 550},
      {"A5", 335, 0, 0, 0, -1, 0, -125, 125, 545},
      {"A6", 80, 0, 0, -1, 0, 0, -360, 360, 1000}}},
    {"GP8",
     "待核实。限位和速度：Yaskawa Motoman GP7/GP8 样本 "
     "https://www.motoman.com/getmedia/1a40ce78-99c3-4e43-bbce-b9318263f464/GP7_GP8.pdf.aspx "
     "（S ±170°/455°/s，L +145°/-65°/385°/s，U +190°/-70°/520°/s，R ±190°/550°/s，B ±135°/550°/s，T ±360°/1000°/s），"
     "速度与 https://www.yaskawa.eu.com/robotics/robots/handling-mounting/productdetail/product/gp8_694 一致。"
     "另一份转载样本把 L 写成 +150°/-65°、U 写成 +255°/-113°，这里没有采用。"
     "连杆原点取 ROS-Industrial xacro："
     "https://github.com/ros-industrial/motoman/blob/kinetic-devel/motoman_gp8_support/urdf/gp8_macro.xacro 。"
     "法兰在 T 轴前方 80 mm，没有单独的公开网格，仍是方盒。公开样本没有关节扭矩和惯量，加速度来源是速度。"
     "连杆用 ROS-Industrial motoman_gp8_support 的视觉 STL。该包声明 BSD-3-Clause，没有单独的网格再分发许可，所以不入库；首次使用下载到本地缓存，失败则该连杆仍是方盒。",
     {0.16, 0.36, 0.72},
     {80, 0, 0},
     {{"A1", 0, 0, 330, 0, 0, 1, -170, 170, 455},
      {"A2", 40, 0, 0, 0, 1, 0, -65, 145, 385},
      {"A3", 0, 0, 345, 0, -1, 0, -70, 190, 520},
      {"A4", 340, 0, 40, -1, 0, 0, -190, 190, 550},
      {"A5", 0, 0, 0, 0, -1, 0, -135, 135, 550},
      {"A6", 0, 0, 0, -1, 0, 0, -360, 360, 1000}}},
};

const Model* findModel(const std::string& name) {
    for (const Model& model : kModels) {
        if (name == model.name) {
            return &model;
        }
    }
    return nullptr;
}

struct MeshSource {
    const char* model;
    const char* link;
    const char* file;
    const char* url;
};

const char* kKuka = "https://raw.githubusercontent.com/ros-industrial/kuka_experimental/indigo-devel/kuka_kr6_support/meshes/";
const char* kAbb = "https://raw.githubusercontent.com/ros-industrial/abb/kinetic-devel/abb_irb120_support/meshes/irb120_3_58/visual/";
const char* kFanuc = "https://raw.githubusercontent.com/ros-industrial/fanuc/noetic-devel/fanuc_lrmate200id_support/meshes/lrmate200id/visual/";
const char* kGp8 = "https://raw.githubusercontent.com/ros-industrial/motoman/noetic-devel/motoman_gp8_support/meshes/visual/";

std::string joinUrl(const char* prefix, const char* path) { return std::string(prefix) + path; }

const MeshSource& meshTable(std::size_t& count) {
    static const std::string kr6Base = joinUrl(kKuka, "kr6_agilus/collision/base_link.stl");
    static const std::string kr6L1 = joinUrl(kKuka, "kr6_agilus/collision/link_1.stl");
    static const std::string kr6L2 = joinUrl(kKuka, "kr6r900sixx/collision/link_2.stl");
    static const std::string kr6L3 = joinUrl(kKuka, "kr6_agilus/collision/link_3.stl");
    static const std::string kr6L4 = joinUrl(kKuka, "kr6r900sixx/collision/link_4.stl");
    static const std::string kr6L5 = joinUrl(kKuka, "kr6_agilus/collision/link_5.stl");
    static const std::string kr6L6 = joinUrl(kKuka, "kr6_agilus/collision/link_6.stl");
    static const std::string irbBase = joinUrl(kAbb, "base_link.stl");
    static const std::string irb1 = joinUrl(kAbb, "link_1.stl");
    static const std::string irb2 = joinUrl(kAbb, "link_2.stl");
    static const std::string irb3 = joinUrl(kAbb, "link_3.stl");
    static const std::string irb4 = joinUrl(kAbb, "link_4.stl");
    static const std::string irb5 = joinUrl(kAbb, "link_5.stl");
    static const std::string irb6 = joinUrl(kAbb, "link_6.stl");
    static const std::string fanBase = joinUrl(kFanuc, "base_link.stl");
    static const std::string fan1 = joinUrl(kFanuc, "link_1.stl");
    static const std::string fan2 = joinUrl(kFanuc, "link_2.stl");
    static const std::string fan3 = joinUrl(kFanuc, "link_3.stl");
    static const std::string fan4 = joinUrl(kFanuc, "link_4.stl");
    static const std::string fan5 = joinUrl(kFanuc, "link_5.stl");
    static const std::string fan6 = joinUrl(kFanuc, "link_6.stl");
    static const std::string gpBase = joinUrl(kGp8, "gp8_base_link.stl");
    static const std::string gp1 = joinUrl(kGp8, "gp8_link_1_s.stl");
    static const std::string gp2 = joinUrl(kGp8, "gp8_link_2_l.stl");
    static const std::string gp3 = joinUrl(kGp8, "gp8_link_3_u.stl");
    static const std::string gp4 = joinUrl(kGp8, "gp8_link_4_r.stl");
    static const std::string gp5 = joinUrl(kGp8, "gp8_link_5_b.stl");
    static const std::string gp6 = joinUrl(kGp8, "gp8_link_6_t.stl");
    static const MeshSource table[] = {
        {"KR6_R900", "base", "base_link.stl", kr6Base.c_str()}, {"KR6_R900", "A1", "link_1.stl", kr6L1.c_str()},
        {"KR6_R900", "A2", "link_2.stl", kr6L2.c_str()},         {"KR6_R900", "A3", "link_3.stl", kr6L3.c_str()},
        {"KR6_R900", "A4", "link_4.stl", kr6L4.c_str()},         {"KR6_R900", "A5", "link_5.stl", kr6L5.c_str()},
        {"KR6_R900", "A6", "link_6.stl", kr6L6.c_str()},         {"IRB120", "base", "base_link.stl", irbBase.c_str()},
        {"IRB120", "A1", "link_1.stl", irb1.c_str()},            {"IRB120", "A2", "link_2.stl", irb2.c_str()},
        {"IRB120", "A3", "link_3.stl", irb3.c_str()},            {"IRB120", "A4", "link_4.stl", irb4.c_str()},
        {"IRB120", "A5", "link_5.stl", irb5.c_str()},            {"IRB120", "A6", "link_6.stl", irb6.c_str()},
        {"LRMate200iD", "base", "base_link.stl", fanBase.c_str()}, {"LRMate200iD", "A1", "link_1.stl", fan1.c_str()},
        {"LRMate200iD", "A2", "link_2.stl", fan2.c_str()},       {"LRMate200iD", "A3", "link_3.stl", fan3.c_str()},
        {"LRMate200iD", "A4", "link_4.stl", fan4.c_str()},       {"LRMate200iD", "A5", "link_5.stl", fan5.c_str()},
        {"LRMate200iD", "A6", "link_6.stl", fan6.c_str()},       {"GP8", "base", "base_link.stl", gpBase.c_str()},
        {"GP8", "A1", "link_1.stl", gp1.c_str()},                {"GP8", "A2", "link_2.stl", gp2.c_str()},
        {"GP8", "A3", "link_3.stl", gp3.c_str()},                {"GP8", "A4", "link_4.stl", gp4.c_str()},
        {"GP8", "A5", "link_5.stl", gp5.c_str()},                {"GP8", "A6", "link_6.stl", gp6.c_str()},
    };
    count = std::size(table);
    return table[0];
}

const MeshSource* findMesh(const std::string& model, const std::string& link) {
    std::size_t count = 0;
    const MeshSource* table = &meshTable(count);
    for (std::size_t i = 0; i < count; ++i) {
        if (model == table[i].model && link == table[i].link) {
            return &table[i];
        }
    }
    return nullptr;
}

bool envFlag(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && std::strcmp(value, "0") != 0;
}

std::filesystem::path meshRoot() {
    if (const char* pinned = std::getenv("SIM_CATALOG_MESH_DIR"); pinned != nullptr && pinned[0] != '\0') {
        return pinned;
    }
    if (const char* xdg = std::getenv("XDG_DATA_HOME"); xdg != nullptr && xdg[0] != '\0') {
        return std::filesystem::path(xdg) / "SimStudio" / "catalog-meshes";
    }
    if (const char* home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
        return std::filesystem::path(home) / ".local" / "share" / "SimStudio" / "catalog-meshes";
    }
    return std::filesystem::temp_directory_path() / "SimStudio" / "catalog-meshes";
}

bool readFile(const std::filesystem::path& path, std::vector<std::uint8_t>& bytes) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0 || static_cast<std::uint64_t>(size) > 32ull * 1024ull * 1024ull) {
        return false;
    }
    in.seekg(0);
    bytes.resize(static_cast<std::size_t>(size));
    if (!bytes.empty()) {
        in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    return static_cast<bool>(in) || in.eof();
}

struct Download {
    std::vector<std::uint8_t> bytes;
    bool overflow = false;
};

std::size_t writeDownload(char* data, std::size_t size, std::size_t count, void* user) {
    auto* download = static_cast<Download*>(user);
    const std::size_t bytes = size * count;
    if (download->bytes.size() + bytes > 32ull * 1024ull * 1024ull) {
        download->overflow = true;
        return 0;
    }
    download->bytes.insert(download->bytes.end(), reinterpret_cast<std::uint8_t*>(data),
                           reinterpret_cast<std::uint8_t*>(data) + bytes);
    return bytes;
}

bool downloadHttps(const std::string& url, std::vector<std::uint8_t>& bytes) {
    if (url.rfind("https://", 0) != 0) {
        return false;
    }
    CURL* curl = curl_easy_init();
    if (curl == nullptr) {
        return false;
    }
    Download download;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "SimStudio");
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeDownload);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &download);
    const CURLcode code = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
    curl_easy_cleanup(curl);
    if (code != CURLE_OK || download.overflow || status < 200 || status >= 300 || download.bytes.empty()) {
        return false;
    }
    bytes = std::move(download.bytes);
    return true;
}

void addTriangle(MeshData& mesh, Vec3 a, Vec3 b, Vec3 c) {
    const Vec3 normal = normalize(cross(b - a, c - a));
    const std::uint32_t base = static_cast<std::uint32_t>(mesh.vertexCount());
    const Vec3 points[] = {a, b, c};
    for (const Vec3& point : points) {
        mesh.positions.push_back(static_cast<float>(point.x * 1000.0));
        mesh.positions.push_back(static_cast<float>(point.y * 1000.0));
        mesh.positions.push_back(static_cast<float>(point.z * 1000.0));
        mesh.normals.push_back(static_cast<float>(normal.x));
        mesh.normals.push_back(static_cast<float>(normal.y));
        mesh.normals.push_back(static_cast<float>(normal.z));
    }
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1);
    mesh.indices.push_back(base + 2);
}

std::shared_ptr<MeshData> meshFromStl(const std::vector<std::uint8_t>& bytes) {
    auto mesh = std::make_shared<MeshData>();
    if (bytes.size() >= 84) {
        std::uint32_t count = 0;
        std::memcpy(&count, bytes.data() + 80, 4);
        const std::uint64_t expected = 84ull + static_cast<std::uint64_t>(count) * 50ull;
        if (count > 0 && count <= 2000000 && bytes.size() >= expected && bytes.size() - expected < 128) {
            for (std::uint32_t i = 0; i < count; ++i) {
                const std::uint8_t* tri = bytes.data() + 84 + static_cast<std::uint64_t>(i) * 50;
                float value[12];
                std::memcpy(value, tri, sizeof(value));
                addTriangle(*mesh, {value[3], value[4], value[5]}, {value[6], value[7], value[8]},
                            {value[9], value[10], value[11]});
            }
            mesh->recomputeBounds();
            return mesh->empty() ? nullptr : mesh;
        }
    }
    const std::string text(bytes.begin(), bytes.end());
    if (text.find("solid") == std::string::npos || text.find("vertex") == std::string::npos) {
        return nullptr;
    }
    std::istringstream stream(text);
    std::string word;
    std::vector<Vec3> vertices;
    while (stream >> word) {
        if (word != "vertex") {
            continue;
        }
        double x = 0;
        double y = 0;
        double z = 0;
        if (!(stream >> x >> y >> z)) {
            break;
        }
        vertices.push_back({x, y, z});
    }
    if (vertices.size() < 3 || vertices.size() % 3 != 0 || vertices.size() / 3 > 2000000) {
        return nullptr;
    }
    for (std::size_t i = 0; i + 2 < vertices.size(); i += 3) {
        addTriangle(*mesh, vertices[i], vertices[i + 1], vertices[i + 2]);
    }
    mesh->recomputeBounds();
    return mesh->empty() ? nullptr : mesh;
}

std::shared_ptr<MeshData> loadCatalogMesh(const std::string& model, const std::string& link) {
    const MeshSource* source = findMesh(model, link);
    if (source == nullptr) {
        return nullptr;
    }
    const std::filesystem::path path = meshRoot() / model / source->file;
    std::vector<std::uint8_t> bytes;
    if (!readFile(path, bytes)) {
        const bool pinned = std::getenv("SIM_CATALOG_MESH_DIR") != nullptr && std::getenv("SIM_CATALOG_MESH_DIR")[0] != '\0';
        if (pinned || envFlag("SIM_CATALOG_MESH_OFFLINE") || !downloadHttps(source->url, bytes)) {
            return nullptr;
        }
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        const auto temporary = path.string() + ".part";
        {
            std::ofstream out(temporary, std::ios::binary);
            out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            if (!out) {
                return meshFromStl(bytes);
            }
        }
        std::filesystem::rename(temporary, path, error);
        if (error) {
            std::filesystem::remove(temporary);
        }
    }
    return meshFromStl(bytes);
}

bool attachMesh(Scene& scene, const std::shared_ptr<SceneNode>& link, const std::string& model, const std::string& linkName,
                Vec3 color) {
    auto mesh = loadCatalogMesh(model, linkName);
    if (!mesh) {
        return false;
    }
    auto part = scene.makeNode(NodeKind::Part, model + "_" + linkName);
    part->meshId = scene.addMesh(std::move(mesh));
    part->color = color;
    scene.addChild(link, part);
    return true;
}

void addBar(Scene& scene, const std::shared_ptr<SceneNode>& link, const std::string& name, Vec3 along, Vec3 color) {
    if (!link) {
        return;
    }
    const double span = std::max(length(along), 36.0);
    Vec3 half{18, 18, span * 0.5};
    const Vec3 direction = length(along) < 1 ? Vec3{0, 0, 1} : normalize(along);
    if (std::abs(direction.x) >= std::abs(direction.y) && std::abs(direction.x) >= std::abs(direction.z)) {
        half = {span * 0.5, 18, 18};
    } else if (std::abs(direction.y) >= std::abs(direction.z)) {
        half = {18, span * 0.5, 18};
    }
    auto bar = scene.makeNode(NodeKind::Part, name);
    bar->meshId = scene.addMesh(makeBoxMesh(half));
    bar->color = color;
    bar->local.translation = length(along) < 1 ? Vec3{0, 0, 18} : along * 0.5;
    scene.addChild(link, bar);
}

}  // namespace

const std::vector<CatalogRobot>& robotCatalog() {
    static const std::vector<CatalogRobot> catalog = [] {
        std::vector<CatalogRobot> rows;
        rows.reserve(std::size(kModels));
        for (const Model& model : kModels) {
            rows.push_back(CatalogRobot{model.name, model.source});
        }
        return rows;
    }();
    return catalog;
}

bool addCatalogRobot(Scene& scene, const std::string& name, Vec3 station, std::string& error) {
    const Model* model = findModel(name);
    if (model == nullptr) {
        error = "no catalog robot named " + name;
        return false;
    }
    if (findResource(scene.resources, name) != nullptr) {
        error = name + " is already in the scene";
        return false;
    }
    KinematicResource& resource = ensureResource(scene, name);
    resource.source = model->source;
    resource.ik = "none";
    Transform baseLocal;
    baseLocal.translation = station;
    addLink(scene, resource, "base", "", baseLocal);
    int meshed = 0;
    auto placeMesh = [&](const std::string& linkName) {
        if (KinLink* link = findLink(resource, linkName)) {
            if (attachMesh(scene, scene.find(link->nodeId), model->name, linkName, model->color)) {
                ++meshed;
                return true;
            }
        }
        return false;
    };
    std::string parent = "base";
    for (const JointRow& row : model->joints) {
        Transform local;
        local.translation = {row.x, row.y, row.z};
        addLink(scene, resource, row.name, parent, local);
        KinJoint& joint = addJoint(resource, scene, row.name, parent, row.name, JointMotion::Revolute,
                                   {row.ax, row.ay, row.az}, local.translation);
        joint.lower = row.lower;
        joint.upper = row.upper;
        joint.maxSpeed = row.speed;
        joint.maxAccel = 0;
        joint.accelSource = "velocity";
        if (!placeMesh(parent)) {
            if (KinLink* parentLink = findLink(resource, parent)) {
                addBar(scene, scene.find(parentLink->nodeId), std::string(model->name) + "_" + parent, local.translation,
                       model->color);
            }
        }
        parent = row.name;
    }
    placeMesh(parent);
    if (length(model->flange) > 1) {
        Transform flange;
        flange.translation = model->flange;
        addLink(scene, resource, "flange", parent, flange);
        if (KinLink* wrist = findLink(resource, parent)) {
            addBar(scene, scene.find(wrist->nodeId), std::string(model->name) + "_flange", model->flange, model->color);
        }
    }
    resource.source += "\n本机网格 " + std::to_string(meshed) + "/7。";
    applyResource(scene, resource);
    error.clear();
    return true;
}

}  // namespace sim
