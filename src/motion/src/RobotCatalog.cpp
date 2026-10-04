#include "sim/motion/Motion.hpp"

#include "sim/core/Mesh.hpp"

#include <cmath>
#include <iterator>

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
     "加速度 180°/s² 是占位。几何是方盒，不是厂商网格。",
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
     "加速度 180°/s² 是占位。几何是方盒，不是厂商网格。",
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
     "加速度 180°/s² 是占位。几何是方盒，不是厂商网格。",
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
     "法兰在 T 轴前方 80 mm。加速度 180°/s² 是占位。几何是方盒，不是厂商网格。",
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
        if (KinLink* parentLink = findLink(resource, parent)) {
            addBar(scene, scene.find(parentLink->nodeId), std::string(model->name) + "_" + parent, local.translation, model->color);
        }
        parent = row.name;
    }
    if (length(model->flange) > 1) {
        Transform flange;
        flange.translation = model->flange;
        addLink(scene, resource, "flange", parent, flange);
        if (KinLink* wrist = findLink(resource, parent)) {
            addBar(scene, scene.find(wrist->nodeId), std::string(model->name) + "_flange", model->flange, model->color);
        }
    }
    applyResource(scene, resource);
    error.clear();
    return true;
}

}  // namespace sim
