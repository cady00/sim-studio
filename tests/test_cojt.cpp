#include "sim/core/DemoScene.hpp"
#include "sim/io/Cojt.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/motion/Motion.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {

void writeFile(const std::filesystem::path& path, const std::string& text) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    REQUIRE(out.good());
    out << text;
}

}  // namespace

TEST_CASE("cojt parsers read links edges speeds and machine data", "[cojt]") {
    sim::CojtGraph graph;
    std::string error;
    const char* xml = R"(<?xml version="1.0"?>
<AddFlow Nodes="2" Links="1">
  <Node><Text>base</Text><Url>{AAA}</Url></Node>
  <Node><Text>arm</Text></Node>
  <Link Org="0" Dst="1"><Text>J1</Text><Tooltip>axis 0 1 0 limit -10 20 speed 15 couple master 0.25</Tooltip></Link>
</AddFlow>)";
    REQUIRE(sim::parseKinGraphXml(xml, graph, error));
    REQUIRE(graph.links.size() == 2);
    REQUIRE(graph.links[0].name == "base");
    REQUIRE(graph.links[0].guid == "{AAA}");
    REQUIRE(graph.edges.size() == 1);
    REQUIRE(graph.edges[0].from == 0);
    REQUIRE(graph.edges[0].to == 1);
    REQUIRE(graph.edges[0].hasAxis);
    REQUIRE(graph.edges[0].axis.y == Catch::Approx(1));
    REQUIRE(graph.edges[0].lower == Catch::Approx(-10));
    REQUIRE(graph.edges[0].upper == Catch::Approx(20));
    REQUIRE(graph.edges[0].speed == Catch::Approx(15));
    REQUIRE(graph.edges[0].coupledTo == "master");
    REQUIRE(graph.edges[0].coupleRatio == Catch::Approx(0.25));

    sim::MotionParameters motion;
    const char* parameters = R"(
* comment
joint_max_speed j1 1.7453292519943295;
joint_acc_time j1 1.200;
joint_settle j1 0.120;
zone_joints_param medium 0.052 0.052;
zone_cart_param medium 20;
cart_max_lin_speed 2000.0;
)";
    REQUIRE(sim::parseMotionParameters(parameters, motion, error));
    REQUIRE(motion.joints.size() == 1);
    REQUIRE(motion.joints[0].speedDeg == Catch::Approx(100.0).margin(1e-4));
    REQUIRE(motion.joints[0].accTime == Catch::Approx(1.2));
    REQUIRE(motion.joints[0].settle == Catch::Approx(0.12));
    REQUIRE(motion.hasCartSpeed);
    REQUIRE(motion.cartMaxLinSpeed == Catch::Approx(2000));
    REQUIRE(motion.zones.size() >= 1);
    REQUIRE(motion.zones[0].hasCart);
    REQUIRE(motion.zones[0].cartParam == Catch::Approx(20));

    sim::MachineData machine;
    const char* dat = R"(
$ACTIVE_AXIS_MASK='B00111111'
$AXIS_TYPE[1]=3
$AXIS_TYPE[2]=3
$AXIS_TYPE[3]=3
$AXIS_TYPE[4]=3
$AXIS_TYPE[5]=3
$AXIS_TYPE[6]=3
$SOFTN_END[1]=-185.0
$SOFTP_END[1]=185.0
$SOFTN_END[2]=-5.0
$SOFTP_END[2]=10.0
$SOFTN_END[3]=-5.0
$SOFTP_END[3]=10.0
$SOFTN_END[4]=-5.0
$SOFTP_END[4]=10.0
$SOFTN_END[5]=-5.0
$SOFTP_END[5]=10.0
$SOFTN_END[6]=-5.0
$SOFTP_END[6]=10.0
$RAT_MOT_AX[1]={N -1798,D 7}
$VEL_AXIS_MA[1]=5000.0
$AXIS_JERK[1]=10429.6799482
$COUP_COMP[4,5]={N -161,D 12580}
$COUP_COMP[4,6]={N -172,D 22917}
$COUP_COMP[5,6]={N -188,D 21461}
FRAME $TIRORO={X 0.0,Y 0.0,Z 645.0,A 0.0,B 0.0,C 0.0}
FRAME $TFLWP={X 0.0,Y 0.0,Z 215.0,A 0.0,B 0.0,C 0.0}
FRAME $TX3P3={X 1220.0,Y 0.0,Z 115.0,A 0.0,B 90.0,C 0.0}
REAL $LENGTH_A=330.0
REAL $LENGTH_B=1150.0
DECL DHART $DH_4={DHART_A 0.0,DHART_D 0.0,DHART_ALPHA 90.0}
DECL DHART $DH_5={DHART_A 0.0,DHART_D 0.0,DHART_ALPHA -90.0}
CHAR $TRAFONAME[32]
$TRAFONAME[]="#KR210R2700_2 C4 FLR"
DECL KINCLASS $KINCLASS=#STANDARD
)";
    REQUIRE(sim::parseMachineDat(dat, machine, error));
    REQUIRE(machine.ok);
    REQUIRE(machine.tiroro.z == Catch::Approx(645));
    REQUIRE(machine.lengthB == Catch::Approx(1150));
    REQUIRE(machine.tx3p3.z == Catch::Approx(115));
    REQUIRE(machine.dh4Alpha == Catch::Approx(90));
    int used = 0;
    for (const sim::MachineAxis& axis : machine.axes) {
        if (axis.used) {
            ++used;
        }
    }
    REQUIRE(used == 6);
    REQUIRE(machine.axes[0].speed == Catch::Approx(5000.0 * 6.0 * 7.0 / 1798.0).margin(1e-6));
    REQUIRE(machine.axes[3].couples.size() == 2);
    REQUIRE(machine.axes[4].couples.size() == 1);
    REQUIRE(machine.axes[4].couples[0].second == Catch::Approx(-188.0 / 21461.0));

    std::vector<sim::InfoAxisLimit> info;
    REQUIRE(sim::parseComponentInfo("Axis1\t-170\t170\nAxis2 -65 85\n", info, error));
    REQUIRE(info.size() == 2);
    REQUIRE(info[0].lower == Catch::Approx(-170));
    REQUIRE(info[1].upper == Catch::Approx(85));
}

TEST_CASE("machine data builds a numerical chain and a speed-limited move", "[cojt]") {
    const auto folder = std::filesystem::temp_directory_path() / "simstudio-kr-sample.cojt";
    std::filesystem::remove_all(folder);
    writeFile(folder / "MADA" / "KRC" / "R1" / "Mada" / "$machine.dat", R"(
$ACTIVE_AXIS_MASK='B00111111'
$AXIS_TYPE[1]=3
$AXIS_TYPE[2]=3
$AXIS_TYPE[3]=3
$AXIS_TYPE[4]=3
$AXIS_TYPE[5]=3
$AXIS_TYPE[6]=3
$SOFTN_END[1]=-185.0
$SOFTP_END[1]=185.0
$SOFTN_END[2]=-140.0
$SOFTP_END[2]=-5.0
$SOFTN_END[3]=-120.0
$SOFTP_END[3]=168.0
$SOFTN_END[4]=-350.0
$SOFTP_END[4]=350.0
$SOFTN_END[5]=-125.0
$SOFTP_END[5]=125.0
$SOFTN_END[6]=-350.0
$SOFTP_END[6]=350.0
$RAT_MOT_AX[1]={N -1798,D 7}
$VEL_AXIS_MA[1]=5000.0
$COUP_COMP[4,5]={N -161,D 12580}
$COUP_COMP[4,6]={N -172,D 22917}
$COUP_COMP[5,6]={N -188,D 21461}
FRAME $TIRORO={X 0.0,Y 0.0,Z 645.0,A 0.0,B 0.0,C 0.0}
FRAME $TFLWP={X 0.0,Y 0.0,Z 215.0,A 0.0,B 0.0,C 0.0}
FRAME $TX3P3={X 1220.0,Y 0.0,Z 115.0,A 0.0,B 90.0,C 0.0}
REAL $LENGTH_A=330.0
REAL $LENGTH_B=1150.0
DECL DHART $DH_4={DHART_A 0.0,DHART_D 0.0,DHART_ALPHA 90.0}
DECL DHART $DH_5={DHART_A 0.0,DHART_D 0.0,DHART_ALPHA -90.0}
)");
    sim::Scene scene;
    const sim::AssetImportResult imported = sim::importCojt(folder.string(), scene);
    INFO(imported.message);
    REQUIRE(imported.ok);
    sim::KinematicResource* robot = sim::findResource(scene.resources, "simstudio-kr-sample");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->ik == "numerical");
    REQUIRE(robot->joints.size() == 6);
    REQUIRE(robot->wrist.d1 == Catch::Approx(645));
    REQUIRE(robot->wrist.shoulder == Catch::Approx(330));
    REQUIRE(robot->wrist.a2 == Catch::Approx(1150));
    REQUIRE(robot->wrist.a3 == Catch::Approx(1220));
    REQUIRE(robot->wrist.d6 == Catch::Approx(215));
    REQUIRE(robot->joints[0].lower == Catch::Approx(-185));
    REQUIRE(robot->joints[0].upper == Catch::Approx(185));
    REQUIRE(robot->joints[0].maxSpeed == Catch::Approx(5000.0 * 6.0 * 7.0 / 1798.0).margin(1e-4));
    REQUIRE(robot->joints[3].coupledTo.empty());
    REQUIRE(robot->joints[4].coupledTo == "A6");
    REQUIRE(robot->joints[4].coupleRatio == Catch::Approx(-188.0 / 21461.0));
    REQUIRE(robot->source.find("TX3P3.Z") != std::string::npos);
    REQUIRE(robot->source.find("多于一个主动轴") != std::string::npos);
    // A2 soft limit is -140..-5, so a stored 0 is clamped. Open the limits to read the DH zero.
    for (sim::KinJoint& joint : robot->joints) {
        joint.lower = -1000;
        joint.upper = 1000;
    }
    const sim::CartesianPose flange = sim::forwardModel(*robot);
    REQUIRE(flange.position.x == Catch::Approx(1765).margin(1e-2));
    REQUIRE(flange.position.y == Catch::Approx(0).margin(1e-2));
    REQUIRE(flange.position.z == Catch::Approx(1910).margin(1e-2));
    robot->joints[0].lower = -185;
    robot->joints[0].upper = 185;
    robot->joints[1].lower = -140;
    robot->joints[1].upper = -5;
    robot->joints[2].lower = -120;
    robot->joints[2].upper = 168;
    robot->joints[1].position = -90;
    robot->joints[2].position = 90;
    const sim::CartesianPose homePose = sim::forwardModel(*robot);
    REQUIRE(homePose.position.x == Catch::Approx(615).margin(1e-2));
    REQUIRE(homePose.position.y == Catch::Approx(0).margin(1e-2));
    REQUIRE(homePose.position.z == Catch::Approx(760).margin(1e-2));
    robot->joints[1].position = -90;
    robot->joints[2].position = 90;

    sim::RobotPath path;
    path.name = "sweep";
    path.resource = robot->name;
    sim::PathPoint home;
    home.move = sim::PathMove::Ptp;
    home.joints = {0, -90, 90, 0, 0, 0};
    sim::PathPoint away = home;
    away.joints[0] = robot->joints[0].maxSpeed;
    path.points = {home, away};
    const sim::MotionPlan plan = sim::planRobotPath(scene, *robot, path);
    const std::string planWarning = plan.warnings.empty() ? std::string("no warnings") : plan.warnings.front();
    INFO(planWarning);
    REQUIRE(plan.ok);
    REQUIRE(plan.seconds == Catch::Approx(1.0).margin(0.05));
    std::filesystem::remove_all(folder);
}

TEST_CASE("a graph edge without machine data keeps the tooltip axis", "[cojt]") {
    const auto folder = std::filesystem::temp_directory_path() / "simstudio-edge.cojt";
    std::filesystem::remove_all(folder);
    writeFile(folder / "kin_graph.xml", R"(<?xml version="1.0"?>
<AddFlow>
  <Node><Text>base</Text></Node>
  <Node><Text>slide</Text></Node>
  <Link Org="0" Dst="1"><Text>E1</Text><Tooltip>prismatic axis 1 0 0 limit 0 1000 speed 250</Tooltip></Link>
</AddFlow>)");
    sim::Scene scene;
    const sim::AssetImportResult imported = sim::importCojt(folder.string(), scene);
    INFO(imported.message);
    REQUIRE(imported.ok);
    sim::KinematicResource* robot = sim::findResource(scene.resources, "simstudio-edge");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->joints.size() == 1);
    REQUIRE(robot->joints[0].motion == sim::JointMotion::Prismatic);
    REQUIRE(robot->joints[0].axis.x == Catch::Approx(1));
    REQUIRE(robot->joints[0].upper == Catch::Approx(1000));
    REQUIRE(robot->joints[0].maxSpeed == Catch::Approx(250));
    std::filesystem::remove_all(folder);
}

TEST_CASE("private cojt components calibrate kinematics", "[cojt]") {
    const auto root = std::filesystem::path(SIM_SOURCE_DIR) / "testdata" / "local" / "cojt";
    if (!std::filesystem::exists(root / "kr210_r2700_2.cojt" / "kr210_r2700_2.jt")) {
        SKIP("testdata/local/cojt is not present");
    }
    sim::Scene kr210;
    const sim::AssetImportResult kr210Result = sim::importCojt((root / "kr210_r2700_2.cojt").string(), kr210);
    INFO(kr210Result.message);
    REQUIRE(kr210Result.ok);
    REQUIRE(kr210Result.triangleCount == 387479);
    sim::KinematicResource* robot = sim::findResource(kr210.resources, "kr210_r2700_2");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->joints.size() == 6);
    REQUIRE(robot->wrist.a2 == Catch::Approx(1150));
    REQUIRE(robot->wrist.d1 == Catch::Approx(645));
    REQUIRE(robot->wrist.shoulder == Catch::Approx(330));
    REQUIRE(robot->wrist.a3 == Catch::Approx(1220));
    REQUIRE(robot->wrist.d6 == Catch::Approx(215));
    REQUIRE(robot->joints[3].coupledTo.empty());
    REQUIRE(robot->source.find("TX3P3.Z") != std::string::npos);
    REQUIRE(robot->joints[0].lower == Catch::Approx(-185));
    REQUIRE(robot->joints[0].upper == Catch::Approx(185));
    REQUIRE(robot->joints[1].lower == Catch::Approx(-140));
    REQUIRE(robot->joints[1].upper == Catch::Approx(-5));
    REQUIRE(robot->joints[0].maxSpeed == Catch::Approx(5000.0 * 6.0 * 7.0 / 1798.0).margin(1e-3));
    REQUIRE(robot->joints[4].coupledTo == "A6");
    REQUIRE(robot->joints[0].axis.z == Catch::Approx(1).margin(1e-6));
    REQUIRE(robot->joints[1].axis.y == Catch::Approx(1).margin(1e-6));
    REQUIRE(robot->joints[2].axis.y == Catch::Approx(1).margin(1e-6));
    REQUIRE(robot->joints[3].axis.x == Catch::Approx(1).margin(1e-6));
    REQUIRE(robot->source.find("没有当成关节轴") != std::string::npos);
    std::cerr << "kr210 " << kr210Result.message << "\n";
    std::cerr << "kr210 matched speeds";
    for (const sim::KinJoint& joint : robot->joints) {
        sim::Mat4 parent = sim::Mat4::identity();
        sim::linkWorld(*robot, joint.parentLink, parent);
        const sim::Vec3 worldAxis = sim::transformVector(parent, joint.axis);
        std::cerr << " " << joint.name << " " << joint.lower << ".." << joint.upper << " v " << joint.maxSpeed << " axis "
                  << joint.axis.x << "," << joint.axis.y << "," << joint.axis.z << " world " << worldAxis.x << ","
                  << worldAxis.y << "," << worldAxis.z;
    }
    std::cerr << "\n";

    sim::Scene trackScene;
    const sim::AssetImportResult trackResult = sim::importCojt((root / "irbt6004_0_d_10m_01.cojt").string(), trackScene);
    INFO(trackResult.message);
    REQUIRE(trackResult.ok);
    REQUIRE(trackResult.triangleCount == 30236);
    sim::KinematicResource* track = sim::findResource(trackScene.resources, "irbt6004_0_d_10m_01");
    REQUIRE(track != nullptr);
    REQUIRE(track->joints.size() == 1);
    REQUIRE(track->joints[0].externalName == "E1");
    REQUIRE(track->joints[0].externalKind == "track");
    REQUIRE(track->joints[0].motion == sim::JointMotion::Prismatic);
    REQUIRE(track->joints[0].upper == Catch::Approx(11568));
    REQUIRE(track->source.find("mesh bounds") != std::string::npos);
    sim::Scene mounted;
    const sim::AssetImportResult mountedTrack =
        sim::importCojt((root / "irbt6004_0_d_10m_01.cojt").string(), mounted);
    REQUIRE(mountedTrack.ok);
    sim::KinematicResource arm;
    arm.name = "arm";
    arm.ik = "spherical_wrist";
    sim::addLink(mounted, arm, "base", {}, {});
    mounted.resources.push_back(arm);
    sim::KinematicResource* armResource = sim::findResource(mounted.resources, "arm");
    REQUIRE(armResource != nullptr);
    std::string attachError;
    REQUIRE(sim::attachExternalAxes(mounted, *armResource, "irbt6004_0_d_10m_01", "carriage", "track", attachError));
    REQUIRE(sim::hasExternalAxis(*armResource));
    const sim::KinJoint* e1 = nullptr;
    for (const sim::KinJoint& joint : armResource->joints) {
        if (joint.externalName == "E1") {
            e1 = &joint;
        }
    }
    REQUIRE(e1 != nullptr);
    REQUIRE(e1->externalKind == "track");

    sim::Scene fanuc;
    const sim::AssetImportResult fanucResult = sim::importCojt((root / "r2000ib210f.cojt").string(), fanuc);
    INFO(fanucResult.message);
    REQUIRE(fanucResult.ok);
    sim::KinematicResource* r2000 = sim::findResource(fanuc.resources, "r2000ib210f");
    REQUIRE(r2000 != nullptr);
    REQUIRE(r2000->joints.size() == 6);
    REQUIRE(r2000->joints[0].maxSpeed == Catch::Approx(100.0).margin(0.02));
    REQUIRE(r2000->joints[0].maxAccel == Catch::Approx(100.0 / 1.2).margin(0.05));
    REQUIRE(r2000->joints[0].accelSource == "motionparameters");
    REQUIRE(r2000->joints[5].maxSpeed == Catch::Approx(210.0).margin(0.05));
    REQUIRE(r2000->source.find("+Z") != std::string::npos);

    sim::Scene abb;
    const sim::AssetImportResult abbResult =
        sim::importCojt((root / "IRB6700_235-265_IRC5_Grey_Rev1.cojt").string(), abb);
    INFO(abbResult.message);
    REQUIRE(abbResult.ok);
    sim::KinematicResource* irb = sim::findResource(abb.resources, "IRB6700_235-265_IRC5_Grey_Rev1");
    REQUIRE(irb != nullptr);
    REQUIRE(irb->joints.size() == 6);
    REQUIRE(irb->joints[0].lower == Catch::Approx(-170));
    REQUIRE(irb->joints[0].upper == Catch::Approx(170));
    REQUIRE(irb->joints[1].lower == Catch::Approx(-65));
    REQUIRE(irb->joints[1].upper == Catch::Approx(85));
    REQUIRE(irb->joints[0].maxSpeed == 0.0);

    sim::Scene kr420;
    const sim::AssetImportResult kr420Result = sim::importCojt((root / "kr420r3080_4.cojt").string(), kr420);
    INFO(kr420Result.message);
    REQUIRE(kr420Result.ok);
    REQUIRE(kr420Result.triangleCount == 418625);
    sim::KinematicResource* heavy = sim::findResource(kr420.resources, "kr420r3080_4");
    REQUIRE(heavy != nullptr);
    REQUIRE(heavy->joints.empty());
    REQUIRE(heavy->wrist.a2 == 0.0);
    REQUIRE(heavy->source.find("zone medium") != std::string::npos);
    std::cerr << "kr420 cojt joints " << heavy->joints.size() << " source-has-zone yes\n";

    const auto step = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/KR420_R3080_KR280_R3080.stp";
    if (!std::filesystem::exists(step)) {
        SKIP("local KR 420 STEP is not in the tree");
    }
    sim::Scene stepScene;
    sim::StepImporter importer;
    const sim::ImportResult stepImported = importer.importFile(step.string(), stepScene);
    REQUIRE(stepImported.ok);
    const sim::SuggestResult suggestion = sim::suggestLinkGroups(stepScene);
    std::cerr << "step fitted d1 " << suggestion.fitted.d1 << " shoulder " << suggestion.fitted.shoulder << " a2 "
              << suggestion.fitted.a2 << " a3 " << suggestion.fitted.a3 << " d6 " << suggestion.fitted.d6 << "\n";
    REQUIRE(suggestion.links.size() == 7);
    std::string suggestError;
    REQUIRE(sim::applyLinkSuggestions(stepScene, "kr420", suggestion, sim::kr420DatasheetLimits(), suggestError));
    sim::addKr420Demonstration(stepScene);
    sim::KinematicResource* stepRobot = sim::findResource(stepScene.resources, "kr420");
    REQUIRE(stepRobot != nullptr);
    sim::RobotPath* cycle = nullptr;
    for (sim::RobotPath& path : stepScene.robotPaths) {
        if (path.name == "cycle") {
            cycle = &path;
        }
    }
    REQUIRE(cycle != nullptr);
    const auto hidden = sim::hidePartsOutsideProximity(stepScene);
    const sim::MotionPlan before = sim::planRobotPath(stepScene, *stepRobot, *cycle);
    REQUIRE(before.ok);
    REQUIRE(before.seconds == Catch::Approx(192.0 / 60.0).margin(1e-6));
    sim::MotionParameters zones;
    std::string zoneError;
    {
        std::ifstream in(root / "kr420r3080_4.cojt" / "motionparameters.e");
        std::ostringstream buffer;
        buffer << in.rdbuf();
        REQUIRE(sim::parseMotionParameters(buffer.str(), zones, zoneError));
    }
    double medium = 20;
    for (const sim::MotionZone& zone : zones.zones) {
        if (zone.name == "medium" && zone.hasCart) {
            medium = zone.cartParam;
        }
    }
    for (sim::PathPoint& point : cycle->points) {
        point.zone = medium;
    }
    const sim::MotionPlan after = sim::planRobotPath(stepScene, *stepRobot, *cycle);
    std::cerr << "kr420 cycle datasheet " << before.seconds << " cojt-zone " << after.seconds << " medium " << medium
              << "\n";
    REQUIRE(after.ok);
    REQUIRE(after.seconds == Catch::Approx(192.0 / 60.0).margin(1e-6));
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }
    (void)heavy;
}

TEST_CASE("commission scene mounts the local KR420 cojt mesh on A1", "[cojt]") {
    const auto folder = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/cojt/kr420r3080_4.cojt";
    if (!std::filesystem::is_directory(folder)) {
        SKIP("testdata/local/cojt/kr420r3080_4.cojt is not present");
    }
    sim::Scene scene;
    sim::buildCommissionLine(scene);
    sim::attachCommissionRobot(scene);
    const sim::KinematicResource* robot = sim::findResource(scene.resources, "r1");
    REQUIRE(robot != nullptr);
    std::string armId;
    for (const sim::KinLink& link : robot->links) {
        if (link.name == "arm") {
            armId = link.nodeId;
        }
    }
    REQUIRE_FALSE(armId.empty());
    std::string note;
    const int triangles = sim::attachCojtVisual(scene, folder.string(), armId, {560, 260, 0}, note);
    INFO(note);
    REQUIRE(triangles > 100000);
    bool onArm = false;
    std::vector<std::shared_ptr<sim::SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        const auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->meshId.empty()) {
            continue;
        }
        for (auto parent = node->parent.lock(); parent; parent = parent->parent.lock()) {
            if (parent->id == armId) {
                onArm = true;
            }
        }
    }
    REQUIRE(onArm);
}
