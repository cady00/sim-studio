#include "sim/ai/Commands.hpp"
#include "sim/collision/Query.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/motion/Scope.hpp"
#include "sim/motion/WeldPlan.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

class EnvGuard {
public:
    EnvGuard(const char* key, const std::string& value) : key_(key) {
        const char* previous = std::getenv(key);
        if (previous != nullptr) {
            had_ = true;
            previous_ = previous;
        }
        setenv(key_.c_str(), value.c_str(), 1);
    }
    ~EnvGuard() {
        if (had_) {
            setenv(key_.c_str(), previous_.c_str(), 1);
        } else {
            unsetenv(key_.c_str());
        }
    }
    EnvGuard(const EnvGuard&) = delete;
    EnvGuard& operator=(const EnvGuard&) = delete;

private:
    std::string key_;
    std::string previous_;
    bool had_ = false;
};

std::filesystem::path emptyMeshDir() {
    const auto dir = std::filesystem::temp_directory_path() / "simstudio-catalog-empty";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

sim::KinematicResource scaraResource() {
    sim::KinematicResource resource;
    resource.name = "scara";
    resource.ik = "scara";
    resource.scara.l1 = 350;
    resource.scara.l2 = 250;
    for (const char* name : {"A1", "A2", "A3", "A4"}) {
        sim::KinJoint joint;
        joint.name = name;
        joint.limited = true;
        resource.joints.push_back(joint);
    }
    resource.joints[0].lower = -132;
    resource.joints[0].upper = 132;
    resource.joints[1].lower = -145;
    resource.joints[1].upper = 145;
    resource.joints[2].motion = sim::JointMotion::Prismatic;
    resource.joints[2].lower = -200;
    resource.joints[2].upper = 0;
    resource.joints[3].lower = -355;
    resource.joints[3].upper = 355;
    return resource;
}

sim::KinematicResource wristResource() {
    sim::KinematicResource resource;
    resource.name = "arm6";
    resource.ik = "spherical_wrist";
    resource.wrist.d1 = 300;
    resource.wrist.a2 = 400;
    resource.wrist.a3 = 320;
    resource.wrist.d6 = 80;
    for (int i = 0; i < 6; ++i) {
        sim::KinJoint joint;
        joint.name = "A" + std::to_string(i + 1);
        joint.lower = -170;
        joint.upper = 170;
        resource.joints.push_back(joint);
    }
    return resource;
}

double poseGap(const sim::CartesianPose& a, const sim::CartesianPose& b) {
    const sim::Quat delta = (a.rotation.conjugate() * b.rotation).normalized();
    const double w = std::clamp(std::abs(delta.w), 0.0, 1.0);
    return sim::length(a.position - b.position) + sim::rad2deg(2.0 * std::acos(w));
}

}  // namespace

TEST_CASE("trapezoid and s-curve profiles finish at the target") {
    const double trap = sim::profileDuration(sim::ProfileKind::Trapezoid, 0, 90, 120, 480);
    // 120°/s and 480°/s²: 0.25 s accel, 0.5 s cruise, 0.25 s decel.
    REQUIRE(trap == Catch::Approx(1.0).margin(1e-12));
    bool done = false;
    REQUIRE(sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, 0, &done) == 0);
    REQUIRE_FALSE(done);
    const double end = sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, trap, &done);
    REQUIRE(done);
    REQUIRE(std::abs(end - 90) < 1e-6);
    const double mid = sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, trap * 0.5, &done);
    REQUIRE(mid == Catch::Approx(45.0).margin(1e-9));

    const double curve = sim::profileDuration(sim::ProfileKind::SCurve, 10, -40, 80, 200);
    const double start = sim::profileSample(sim::ProfileKind::SCurve, 10, -40, 80, 200, 0, &done);
    const double finish = sim::profileSample(sim::ProfileKind::SCurve, 10, -40, 80, 200, curve, &done);
    REQUIRE(std::abs(start - 10) < 1e-9);
    REQUIRE(std::abs(finish + 40) < 1e-6);
    const double early = sim::profileSample(sim::ProfileKind::SCurve, 10, -40, 80, 200, curve * 0.02, nullptr);
    REQUIRE(std::abs(early - 10) < std::abs(-40 - 10) * 0.05);
}

TEST_CASE("scara and spherical wrist forward kinematics round trip") {
    sim::KinematicResource scara = scaraResource();
    const std::vector<double> scaraJoints{20, -35, -80, 15};
    sim::setJointVector(scara, scaraJoints);
    const sim::CartesianPose scaraPose = sim::forwardModel(scara);
    const sim::IkResult scaraIk = sim::inverseModel(scara, scaraPose, scaraJoints);
    REQUIRE(scaraIk.ok);
    REQUIRE_FALSE(scaraIk.approximated);
    sim::setJointVector(scara, scaraIk.joints);
    REQUIRE(poseGap(scaraPose, sim::forwardModel(scara)) < 1e-3);

    sim::KinematicResource arm = wristResource();
    const std::vector<double> armJoints{15, 25, -40, 20, 30, -10};
    sim::setJointVector(arm, armJoints);
    const sim::CartesianPose armPose = sim::forwardModel(arm);
    const sim::IkResult armIk = sim::inverseModel(arm, armPose, armJoints);
    REQUIRE(armIk.ok);
    REQUIRE_FALSE(armIk.approximated);
    sim::setJointVector(arm, armIk.joints);
    REQUIRE(poseGap(armPose, sim::forwardModel(arm)) < 1e-2);
}

TEST_CASE("inverse kinematics reports a joint limit") {
    sim::KinematicResource scara = scaraResource();
    sim::CartesianPose far = sim::forwardModel(scara);
    far.position = {0, 0, -400};
    const sim::IkResult solved = sim::inverseModel(scara, far, {});
    REQUIRE_FALSE(solved.ok);
    const bool explained = solved.warning.find("limit") != std::string::npos || solved.warning.find("unreachable") != std::string::npos;
    REQUIRE(explained);
}

TEST_CASE("coupled joints follow the master and a stacker crane reaches a pose") {
    sim::Scene scene;
    auto& resource = sim::ensureResource(scene, "stacker");
    sim::addLink(scene, resource, "base", "", {});
    sim::Transform travelLocal;
    sim::KinLink& travel = sim::addLink(scene, resource, "travel", "base", travelLocal);
    sim::Transform liftLocal;
    sim::KinLink& lift = sim::addLink(scene, resource, "lift", "travel", liftLocal);
    sim::Transform forkLocal;
    sim::addLink(scene, resource, "fork", "lift", forkLocal);
    sim::addLink(scene, resource, "roller", "fork", {});
    sim::KinJoint& x = sim::addJoint(resource, scene, "travel", "base", "travel", sim::JointMotion::Prismatic, {1, 0, 0}, {});
    x.upper = 4000;
    x.maxSpeed = 1000;
    sim::KinJoint& z = sim::addJoint(resource, scene, "lift", "travel", "lift", sim::JointMotion::Prismatic, {0, 0, 1}, {});
    z.upper = 2000;
    sim::KinJoint& y = sim::addJoint(resource, scene, "fork", "lift", "fork", sim::JointMotion::Prismatic, {0, 1, 0}, {});
    y.upper = 800;
    sim::KinJoint& roller = sim::addJoint(resource, scene, "roller", "fork", "roller", sim::JointMotion::Revolute, {0, 0, 1}, {});
    roller.coupledTo = "travel";
    roller.coupleRatio = 0.05;
    roller.limited = false;
    resource.joints[0].position = 1000;
    resource.joints[1].position = 500;
    resource.joints[2].position = 200;
    sim::applyResource(scene, resource);
    auto fork = scene.find(resource.links[3].nodeId);
    REQUIRE(fork);
    REQUIRE(std::abs(fork->worldMatrix()(3, 0) - 1000) < 1e-6);
    REQUIRE(std::abs(fork->worldMatrix()(3, 1) - 200) < 1e-6);
    REQUIRE(std::abs(fork->worldMatrix()(3, 2) - 500) < 1e-6);
    REQUIRE(std::abs(sim::jointCommand(resource, roller) - 50) < 1e-6);
    (void)travel;
    (void)lift;
}

TEST_CASE("logic block, save, and repeat are deterministic") {
    sim::Scene scene;
    sim::buildTurntableLiftDemo(scene);
    sim::LogicMachine first;
    sim::LogicMachine second;
    for (int i = 0; i < 400; ++i) {
        first.step(scene, sim::Simulation::kFixedDt);
    }
    REQUIRE(scene.findSignal("done") != nullptr);
    REQUIRE(scene.findSignal("done")->flag);
    const double spin = sim::findJoint(*sim::findResource(scene.resources, "turntable_lift"), "spin")->position;
    const double lift = sim::findJoint(*sim::findResource(scene.resources, "turntable_lift"), "lift")->position;
    REQUIRE(std::abs(spin) < 1e-3);
    REQUIRE(std::abs(lift) < 1e-3);

    sim::Scene again;
    sim::buildTurntableLiftDemo(again);
    for (int i = 0; i < 400; ++i) {
        second.step(again, sim::Simulation::kFixedDt);
    }
    REQUIRE(sim::findJoint(*sim::findResource(again.resources, "turntable_lift"), "spin")->position == spin);
    REQUIRE(sim::findJoint(*sim::findResource(again.resources, "turntable_lift"), "lift")->position == lift);

    const auto path = std::filesystem::temp_directory_path() / "simstudio-phase3.simproj";
    REQUIRE(sim::ProjectStore::save(scene, path.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(path.string(), loaded).ok);
    REQUIRE(loaded.resources.size() == 1);
    REQUIRE(loaded.logicBlocks.size() == 1);
    REQUIRE(loaded.logicBlocks[0].python.find("on_step") != std::string::npos);
    REQUIRE(sim::findPose(loaded.resources[0], "work") != nullptr);

    const auto library = std::filesystem::temp_directory_path() / "turntable.simlib";
    std::string error;
    REQUIRE(sim::saveLibrary(scene, "turntable_lift", library.string(), error));
    sim::Scene empty;
    REQUIRE(sim::loadLibrary(empty, library.string(), error));
    REQUIRE(empty.resources.size() == 1);
    REQUIRE(empty.logicBlocks.size() == 1);
}

TEST_CASE("robot path reports cycle time and a collision pair") {
    sim::Scene scene;
    sim::buildScaraDemo(scene);
    auto* resource = sim::findResource(scene.resources, "scara");
    REQUIRE(resource != nullptr);
    const sim::RobotPath& path = scene.robotPaths.at(0);
    const sim::MotionPlan fine = sim::planRobotPath(scene, *resource, path);
    REQUIRE(fine.samples.size() == 319);
    REQUIRE(fine.segmentSeconds.size() == 4);
    REQUIRE(fine.segmentSeconds[0] == Catch::Approx(58.0 / 60.0).margin(1e-9));
    REQUIRE(fine.segmentSeconds[1] == Catch::Approx(63.0 / 60.0).margin(1e-9));
    REQUIRE(fine.segmentSeconds[2] == Catch::Approx(51.0 / 60.0).margin(1e-9));
    REQUIRE(fine.segmentSeconds[3] == Catch::Approx(154.0 / 60.0).margin(1e-9));
    REQUIRE(fine.seconds == Catch::Approx(318.0 / 60.0).margin(1e-9));
    sim::RobotPath zoned = path;
    zoned.points[0].zone = 40;
    const sim::MotionPlan blended = sim::planRobotPath(scene, *resource, zoned);
    REQUIRE(blended.seconds <= fine.seconds + 1e-6);

    sim::Scene blocked;
    sim::buildScaraDemo(blocked);
    auto obstacle = blocked.makeNode(sim::NodeKind::Part, "Obstacle");
    obstacle->meshId = blocked.addMesh(sim::makeBoxMesh({40, 40, 40}));
    obstacle->local.translation = {500, 0, 80};
    blocked.addChild(blocked.root(), obstacle);
    auto* arm = sim::findResource(blocked.resources, "scara");
    sim::setJointVector(*arm, {0, 0, -100, 0});
    sim::applyResource(blocked, *arm);
    auto forearm = blocked.findByName("Forearm");
    REQUIRE(forearm);
    UNSCOPED_INFO("forearm " << forearm->worldMatrix()(3, 0) << " " << forearm->worldMatrix()(3, 1) << " " << forearm->worldMatrix()(3, 2));
    UNSCOPED_INFO("obstacle " << obstacle->worldMatrix()(3, 0) << " " << obstacle->worldMatrix()(3, 1) << " " << obstacle->worldMatrix()(3, 2));
    const auto hits = sim::collidingPairs(blocked, *arm);
    REQUIRE(hits.size() == 1);
    REQUIRE(hits.front().first == "Forearm");
    REQUIRE(hits.front().second == "Obstacle");
}

TEST_CASE("command layer covers kinematics logic and planning") {
    sim::Scene scene;
    sim::buildTurntableLiftDemo(scene);
    sim::CommandExecutor commands(scene, nullptr, nullptr);
    REQUIRE(commands.execute({{"op", "kin_list"}}).ok);
    REQUIRE(commands.execute({{"op", "kin_set_joint"}, {"resource", "turntable_lift"}, {"joint", "spin"}, {"value", 15}}).ok);
    REQUIRE(std::abs(sim::findJoint(*sim::findResource(scene.resources, "turntable_lift"), "spin")->position - 15) < 1e-6);
    const auto moved = commands.execute({{"op", "kin_move_pose"}, {"resource", "turntable_lift"}, {"pose", "work"}, {"profile", "scurve"}});
    REQUIRE(moved.ok);
    sim::buildScaraDemo(scene);
    REQUIRE(commands.execute({{"op", "kin_fk"}, {"resource", "scara"}}).ok);
    const auto plan = commands.execute({{"op", "robot_plan"}, {"path", "pick_place"}});
    REQUIRE(plan.ok);
    REQUIRE(plan.value.at("seconds").get<double>() == Catch::Approx(318.0 / 60.0).margin(1e-9));
    auto tool = scene.makeNode(sim::NodeKind::Part, "Gun");
    scene.addChild(scene.root(), tool);
    REQUIRE(commands.execute({{"op", "attach_tool"}, {"resource", "scara"}, {"name", "Gun"}}).ok);
    REQUIRE(tool->parent.lock()->name == "tool");
    REQUIRE(commands.execute({{"op", "detach_tool"}, {"name", "Gun"}}).ok);
}

TEST_CASE("flat solids can be grouped and a cylinder face supplies an axis") {
    sim::Scene scene;
    auto add = [&](const char* name, sim::Vec3 center, double half, sim::Vec3 axisPoint, sim::Vec3 axisDir, double radius) {
        auto node = scene.makeNode(sim::NodeKind::Part, name);
        node->local.translation = center;
        node->meshId = scene.addMesh(sim::makeBoxMesh({half, half, half}));
        if (radius > 0) {
            for (int copy = 0; copy < 4; ++copy) {
                sim::FaceAxis axis;
                axis.origin = axisPoint - center;
                axis.direction = axisDir;
                axis.radius = radius;
                node->faceAxes.push_back(axis);
            }
        }
        scene.addChild(scene.root(), node);
        return node;
    };
    auto foot = add("foot", {0, 0, 40}, 50, {}, {}, 0);
    auto column = add("column", {0, 0, 600}, 40, {0, 0, 600}, {0, 0, 1}, 80);
    add("shoulder", {500, 0, 1100}, 40, {500, 0, 1100}, {0, 1, 0}, 70);
    add("elbow", {500, 0, 2400}, 40, {500, 0, 2400}, {0, 1, 0}, 60);
    add("forearm", {1100, 0, 2400}, 40, {1100, 0, 2400}, {1, 0, 0}, 55);
    add("wrist", {1900, 0, 2400}, 40, {1900, 0, 2400}, {0, 1, 0}, 50);
    add("flange", {2300, 0, 2750}, 40, {2300, 0, 2750}, {1, 0, 0}, 45);
    auto fastener = add("screw", {30, 0, 40}, 8, {}, {}, 0);
    scene.setSelection({column->id, foot->id});
    REQUIRE(scene.selection().size() == 2);
    scene.toggleSelected(foot->id);
    REQUIRE(scene.selection().size() == 1);
    REQUIRE(scene.selectedId() == column->id);

    sim::Vec3 origin;
    sim::Vec3 direction;
    double radius = 0;
    REQUIRE(sim::axisFromCylindricalFace(scene, column->id, {80, 0, 600}, {1, 0, 0}, origin, direction, radius));
    REQUIRE(std::abs(radius - 80) < 1e-6);
    REQUIRE(std::abs(std::abs(direction.z) - 1) < 1e-6);
    REQUIRE_FALSE(sim::axisFromCylindricalFace(scene, column->id, {80, 0, 600}, {0, 0, 1}, origin, direction, radius));

    const sim::SuggestResult suggestion = sim::suggestLinkGroups(scene);
    REQUIRE(suggestion.links.size() == 7);
    REQUIRE(suggestion.links.front().name == "base");
    for (int i = 1; i <= 6; ++i) {
        REQUIRE(suggestion.links[static_cast<std::size_t>(i)].name == "A" + std::to_string(i));
        REQUIRE(suggestion.links[static_cast<std::size_t>(i)].hasJoint);
    }
    REQUIRE(suggestion.a1ToA2Mm > 400);
    std::string error;
    REQUIRE(sim::applyLinkSuggestions(scene, "arm", suggestion, sim::kr420DatasheetLimits(), error));
    REQUIRE(error.empty());
    sim::KinematicResource* resource = sim::findResource(scene.resources, "arm");
    REQUIRE(resource != nullptr);
    REQUIRE(resource->ik == "numerical");
    REQUIRE(resource->joints.size() == 6);
    REQUIRE(resource->joints[0].lower == -185);
    REQUIRE(resource->joints[1].upper == 20);
    REQUIRE(resource->joints[5].maxSpeed == 130);
    REQUIRE(resource->joints[0].accelSource == "velocity");
    REQUIRE(sim::jointAccelLimit(*resource, resource->joints[0]).source == "velocity");
    const bool footInBase = std::find(suggestion.links[0].partIds.begin(), suggestion.links[0].partIds.end(), foot->id) !=
                            suggestion.links[0].partIds.end();
    const bool screwFollows = std::find(suggestion.links[0].partIds.begin(), suggestion.links[0].partIds.end(), fastener->id) !=
                              suggestion.links[0].partIds.end();
    REQUIRE(footInBase);
    REQUIRE(screwFollows);

    const std::vector<double> nudge{18, -12, 15, 25, -10, 20};
    sim::setJointVector(*resource, nudge);
    const sim::CartesianPose away = sim::forwardModel(*resource);
    std::vector<double> seed = nudge;
    for (double& value : seed) {
        value *= 0.5;
    }
    const sim::IkResult numeric = sim::inverseModel(*resource, away, seed);
    REQUIRE(numeric.ok);
    REQUIRE(numeric.approximated);
    sim::setJointVector(*resource, numeric.joints);
    REQUIRE(sim::length(sim::forwardModel(*resource).position - away.position) < 2.0);

    sim::KinematicResource analytic = *resource;
    analytic.ik = "spherical_wrist";
    const std::vector<double> modest{20, -15, 25, 30, -20, 40};
    sim::setJointVector(analytic, modest);
    const sim::CartesianPose analyticPose = sim::forwardModel(analytic);
    const sim::IkResult analyticIk = sim::inverseModel(analytic, analyticPose, modest);
    REQUIRE(analyticIk.ok);
    REQUIRE_FALSE(analyticIk.approximated);
    sim::setJointVector(analytic, analyticIk.joints);
    REQUIRE(sim::length(sim::forwardModel(analytic).position - analyticPose.position) < 1e-2);
}

TEST_CASE("kr420 flat step becomes a six axis chain") {
    const std::filesystem::path step = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/KR420_R3080_KR280_R3080.stp";
    if (!std::filesystem::exists(step)) {
        SKIP("local KR 420 STEP is not in the tree");
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult imported = importer.importFile(step.string(), scene);
    REQUIRE(imported.ok);
    const sim::SuggestResult suggestion = sim::suggestLinkGroups(scene);
    std::cerr << "kr420 note: " << suggestion.note << "\n";
    std::cerr << "fitted d1 " << suggestion.fitted.d1 << " shoulder " << suggestion.fitted.shoulder << " a2 "
              << suggestion.fitted.a2 << " a3 " << suggestion.fitted.a3 << " d6 " << suggestion.fitted.d6
              << " a1ToA2 " << suggestion.a1ToA2Mm << "\n";
    for (const sim::LinkSuggestion& link : suggestion.links) {
        std::cerr << link.name << " parts " << link.partIds.size() << " cylinders " << link.axisCount << " r "
                  << link.axisRadius << " origin " << link.jointOrigin.x << " " << link.jointOrigin.y << " "
                  << link.jointOrigin.z << " axis " << link.jointAxis.x << " " << link.jointAxis.y << " "
                  << link.jointAxis.z << "\n";
    }
    REQUIRE(suggestion.links.size() == 7);
    std::string error;
    REQUIRE(sim::applyLinkSuggestions(scene, "kr420", suggestion, sim::kr420DatasheetLimits(), error));
    sim::KinematicResource* resource = sim::findResource(scene.resources, "kr420");
    REQUIRE(resource != nullptr);
    REQUIRE(resource->joints.size() == 6);
    const std::vector<double> nudge{18, -12, 15, 25, -10, 20};
    sim::setJointVector(*resource, nudge);
    const sim::CartesianPose away = sim::forwardModel(*resource);
    std::vector<double> seed = nudge;
    for (double& value : seed) {
        value *= 0.5;
    }
    const sim::IkResult numeric = sim::inverseModel(*resource, away, seed);
    std::cerr << "numerical ok " << numeric.ok << " warning " << numeric.warning << "\n";
    REQUIRE(numeric.ok);
    sim::setJointVector(*resource, numeric.joints);
    const double numericGap = sim::length(sim::forwardModel(*resource).position - away.position);
    std::cerr << "numerical gap mm " << numericGap << "\n";
    REQUIRE(numericGap < 5.0);
    sim::setJointVector(*resource, std::vector<double>(6, 0));

    sim::KinematicResource analytic = *resource;
    analytic.ik = "spherical_wrist";
    const std::vector<double> modest{20, -15, 25, 30, -20, 40};
    sim::setJointVector(analytic, modest);
    const sim::CartesianPose analyticPose = sim::forwardModel(analytic);
    const sim::IkResult analyticIk = sim::inverseModel(analytic, analyticPose, modest);
    std::cerr << "analytic ok " << analyticIk.ok << " warning " << analyticIk.warning << "\n";
    REQUIRE(analyticIk.ok);
    REQUIRE_FALSE(analyticIk.approximated);

    sim::addKr420Demonstration(scene);
    sim::RobotPath* path = nullptr;
    for (sim::RobotPath& candidate : scene.robotPaths) {
        if (candidate.name == "cycle") {
            path = &candidate;
        }
    }
    REQUIRE(path != nullptr);
    const auto hidden = sim::hidePartsOutsideProximity(scene);
    const sim::MotionPlan plan = sim::planRobotPath(scene, *resource, *path);
    std::cerr << "cycle seconds " << plan.seconds << " samples " << plan.samples.size() << " collisions "
              << plan.collisions.size() << " sweep " << plan.sweepHit << "\n";
    REQUIRE(plan.segmentSeconds.size() == 3);
    REQUIRE(plan.segmentSeconds[0] == Catch::Approx(1.0 / 60.0).margin(1e-9));
    REQUIRE(plan.segmentSeconds[1] == Catch::Approx(17.0 / 60.0).margin(1e-9));
    REQUIRE(plan.segmentSeconds[2] == Catch::Approx(178.0 / 60.0).margin(1e-9));
    const double raw = plan.segmentSeconds[0] + plan.segmentSeconds[1] + plan.segmentSeconds[2];
    // Unblended chunks are 196 frames. Zone blend drops 4 frames, leaving the
    // 193 samples whose span is the 3.200 s cycle.
    REQUIRE(raw == Catch::Approx(196.0 / 60.0).margin(1e-9));
    REQUIRE(raw - plan.seconds == Catch::Approx(4.0 / 60.0).margin(1e-9));
    for (const std::string& warning : plan.warnings) {
        std::cerr << "plan warning " << warning << "\n";
        REQUIRE(warning.find("unreachable") == std::string::npos);
    }
    REQUIRE(plan.ok);
    REQUIRE(plan.warnings.empty());
    REQUIRE(plan.collisions.size() == 1);
    REQUIRE(plan.sweepHit);
    REQUIRE(plan.samples.size() == 193);
    REQUIRE(plan.dt == Catch::Approx(1.0 / 60.0).margin(1e-15));
    REQUIRE(plan.seconds == Catch::Approx((static_cast<double>(plan.samples.size()) - 1.0) * plan.dt).margin(1e-12));
    REQUIRE(plan.seconds == Catch::Approx(192.0 / 60.0).margin(1e-12));
    auto query = sim::createCollisionQuery();
    query->rebuild(scene);
    bool near = false;
    double gap = 1e9;
    for (const sim::ContactHit& hit : query->query(scene, scene.proximity.clearanceMm)) {
        if (hit.a == "Fixture" || hit.b == "Fixture") {
            near = true;
            gap = std::min(gap, hit.distance);
        }
    }
    std::cerr << "home clearance mm " << gap << " near " << near << "\n";
    REQUIRE(near);
    REQUIRE(gap < scene.proximity.clearanceMm + 1e-6);
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }

    const std::vector<double> nudgeAgain{18, -12, 15, 25, -10, 20};
    sim::setJointVector(*resource, std::vector<double>(6, 0));
    const sim::CartesianPose homePose = sim::forwardModel(*resource);
    sim::setJointVector(*resource, nudgeAgain);
    const sim::CartesianPose farPose = sim::forwardModel(*resource);
    auto spotFrom = [](const char* id, const sim::CartesianPose& pose) {
        sim::WeldSpot spot;
        spot.id = id;
        spot.position = pose.position;
        const sim::Vec3 toolZ = pose.rotation.rotate({0, 0, 1});
        spot.normal = sim::length(toolZ) > 0.5 ? sim::Vec3{-toolZ.x, -toolZ.y, -toolZ.z} : sim::Vec3{0, 0, 1};
        return spot;
    };
    scene.welds = {spotFrom("near", homePose), spotFrom("far", farPose)};
    sim::setJointVector(*resource, std::vector<double>(6, 0));
    const int tourPoints = sim::planExampleWeldTour(scene);
    std::cerr << "weld tour points " << tourPoints << "\n";
    REQUIRE(tourPoints == 4);
    const sim::RobotPath* tour = nullptr;
    for (const sim::RobotPath& candidate : scene.robotPaths) {
        if (candidate.name == "weld_tour") {
            tour = &candidate;
        }
    }
    REQUIRE(tour != nullptr);
    const sim::MotionPlan tourPlan = sim::planRobotPath(scene, *resource, *tour);
    REQUIRE(tourPlan.warnings.empty());
    REQUIRE(tourPlan.ok);
    REQUIRE(tourPlan.segmentSeconds.size() == 4);
    REQUIRE(tourPlan.segmentSeconds[0] == Catch::Approx(1.0 / 60.0).margin(1e-9));
    REQUIRE(tourPlan.segmentSeconds[1] == Catch::Approx(1.0 / 60.0).margin(1e-9));
    REQUIRE(tourPlan.segmentSeconds[2] == Catch::Approx(178.0 / 60.0).margin(1e-9));
    REQUIRE(tourPlan.segmentSeconds[3] == Catch::Approx(178.0 / 60.0).margin(1e-9));
    const double tourRaw = tourPlan.segmentSeconds[0] + tourPlan.segmentSeconds[1] + tourPlan.segmentSeconds[2] +
                           tourPlan.segmentSeconds[3];
    REQUIRE(tourPlan.samples.size() == 359);
    REQUIRE(tourPlan.seconds == Catch::Approx(358.0 / 60.0).margin(1e-9));
    REQUIRE(tourPlan.seconds == Catch::Approx(tourRaw).margin(1e-9));
    std::cerr << "weld tour seconds " << tourPlan.seconds << "\n";
}

sim::KinematicResource redundantArm() {
    sim::KinematicResource resource;
    resource.name = "arm";
    resource.ik = "spherical_wrist";
    resource.wrist.d1 = 300;
    resource.wrist.a2 = 400;
    resource.wrist.a3 = 300;
    resource.wrist.d6 = 0;
    for (int i = 0; i < 6; ++i) {
        sim::KinJoint joint;
        joint.name = "A" + std::to_string(i + 1);
        joint.lower = -180;
        joint.upper = 180;
        joint.maxSpeed = 180;
        joint.maxAccel = 360;
        resource.joints.push_back(joint);
    }
    return resource;
}

sim::KinJoint& addExternal(sim::KinematicResource& resource, const std::string& name, const std::string& kind, sim::JointMotion motion,
                          sim::Vec3 axis, double lower, double upper) {
    sim::KinJoint joint;
    joint.name = name;
    joint.externalName = name;
    joint.externalKind = kind;
    joint.motion = motion;
    joint.axis = axis;
    joint.lower = lower;
    joint.upper = upper;
    joint.limited = true;
    joint.maxSpeed = motion == sim::JointMotion::Prismatic ? 1500 : 60;
    joint.maxAccel = motion == sim::JointMotion::Prismatic ? 1500 : 120;
    resource.joints.push_back(std::move(joint));
    return resource.joints.back();
}

double externalAt(const std::vector<double>& joints, const sim::KinematicResource& resource, const std::string& name) {
    for (std::size_t i = 0; i < resource.joints.size() && i < joints.size(); ++i) {
        if (resource.joints[i].externalName == name) {
            return joints[i];
        }
    }
    return 0;
}

double distanceToSegment(sim::Vec3 point, sim::Vec3 a, sim::Vec3 b) {
    const sim::Vec3 ab = b - a;
    const double denom = sim::dot(ab, ab);
    if (denom < 1e-12) {
        return sim::length(point - a);
    }
    const double t = std::clamp(sim::dot(point - a, ab) / denom, 0.0, 1.0);
    return sim::length(point - (a + ab * t));
}

TEST_CASE("external track shifts the chain and mounts the arm") {
    sim::KinematicResource arm = redundantArm();
    addExternal(arm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 10000);
    const sim::CartesianPose home = sim::forwardModel(arm);
    REQUIRE(std::abs(home.position.x - 700) < 1e-4);
    REQUIRE(std::abs(home.position.y) < 1e-4);
    REQUIRE(std::abs(home.position.z - 300) < 1e-4);
    arm.joints.back().position = 1000;
    const sim::CartesianPose slid = sim::forwardModel(arm);
    REQUIRE(std::abs(slid.position.x - home.position.x - 1000) < 1e-3);
    REQUIRE(std::abs(slid.position.y - home.position.y) < 1e-3);
    REQUIRE(std::abs(slid.position.z - home.position.z) < 1e-3);

    sim::Scene scene;
    auto& robot = sim::ensureResource(scene, "robot");
    robot.ik = "numerical";
    sim::addLink(scene, robot, "rail", "", {});
    sim::addLink(scene, robot, "carriage", "rail", {});
    sim::addLink(scene, robot, "arm", "carriage", {});
    sim::KinJoint& slide = sim::addJoint(robot, scene, "slide", "rail", "carriage", sim::JointMotion::Prismatic, {1, 0, 0}, {});
    slide.lower = 0;
    slide.upper = 10000;
    slide.externalName = "E1";
    slide.externalKind = "track";
    sim::KinLink* armLink = sim::findLink(robot, "arm");
    REQUIRE(armLink != nullptr);
    armLink->mountedOn = "carriage";
    armLink->mountRest = {};
    sim::NamedFrame tcp;
    tcp.name = "tcp";
    tcp.link = "arm";
    tcp.origin = {500, 0, 200};
    robot.frames.push_back(tcp);
    const sim::CartesianPose atZero = sim::forwardModel(robot);
    REQUIRE(std::abs(atZero.position.x - 500) < 1e-4);
    REQUIRE(std::abs(atZero.position.z - 200) < 1e-4);
    slide.position = 1000;
    const sim::CartesianPose atFar = sim::forwardModel(robot);
    REQUIRE(std::abs(atFar.position.x - 1500) < 1e-3);
    REQUIRE(std::abs(atFar.position.y) < 1e-3);
    REQUIRE(std::abs(atFar.position.z - 200) < 1e-3);
}

TEST_CASE("redundancy resolution can hold, shorten travel, center joints, or open the elbow") {
    const sim::CartesianPose target{{700, 0, 300}, sim::Quat::identity()};

    sim::KinematicResource fixedArm = redundantArm();
    addExternal(fixedArm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 1000);
    fixedArm.redundancy.mode = "fixed";
    fixedArm.redundancy.fixed = {0};
    const sim::IkResult atZero = sim::inverseModel(fixedArm, target, {});
    REQUIRE(atZero.ok);
    REQUIRE(std::abs(externalAt(atZero.joints, fixedArm, "E1")) < 1e-3);
    fixedArm.redundancy.fixed = {500};
    const sim::IkResult atRail = sim::inverseModel(fixedArm, target, {});
    REQUIRE(atRail.ok);
    REQUIRE(std::abs(externalAt(atRail.joints, fixedArm, "E1") - 500) < 1e-3);

    sim::KinematicResource travelArm = redundantArm();
    addExternal(travelArm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 800);
    travelArm.redundancy.mode = "fixed";
    travelArm.redundancy.fixed = {200};
    const sim::IkResult bent = sim::inverseModel(travelArm, target, {});
    REQUIRE(bent.ok);
    sim::setJointVector(travelArm, bent.joints);
    travelArm.redundancy.mode = "travel";
    travelArm.redundancy.travel = 1;
    travelArm.redundancy.midrange = 0;
    travelArm.redundancy.manipulability = 0;
    travelArm.redundancy.fixed.clear();
    const sim::IkResult shortTravel = sim::inverseModel(travelArm, target, sim::jointVector(travelArm));
    REQUIRE(shortTravel.ok);
    const double travelE = externalAt(shortTravel.joints, travelArm, "E1");
    std::cerr << "travel external " << travelE << "\n";
    REQUIRE(std::abs(travelE - 200) < std::abs(travelE - 0));

    sim::KinematicResource midArm = redundantArm();
    addExternal(midArm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, -400, 400);
    midArm.redundancy.mode = "fixed";
    midArm.redundancy.fixed = {200};
    const sim::IkResult midSeed = sim::inverseModel(midArm, target, {});
    REQUIRE(midSeed.ok);
    sim::setJointVector(midArm, midSeed.joints);
    midArm.redundancy.mode = "midrange";
    midArm.redundancy.travel = 0;
    midArm.redundancy.midrange = 1;
    midArm.redundancy.manipulability = 0;
    midArm.redundancy.fixed.clear();
    const sim::IkResult centered = sim::inverseModel(midArm, target, sim::jointVector(midArm));
    REQUIRE(centered.ok);
    const double midE = externalAt(centered.joints, midArm, "E1");
    std::cerr << "midrange external " << midE << "\n";
    REQUIRE(std::abs(midE) < std::abs(midE - 200));

    sim::KinematicResource openArm = redundantArm();
    addExternal(openArm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 800);
    openArm.redundancy.mode = "manipulability";
    openArm.redundancy.travel = 0.05;
    openArm.redundancy.midrange = 0.05;
    openArm.redundancy.manipulability = 1;
    const sim::IkResult opened = sim::inverseModel(openArm, target, {});
    REQUIRE(opened.ok);
    const double openE = externalAt(opened.joints, openArm, "E1");
    std::cerr << "manipulability external " << openE << "\n";
    // Translational manipulability for this arm peaks near elbow 63° (track 100 mm),
    // not at 90°. The base joint's lever arm is longer there than at the square elbow.
    // The stretched pose (track 0) is still rejected.
    REQUIRE(std::abs(openE - 100) < 1.0);
    REQUIRE(std::abs(openE) > 50);

    sim::KinematicResource held = redundantArm();
    addExternal(held, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, -400, 400);
    addExternal(held, "E2", "positioner", sim::JointMotion::Revolute, {0, 0, 1}, -180, 180);
    addExternal(held, "E3", "positioner", sim::JointMotion::Revolute, {0, 0, 1}, -180, 180);
    held.joints.back().position = 40;
    held.redundancy.mode = "midrange";
    held.redundancy.travel = 0;
    held.redundancy.midrange = 1;
    held.redundancy.manipulability = 0;
    const sim::CartesianPose home = sim::forwardModel(held);
    const sim::IkResult kept = sim::inverseModel(held, home, sim::jointVector(held));
    REQUIRE(kept.ok);
    REQUIRE(std::abs(externalAt(kept.joints, held, "E3") - 40) < 1e-3);
}

TEST_CASE("synchronised linear motion keeps the tcp on the segment and waits for the track") {
    sim::Scene scene;
    auto& arm = sim::ensureResource(scene, "arm");
    arm = redundantArm();
    addExternal(arm, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 10000);
    const sim::CartesianPose start = sim::forwardModel(arm);
    sim::RobotPath path;
    path.name = "long";
    path.resource = arm.name;
    path.external = "sync";
    sim::PathPoint far;
    far.name = "far";
    far.move = sim::PathMove::Lin;
    far.cartesian = true;
    far.position = start.position + sim::Vec3{8000, 0, 0};
    far.orientation = start.rotation;
    far.speed = 5000;
    path.points.push_back(far);
    const sim::MotionPlan plan = sim::planRobotPath(scene, arm, path);
    std::cerr << "sync seconds " << plan.seconds << " samples " << plan.samples.size() << "\n";
    for (const std::string& warning : plan.warnings) {
        std::cerr << "sync warning " << warning << "\n";
    }
    REQUIRE(plan.ok);
    REQUIRE(plan.seconds == Catch::Approx(380.0 / 60.0).margin(1e-9));
    REQUIRE(plan.samples.size() == 381);
    const double moved = plan.samples.back()[6] - plan.samples.front()[6];
    REQUIRE(moved > 2000);
    for (const std::vector<double>& sample : plan.samples) {
        REQUIRE(sample[6] >= -1e-3);
        REQUIRE(sample[6] <= 10000 + 1e-3);
        sim::setJointVector(arm, sample);
        const sim::CartesianPose pose = sim::forwardModel(arm);
        REQUIRE(distanceToSegment(pose.position, start.position, far.position) < 2.0);
    }
    sim::setJointVector(arm, plan.samples.back());
    REQUIRE(sim::length(sim::forwardModel(arm).position - far.position) < 2.0);
}

TEST_CASE("positioner sync keeps the tcp on the workpiece and a track reports partial reach") {
    sim::Scene scene;
    auto& robot = sim::ensureResource(scene, "arm");
    robot.ik = "spherical_wrist";
    robot.wrist.a2 = 600;
    robot.wrist.a3 = 600;
    robot.wrist.d1 = 0;
    robot.wrist.d6 = 0;
    for (int i = 0; i < 6; ++i) {
        sim::KinJoint joint;
        joint.name = "A" + std::to_string(i + 1);
        joint.lower = -180;
        joint.upper = 180;
        robot.joints.push_back(joint);
    }
    sim::addLink(scene, robot, "ground", "", {});
    sim::addLink(scene, robot, "platter", "ground", {});
    sim::KinJoint& spin = sim::addJoint(robot, scene, "spin", "ground", "platter", sim::JointMotion::Revolute, {0, 0, 1}, {});
    spin.externalName = "E1";
    spin.externalKind = "positioner";
    spin.lower = -180;
    spin.upper = 180;
    spin.maxSpeed = 60;
    spin.maxAccel = 120;
    sim::NamedFrame work;
    work.name = "work";
    work.link = "platter";
    work.origin = {300, 0, 0};
    robot.frames.push_back(work);

    sim::CartesianPose start{{300, 0, 50}, sim::Quat::identity()};
    const sim::IkResult home = sim::inverseModel(robot, start, {});
    REQUIRE(home.ok);
    sim::setJointVector(robot, home.joints);
    REQUIRE(sim::length(sim::forwardModel(robot).position - start.position) < 1.0);

    spin.position = 90;
    sim::CartesianPose frame;
    REQUIRE(sim::namedFrameWorld(robot, "work", frame));
    const sim::Vec3 turned = frame.position + frame.rotation.rotate({0, 0, 50});
    REQUIRE(sim::length(turned - sim::Vec3{0, 300, 50}) < 1e-3);
    sim::CartesianPose goal;
    goal.position = turned;
    goal.rotation = frame.rotation;
    const sim::IkResult atTurn = sim::inverseModel(robot, goal, sim::jointVector(robot));
    REQUIRE(atTurn.ok);
    sim::setJointVector(robot, atTurn.joints);
    REQUIRE(sim::length(sim::forwardModel(robot).position - goal.position) < 1.0);

    sim::setJointVector(robot, home.joints);
    sim::RobotPath path;
    path.name = "spin";
    path.resource = robot.name;
    path.external = "positioner";
    path.workpiece = "work";
    sim::PathPoint point;
    point.name = "spot";
    point.move = sim::PathMove::Lin;
    point.cartesian = true;
    point.position = {0, 0, 50};
    point.orientation = sim::Quat::identity();
    point.externalValues = {90};
    point.speed = 5000;
    path.points.push_back(point);
    const sim::MotionPlan plan = sim::planRobotPath(scene, robot, path);
    REQUIRE(plan.ok);
    REQUIRE(plan.seconds > 1.5);
    for (const std::vector<double>& sample : plan.samples) {
        sim::setJointVector(robot, sample);
        sim::CartesianPose live;
        REQUIRE(sim::namedFrameWorld(robot, "work", live));
        const sim::Vec3 onPart = live.position + live.rotation.rotate({0, 0, 50});
        REQUIRE(sim::length(sim::forwardModel(robot).position - onPart) < 1.0);
    }

    sim::KinematicResource reach = redundantArm();
    addExternal(reach, "E1", "track", sim::JointMotion::Prismatic, {1, 0, 0}, 0, 2000);
    const sim::CartesianPose far{{1500, 0, 300}, sim::Quat::identity()};
    const sim::ReachReport report = sim::analyzeReach(reach, far, 9);
    REQUIRE(report.samples == 9);
    REQUIRE(report.reachable == 4);

    const std::vector<sim::OlpAxisName> names = sim::externalExportMap(reach);
    REQUIRE(names.size() == 1);
    REQUIRE(names[0].e == "E1");
    REQUIRE(names[0].rapid == "eax_a");
    REQUIRE(names[0].fanuc == "GP2:J1");

    sim::Scene mounted;
    sim::ensureResource(mounted, "robot");
    sim::ensureResource(mounted, "kl");
    sim::KinematicResource& userRobot = *sim::findResource(mounted.resources, "robot");
    sim::KinematicResource& carrier = *sim::findResource(mounted.resources, "kl");
    sim::addLink(mounted, userRobot, "base", "", {});
    sim::addLink(mounted, carrier, "rail", "", {});
    sim::addLink(mounted, carrier, "carriage", "rail", {});
    sim::KinJoint& rail = sim::addJoint(carrier, mounted, "slide", "rail", "carriage", sim::JointMotion::Prismatic, {1, 0, 0}, {});
    rail.upper = 10000;
    std::string error;
    REQUIRE(sim::attachExternalAxes(mounted, userRobot, "kl", "carriage", "track", error));
    REQUIRE(error.empty());
    REQUIRE(userRobot.joints.back().externalName == "E1");
    REQUIRE(userRobot.joints.back().externalKind == "track");
    sim::KinLink* base = sim::findLink(userRobot, "base");
    REQUIRE(base != nullptr);
    REQUIRE(base->mountedOn == "carriage");
    const auto baseNode = mounted.find(base->nodeId);
    const auto carriage = mounted.find(sim::findLink(userRobot, "carriage")->nodeId);
    REQUIRE(baseNode);
    REQUIRE(carriage);
    REQUIRE(baseNode->parent.lock() == carriage);

    reach.redundancy.mode = "travel";
    reach.redundancy.travel = 1;
    reach.redundancy.fixed = {250};
    sim::RobotPath stored;
    stored.name = "job";
    stored.resource = reach.name;
    stored.external = "sync";
    stored.workpiece = "work";
    sim::PathPoint storedPoint;
    storedPoint.external = "sync";
    storedPoint.externalValues = {250, 10};
    stored.points.push_back(storedPoint);
    std::string jsonError;
    sim::KinematicResource loaded;
    REQUIRE(sim::kinematicsFromJson(sim::kinematicsToJson(reach), loaded, jsonError));
    REQUIRE(loaded.joints.back().externalName == "E1");
    REQUIRE(loaded.joints.back().externalKind == "track");
    REQUIRE(loaded.redundancy.mode == "travel");
    REQUIRE(loaded.redundancy.fixed.size() == 1);
    REQUIRE(std::abs(loaded.redundancy.fixed[0] - 250) < 1e-9);
    sim::RobotPath loadedPath;
    REQUIRE(sim::robotPathFromJson(sim::robotPathToJson(stored), loadedPath, jsonError));
    REQUIRE(loadedPath.external == "sync");
    REQUIRE(loadedPath.workpiece == "work");
    REQUIRE(loadedPath.points[0].external == "sync");
    REQUIRE(loadedPath.points[0].externalValues.size() == 2);
}

TEST_CASE("kr420 on a ten metre track follows a line longer than its reach") {
    const std::filesystem::path step = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/KR420_R3080_KR280_R3080.stp";
    if (!std::filesystem::exists(step)) {
        SKIP("local KR 420 STEP is not in the tree");
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult imported = importer.importFile(step.string(), scene);
    REQUIRE(imported.ok);
    const sim::SuggestResult suggestion = sim::suggestLinkGroups(scene);
    std::string error;
    REQUIRE(sim::applyLinkSuggestions(scene, "kr420", suggestion, sim::kr420DatasheetLimits(), error));
    REQUIRE(sim::addKr420TrackDemonstration(scene, error));
    sim::KinematicResource* robot = sim::findResource(scene.resources, "kr420");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->joints.size() == 7);
    REQUIRE(robot->joints.back().externalName == "E1");
    REQUIRE(robot->joints.back().externalKind == "track");
    REQUIRE(robot->joints.back().upper == 10000);
    const sim::RobotPath* path = nullptr;
    for (const sim::RobotPath& candidate : scene.robotPaths) {
        if (candidate.name == "long") {
            path = &candidate;
        }
    }
    REQUIRE(path != nullptr);
    const sim::CartesianPose start = sim::forwardModel(*robot);
    const auto hidden = sim::hideAllMeshedParts(scene);
    const sim::MotionPlan plan = sim::planRobotPath(scene, *robot, *path);
    std::cerr << "kr420 track seconds " << plan.seconds << " samples " << plan.samples.size() << "\n";
    for (const std::string& warning : plan.warnings) {
        std::cerr << "kr420 track warning " << warning << "\n";
    }
    REQUIRE(plan.ok);
    REQUIRE(plan.seconds > 6.0);
    REQUIRE(plan.samples.back()[6] - plan.samples.front()[6] > 3000);
    double partX = 0;
    if (const auto base = sim::findLink(*robot, "base")) {
        if (const auto node = scene.find(base->nodeId)) {
            if (!node->children.empty() && node->children.front()) {
                partX = node->children.front()->worldMatrix()(3, 0);
            }
        }
    }
    sim::setJointVector(*robot, plan.samples.back());
    sim::applyResource(scene, *robot);
    double partXFar = partX;
    if (const auto base = sim::findLink(*robot, "base")) {
        if (const auto node = scene.find(base->nodeId)) {
            if (!node->children.empty() && node->children.front()) {
                partXFar = node->children.front()->worldMatrix()(3, 0);
            }
        }
    }
    std::cerr << "scene part x " << partX << " -> " << partXFar << " track " << plan.samples.back()[6] << "\n";
    REQUIRE(partXFar - partX > 3000);
    sim::setJointVector(*robot, plan.samples[plan.samples.size() / 2]);
    const sim::CartesianPose mid = sim::forwardModel(*robot);
    const sim::Vec3 end = path->points.back().position;
    REQUIRE(distanceToSegment(mid.position, start.position, end) < 5.0);
    sim::setJointVector(*robot, plan.samples.back());
    REQUIRE(sim::length(sim::forwardModel(*robot).position - end) < 5.0);
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }
}

TEST_CASE("zone blend keeps joint velocity continuous and inside the limits") {
    const double q0 = 2;
    const double q1 = 8;
    const double slope0 = 3.5;
    const double slope1 = -1.25;
    REQUIRE(sim::zoneBlend(0, q0, q1, slope0, slope1) == Catch::Approx(q0));
    REQUIRE(sim::zoneBlend(1, q0, q1, slope0, slope1) == Catch::Approx(q1));
    const double eps = 1e-5;
    const double startSlope = (sim::zoneBlend(eps, q0, q1, slope0, slope1) - sim::zoneBlend(0, q0, q1, slope0, slope1)) / eps;
    const double endSlope = (sim::zoneBlend(1, q0, q1, slope0, slope1) - sim::zoneBlend(1 - eps, q0, q1, slope0, slope1)) / eps;
    REQUIRE(startSlope == Catch::Approx(slope0).margin(1e-3));
    REQUIRE(endSlope == Catch::Approx(slope1).margin(1e-3));
    REQUIRE(sim::zoneBlend(0.5, 40, 40, 12, -12) > 40);

    sim::KinematicResource resource;
    resource.name = "axis";
    resource.ik = "none";
    sim::KinJoint joint;
    joint.name = "J1";
    joint.lower = -40;
    joint.upper = 40;
    joint.maxSpeed = 120;
    joint.maxAccel = 400;
    joint.limited = true;
    resource.joints.push_back(joint);

    sim::Scene scene;
    sim::RobotPath finePath;
    finePath.name = "turn";
    finePath.resource = resource.name;
    sim::PathPoint out;
    out.name = "out";
    out.move = sim::PathMove::Ptp;
    out.joints = {40};
    out.speed = 80;
    sim::PathPoint back;
    back.name = "back";
    back.move = sim::PathMove::Ptp;
    back.joints = {-20};
    back.speed = 80;
    finePath.points = {out, back};
    const sim::MotionPlan fine = sim::planRobotPath(scene, resource, finePath);

    sim::RobotPath zoned = finePath;
    zoned.points[0].zone = 30;
    const sim::MotionPlan blended = sim::planRobotPath(scene, resource, zoned);
    REQUIRE(blended.seconds + 1e-4 < fine.seconds);
    double raw = 0;
    for (double span : blended.segmentSeconds) {
        raw += span;
    }
    REQUIRE(fine.seconds == Catch::Approx(86.0 / 60.0).margin(1e-9));
    REQUIRE(raw == Catch::Approx(86.0 / 60.0).margin(1e-9));
    REQUIRE(blended.seconds == Catch::Approx(75.0 / 60.0).margin(1e-9));
    REQUIRE(blended.samples.size() == 76);
    for (const std::vector<double>& sample : blended.samples) {
        REQUIRE(sample[0] <= 40.0 + 1e-3);
        REQUIRE(sample[0] >= -40.0 - 1e-3);
    }
}

TEST_CASE("logic text, wires, and the running step stay in sync") {
    const char* text = R"(name load
in go
out done
param dwell real 0.2
python:
def on_step(dt):
    sim.set_bool("go", True)
---
wait 0.2
set_output done
step
)";
    sim::LogicBlock block;
    std::string error;
    REQUIRE(sim::logicFromText(text, block, error));
    REQUIRE(block.name == "load");
    REQUIRE(block.inputs.size() == 1);
    REQUIRE(block.outputs.size() == 1);
    REQUIRE(block.parameters.size() == 1);
    REQUIRE(block.parameters[0].real == Catch::Approx(0.2));
    REQUIRE(block.python.find("set_bool") != std::string::npos);
    REQUIRE(block.steps.size() == 1);
    REQUIRE(block.steps[0].actions.size() == 2);
    REQUIRE(block.steps[0].actions[1].signal == "done");
    const std::string written = sim::logicToText(block);
    sim::LogicBlock again;
    REQUIRE(sim::logicFromText(written, again, error));
    REQUIRE(again.python == block.python);
    REQUIRE(again.steps[0].actions[0].seconds == Catch::Approx(0.2));

    sim::LogicBlock next;
    next.name = "weld";
    std::string signal;
    REQUIRE(sim::wireLogicBlocks(block, next, signal));
    REQUIRE(signal == "done");
    REQUIRE(std::find(next.inputs.begin(), next.inputs.end(), "done") != next.inputs.end());

    sim::Scene scene;
    scene.logicBlocks = {block, next};
    sim::LogicMachine machine;
    REQUIRE(machine.start(scene, "load", error));
    REQUIRE(machine.activeStep("load") == 0);
    machine.step(scene, 0.05);
    REQUIRE(machine.activeStep("load") == 0);
    machine.step(scene, 0.2);
    REQUIRE(scene.findSignal("done") != nullptr);
    REQUIRE(scene.findSignal("done")->flag);
}

TEST_CASE("editor devices are built from links joints and poses") {
    sim::Scene scene;
    sim::buildEditorDevices(scene);
    REQUIRE(scene.resources.size() == 3);

    auto* turntable = sim::findResource(scene.resources, "turntable");
    auto* bed = sim::findResource(scene.resources, "rollerbed");
    auto* hoist = sim::findResource(scene.resources, "hoist");
    REQUIRE(turntable != nullptr);
    REQUIRE(bed != nullptr);
    REQUIRE(hoist != nullptr);

    const sim::KinJoint* spin = sim::findJoint(*turntable, "spin");
    REQUIRE(spin != nullptr);
    REQUIRE(spin->motion == sim::JointMotion::Revolute);
    REQUIRE(sim::findPose(*turntable, "home") != nullptr);
    REQUIRE(sim::findPose(*turntable, "quarter") != nullptr);
    REQUIRE(sim::findPose(*turntable, "quarter")->joints.at(0).second == Catch::Approx(90));

    const sim::KinJoint* travel = sim::findJoint(*bed, "travel");
    const sim::KinJoint* roll = sim::findJoint(*bed, "roll");
    REQUIRE(travel != nullptr);
    REQUIRE(roll != nullptr);
    REQUIRE(travel->motion == sim::JointMotion::Prismatic);
    REQUIRE(travel->upper == Catch::Approx(800));
    REQUIRE(roll->motion == sim::JointMotion::Revolute);
    REQUIRE(roll->coupledTo == "travel");
    REQUIRE(roll->coupleRatio == Catch::Approx(0.02));
    REQUIRE(sim::findPose(*bed, "mid") != nullptr);
    REQUIRE(sim::findPose(*bed, "mid")->joints.at(0).second == Catch::Approx(400));

    sim::KinJoint* rise = sim::findJoint(*hoist, "rise");
    REQUIRE(rise != nullptr);
    REQUIRE(rise->motion == sim::JointMotion::Prismatic);
    REQUIRE(rise->upper == Catch::Approx(400));
    REQUIRE(sim::findPose(*hoist, "down") != nullptr);
    REQUIRE(sim::findPose(*hoist, "up") != nullptr);
    REQUIRE(sim::findPose(*hoist, "up")->joints.at(0).second == Catch::Approx(300));

    auto carriage = scene.findByName("HoistCarriage");
    REQUIRE(carriage);
    REQUIRE(carriage->parent.lock());
    REQUIRE(carriage->parent.lock()->name == "carriage");
    const double z0 = carriage->worldMatrix()(3, 2);
    rise->position = 300;
    sim::applyResource(scene, *hoist);
    REQUIRE(carriage->worldMatrix()(3, 2) == Catch::Approx(z0 + 300));

    auto belt = scene.findByName("BedBelt");
    REQUIRE(belt);
    const double x0 = belt->worldMatrix()(3, 0);
    sim::findJoint(*bed, "travel")->position = 400;
    sim::applyResource(scene, *bed);
    REQUIRE(belt->worldMatrix()(3, 0) == Catch::Approx(x0 + 400));
    REQUIRE(sim::jointCommand(*bed, *roll) == Catch::Approx(8));

    auto platen = scene.find(sim::findLink(*turntable, "platen")->nodeId);
    REQUIRE(platen);
    sim::findJoint(*turntable, "spin")->position = 90;
    sim::applyResource(scene, *turntable);
    const sim::Quat turned = platen->local.rotation.normalized();
    REQUIRE(std::abs(turned.z) > 0.6);
    REQUIRE(std::abs(turned.w) > 0.6);

    std::string error;
    REQUIRE(sim::jogJoint(*hoist, "rise", 50, error));
    REQUIRE(sim::findJoint(*hoist, "rise")->position == Catch::Approx(350));
}

TEST_CASE("scope records signals, joint velocity, and csv") {
    sim::Scene scene;
    sim::buildEditorDevices(scene);
    sim::Signal go;
    go.name = "go";
    go.type = sim::SignalType::Bool;
    scene.signals.push_back(go);
    sim::ScopeTrace trace;
    trace.watch({"sig:go", "jnt:hoist/rise"});
    sim::KinJoint* rise = sim::findJoint(*sim::findResource(scene.resources, "hoist"), "rise");
    REQUIRE(rise != nullptr);
    const double positions[] = {0, 10, 30};
    for (int i = 0; i < 3; ++i) {
        rise->position = positions[i];
        scene.findSignal("go")->flag = i >= 2;
        trace.sample(scene, i * 0.1);
    }
    REQUIRE(trace.time.size() == 3);
    REQUIRE(trace.valueAt("sig:go", 0.05) == Catch::Approx(0));
    REQUIRE(trace.valueAt("sig:go", 0.2) == Catch::Approx(1));
    REQUIRE(trace.valueAt("sig:go", 0.25) == Catch::Approx(1));
    const std::vector<double> speed = trace.velocity("jnt:hoist/rise");
    const std::vector<double> accel = trace.acceleration("jnt:hoist/rise");
    REQUIRE(speed.size() == 3);
    REQUIRE(speed[0] == Catch::Approx(0));
    REQUIRE(speed[1] == Catch::Approx(100));
    REQUIRE(speed[2] == Catch::Approx(200));
    REQUIRE(accel[1] == Catch::Approx(1000));
    REQUIRE(accel[2] == Catch::Approx(1000));
    const std::string csv = trace.toCsv(true, true);
    REQUIRE(csv.find("time,sig:go,jnt:hoist/rise,jnt:hoist/rise.v,jnt:hoist/rise.a") == 0);
    REQUIRE(csv.find("0.200000,1.000000,30.000000,200.000000,1000.000000") != std::string::npos);

    trace.watch({"sig:missing"});
    trace.sample(scene, 0.3);
    REQUIRE(trace.valueAt("sig:missing", 0.3) == Catch::Approx(0));
    for (int i = 0; i < 240000; ++i) {
        trace.sample(scene, 1.0 + i * 0.01);
    }
    REQUIRE(trace.time.size() == sim::ScopeTrace::kMaxSamples);
}

TEST_CASE("robot catalog copies published limits and link origins") {
    const EnvGuard offline("SIM_CATALOG_MESH_OFFLINE", "1");
    const EnvGuard pinned("SIM_CATALOG_MESH_DIR", emptyMeshDir().string());
    const std::vector<sim::CatalogRobot>& catalog = sim::robotCatalog();
    REQUIRE(catalog.size() == 4);
    sim::Scene scene;
    const char* names[] = {"KR6_R900", "IRB120", "LRMate200iD", "GP8"};
    for (int i = 0; i < 4; ++i) {
        std::string error;
        REQUIRE(sim::addCatalogRobot(scene, names[i], {i * 1400.0, 0, 0}, error));
        const sim::KinematicResource* resource = sim::findResource(scene.resources, names[i]);
        REQUIRE(resource != nullptr);
        REQUIRE(resource->joints.size() == 6);
        REQUIRE(resource->source.find("http") != std::string::npos);
        REQUIRE(resource->source.find("待核实") != std::string::npos);
        for (const sim::KinJoint& joint : resource->joints) {
            REQUIRE(joint.motion == sim::JointMotion::Revolute);
            REQUIRE(joint.accelSource == "velocity");
            REQUIRE(sim::jointAccelLimit(*resource, joint).source == "velocity");
            REQUIRE(sim::jointAccelLimit(*resource, joint).value > 1.0e8);
        }
        sim::KinematicResource loaded;
        REQUIRE(sim::kinematicsFromJson(sim::kinematicsToJson(*resource), loaded, error));
        REQUIRE(loaded.source == resource->source);
        REQUIRE(loaded.joints[1].lower == Catch::Approx(resource->joints[1].lower));
    }
    sim::KinematicResource* kr6 = sim::findResource(scene.resources, "KR6_R900");
    REQUIRE(sim::findJoint(*kr6, "A1")->lower == Catch::Approx(-170));
    REQUIRE(sim::findJoint(*kr6, "A1")->maxSpeed == Catch::Approx(360));
    REQUIRE(sim::findJoint(*kr6, "A2")->upper == Catch::Approx(45));
    REQUIRE(sim::findJoint(*kr6, "A6")->maxSpeed == Catch::Approx(615));
    auto a3 = scene.find(sim::findLink(*kr6, "A3")->nodeId);
    REQUIRE(a3);
    REQUIRE(a3->local.translation.x == Catch::Approx(455));

    const sim::KinematicResource* irb = sim::findResource(scene.resources, "IRB120");
    REQUIRE(sim::findJoint(*irb, "A3")->lower == Catch::Approx(-110));
    REQUIRE(sim::findJoint(*irb, "A3")->upper == Catch::Approx(70));
    REQUIRE(sim::findJoint(*irb, "A6")->lower == Catch::Approx(-400));
    REQUIRE(sim::findJoint(*irb, "A1")->maxSpeed == Catch::Approx(250));

    const sim::KinematicResource* mate = sim::findResource(scene.resources, "LRMate200iD");
    REQUIRE(sim::findJoint(*mate, "A3")->lower == Catch::Approx(-70));
    REQUIRE(sim::findJoint(*mate, "A3")->upper == Catch::Approx(205));
    REQUIRE(sim::findJoint(*mate, "A1")->maxSpeed == Catch::Approx(450));
    REQUIRE(mate->source.find("420") != std::string::npos);

    sim::KinematicResource* gp8 = sim::findResource(scene.resources, "GP8");
    REQUIRE(sim::findJoint(*gp8, "A2")->lower == Catch::Approx(-65));
    REQUIRE(sim::findJoint(*gp8, "A2")->upper == Catch::Approx(145));
    REQUIRE(sim::findJoint(*gp8, "A3")->upper == Catch::Approx(190));
    REQUIRE(sim::findJoint(*gp8, "A1")->maxSpeed == Catch::Approx(455));
    REQUIRE(sim::findLink(*gp8, "flange") != nullptr);

    auto wrist = scene.find(sim::findLink(*kr6, "A6")->nodeId);
    REQUIRE(wrist);
    const double before = wrist->worldMatrix()(3, 2);
    sim::findJoint(*kr6, "A2")->position = 30;
    sim::applyResource(scene, *kr6);
    REQUIRE(std::abs(wrist->worldMatrix()(3, 2) - before) > 1.0);

    std::string error;
    REQUIRE_FALSE(sim::addCatalogRobot(scene, "no-such-robot", {}, error));
}

TEST_CASE("joint acceleration comes from torque inertia payload or the user") {
    sim::Scene scene;
    sim::KinematicResource& resource = sim::ensureResource(scene, "dyn");
    sim::addLink(scene, resource, "base", "", {});
    sim::Transform local;
    local.translation = {1000, 0, 0};
    sim::addLink(scene, resource, "arm", "base", local);
    sim::KinJoint& joint = sim::addJoint(resource, scene, "j", "base", "arm", sim::JointMotion::Revolute, {0, 0, 1}, {0, 0, 0});
    joint.effort = 10;
    joint.accelSource = "dynamics";
    joint.maxAccel = 0;
    joint.maxSpeed = 1000;
    sim::KinLink* arm = sim::findLink(resource, "arm");
    REQUIRE(arm != nullptr);
    arm->hasInertia = true;
    arm->mass = 2;
    const sim::JointAccelLimit unloaded = sim::jointAccelLimit(resource, joint);
    REQUIRE(unloaded.source == "dynamics");
    REQUIRE(unloaded.value == Catch::Approx(10.0 / 2.0 * 180.0 / 3.14159265358979323846));

    resource.payload = 2;
    const sim::JointAccelLimit loaded = sim::jointAccelLimit(resource, joint);
    REQUIRE(loaded.source == "dynamics");
    REQUIRE(loaded.payloadScaled);
    REQUIRE(loaded.value == Catch::Approx(10.0 / 4.0 * 180.0 / 3.14159265358979323846));
    REQUIRE(loaded.value < unloaded.value);

    joint.accelSource = "user";
    joint.maxAccel = 100;
    const sim::JointAccelLimit typed = sim::jointAccelLimit(resource, joint);
    REQUIRE(typed.source == "user");
    REQUIRE(typed.payloadScaled);
    REQUIRE(typed.value == Catch::Approx(50));

    joint.accelSource = "velocity";
    joint.effort = 0;
    const sim::JointAccelLimit speedOnly = sim::jointAccelLimit(resource, joint);
    REQUIRE(speedOnly.source == "velocity");
    REQUIRE(speedOnly.value > 1.0e8);

    joint.accelSource = "dynamics";
    joint.effort = 10;
    resource.payload = 2;
    sim::ActiveMove move;
    std::string error;
    REQUIRE(sim::beginJointMove(resource, {90}, sim::ProfileKind::Trapezoid, move, error));
    REQUIRE(move.duration == Catch::Approx(sim::profileDuration(sim::ProfileKind::Trapezoid, 0, 90, 1000, loaded.value)));

    joint.accelSource = "velocity";
    REQUIRE(sim::beginJointMove(resource, {90}, sim::ProfileKind::Trapezoid, move, error));
    REQUIRE(move.duration < 1.0);
    REQUIRE(move.duration == Catch::Approx(0.09).margin(0.02));
}

TEST_CASE("the public example cell covers layout welds path clamp signals and cycle time") {
    const EnvGuard offline("SIM_CATALOG_MESH_OFFLINE", "1");
    const EnvGuard pinned("SIM_CATALOG_MESH_DIR", emptyMeshDir().string());
    sim::Scene scene;
    sim::buildPublicCell(scene);
    REQUIRE(sim::findResource(scene.resources, "kr420") == nullptr);
    sim::KinematicResource* robot = sim::findResource(scene.resources, "IRB120");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->payload == Catch::Approx(3));
    REQUIRE(sim::jointAccelLimit(*robot, robot->joints.front()).source == "velocity");
    sim::KinematicResource* clamp = sim::findResource(scene.resources, "clamp");
    REQUIRE(clamp != nullptr);
    const sim::JointAccelLimit slide = sim::jointAccelLimit(*clamp, *sim::findJoint(*clamp, "slide"));
    REQUIRE(slide.source == "dynamics");
    REQUIRE(slide.value == Catch::Approx(5000));
    REQUIRE(scene.layout.stations.size() == 3);
    REQUIRE(scene.welds.size() == 3);
    REQUIRE(scene.robotPaths.size() == 1);
    REQUIRE(scene.robotPaths.front().name == "weld");
    REQUIRE(scene.logicBlocks.size() == 1);
    REQUIRE(scene.findSignal("part_seen") != nullptr);
    REQUIRE(scene.findSignal("weld_done") != nullptr);
    REQUIRE(scene.findSignal("clamp") != nullptr);
    const sim::ScheduleReport report = sim::scheduleSequence(scene.operations, scene.sequence, scene.layout);
    REQUIRE(report.error.empty());
    const sim::ScheduleBar* unload = nullptr;
    for (const sim::ScheduleBar& bar : report.bars) {
        if (bar.id == "unload") {
            unload = &bar;
        }
    }
    REQUIRE(unload != nullptr);
    const sim::ScheduleBar* weld = nullptr;
    for (const sim::ScheduleBar& bar : report.bars) {
        if (bar.id == "weld") {
            weld = &bar;
        }
    }
    REQUIRE(weld != nullptr);
    REQUIRE(unload->start == Catch::Approx(weld->finish + 0.4));
    REQUIRE(report.makespan > weld->finish);
    const sim::Operation* weldOp = nullptr;
    for (const sim::Operation& operation : scene.operations) {
        if (operation.id == "weld") {
            weldOp = &operation;
        }
    }
    REQUIRE(weldOp != nullptr);
    REQUIRE(weldOp->timeSource == "plan");
    REQUIRE(weldOp->seconds == Catch::Approx(8.0 / 60.0));
    REQUIRE(report.makespan == Catch::Approx(2.0 + 8.0 / 60.0 + 0.4 + 2.0));
    REQUIRE(scene.findByName("IRB120_base") != nullptr);
    REQUIRE(scene.mesh(scene.findByName("IRB120_base")->meshId)->triangleCount() == 12);
}

void writeTetrahedron(const std::filesystem::path& path) {
    const float v[4][3] = {{0, 0, 0}, {0.01f, 0, 0}, {0, 0.01f, 0}, {0, 0, 0.01f}};
    const int faces[4][3] = {{0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2}};
    std::vector<char> bytes(84 + 4 * 50, 0);
    const std::uint32_t count = 4;
    std::memcpy(bytes.data() + 80, &count, 4);
    for (int face = 0; face < 4; ++face) {
        float values[12] = {};
        values[0] = 0;
        values[1] = 0;
        values[2] = 1;
        for (int corner = 0; corner < 3; ++corner) {
            for (int axis = 0; axis < 3; ++axis) {
                values[3 + corner * 3 + axis] = v[faces[face][corner]][axis];
            }
        }
        std::memcpy(bytes.data() + 84 + face * 50, values, sizeof(values));
    }
    std::ofstream out(path, std::ios::binary);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

TEST_CASE("a provided catalog stl replaces the link box and collides") {
    const auto root = std::filesystem::temp_directory_path() / "simstudio-catalog-stl";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root / "IRB120");
    writeTetrahedron(root / "IRB120" / "base_link.stl");
    const EnvGuard offline("SIM_CATALOG_MESH_OFFLINE", "1");
    const EnvGuard pinned("SIM_CATALOG_MESH_DIR", root.string());

    sim::Scene scene;
    std::string error;
    REQUIRE(sim::addCatalogRobot(scene, "IRB120", {}, error));
    auto base = scene.findByName("IRB120_base");
    REQUIRE(base);
    const auto mesh = scene.mesh(base->meshId);
    REQUIRE(mesh);
    REQUIRE(mesh->triangleCount() == 4);
    REQUIRE(mesh->positions[3] == Catch::Approx(10.0f));
    REQUIRE(mesh->positions[7] == Catch::Approx(10.0f));
    bool foundHeight = false;
    for (std::size_t i = 2; i < mesh->positions.size(); i += 3) {
        if (std::abs(mesh->positions[i] - 10.0f) < 1e-4f) {
            foundHeight = true;
        }
    }
    REQUIRE(foundHeight);
    auto arm = scene.findByName("IRB120_A1");
    REQUIRE(arm);
    REQUIRE(scene.mesh(arm->meshId)->triangleCount() == 12);
    const sim::KinematicResource* robot = sim::findResource(scene.resources, "IRB120");
    REQUIRE(robot->source.find("本机网格 1/7") != std::string::npos);

    auto obstacle = scene.makeNode(sim::NodeKind::Part, "obstacle");
    obstacle->meshId = scene.addMesh(sim::makeBoxMesh({3, 3, 3}));
    obstacle->local.translation = {2, 2, 2};
    scene.addChild(scene.root(), obstacle);
    auto query = sim::createCollisionQuery();
    query->rebuild(scene);
    bool hit = false;
    for (const sim::ContactHit& contact : query->query(scene, 0)) {
        const bool pair = (contact.a == "IRB120_base" && contact.b == "obstacle") ||
                          (contact.b == "IRB120_base" && contact.a == "obstacle");
        if (pair && contact.distance <= 0) {
            hit = true;
        }
    }
    REQUIRE(hit);

    obstacle->local.translation = {500, 0, 0};
    query->rebuild(scene);
    bool far = false;
    for (const sim::ContactHit& contact : query->query(scene, 0)) {
        const bool pair = (contact.a == "IRB120_base" && contact.b == "obstacle") ||
                          (contact.b == "IRB120_base" && contact.a == "obstacle");
        if (pair && contact.distance <= 0) {
            far = true;
        }
    }
    REQUIRE_FALSE(far);
}

TEST_CASE("public catalog meshes keep the example cycle", "[live-mesh]") {
    if (std::getenv("SIM_CATALOG_MESH_LIVE") == nullptr) {
        SKIP("set SIM_CATALOG_MESH_LIVE=1 to download the public meshes");
    }
    unsetenv("SIM_CATALOG_MESH_OFFLINE");
    unsetenv("SIM_CATALOG_MESH_DIR");
    sim::Scene scene;
    sim::buildPublicCell(scene);
    const sim::KinematicResource* robot = sim::findResource(scene.resources, "IRB120");
    REQUIRE(robot != nullptr);
    REQUIRE(robot->source.find("本机网格 7/7") != std::string::npos);
    int triangles = 0;
    for (const char* link : {"base", "A1", "A2", "A3", "A4", "A5", "A6"}) {
        auto part = scene.findByName(std::string("IRB120_") + link);
        REQUIRE(part != nullptr);
        const auto mesh = scene.mesh(part->meshId);
        REQUIRE(mesh);
        REQUIRE(mesh->triangleCount() > 12);
        triangles += static_cast<int>(mesh->triangleCount());
    }
    const sim::MotionPlan plan = sim::planRobotPath(scene, *sim::findResource(scene.resources, "IRB120"), scene.robotPaths.front());
    REQUIRE(plan.ok);
    REQUIRE(plan.warnings.empty());
    REQUIRE(plan.collisions.empty());
    REQUIRE(plan.samples.size() == 9);
    REQUIRE(plan.seconds == Catch::Approx(8.0 / 60.0));
    REQUIRE(triangles == 40020);
    const sim::ScheduleReport report = sim::scheduleSequence(scene.operations, scene.sequence, scene.layout);
    REQUIRE(report.makespan == Catch::Approx(2.0 + 8.0 / 60.0 + 0.4 + 2.0));
    auto base = scene.findByName("IRB120_base");
    const auto baseMesh = scene.mesh(base->meshId);
    const sim::Vec3 vertex{baseMesh->positions[0], baseMesh->positions[1], baseMesh->positions[2]};
    auto fixture = scene.makeNode(sim::NodeKind::Part, "fixture");
    fixture->meshId = scene.addMesh(sim::makeBoxMesh({30, 30, 30}));
    fixture->local.translation = sim::transformPoint(base->worldMatrix(), vertex);
    scene.addChild(scene.root(), fixture);
    auto query = sim::createCollisionQuery();
    query->rebuild(scene);
    bool hit = false;
    for (const sim::ContactHit& contact : query->query(scene, 0)) {
        const bool pair = (contact.a == "IRB120_base" && contact.b == "fixture") ||
                          (contact.b == "IRB120_base" && contact.a == "fixture");
        if (pair && contact.distance <= 0) {
            hit = true;
        }
    }
    REQUIRE(hit);
    scene.removeNode(fixture->id);

    const char* others[] = {"KR6_R900", "LRMate200iD", "GP8"};
    for (const char* name : others) {
        std::string error;
        REQUIRE(sim::addCatalogRobot(scene, name, {2000, 0, 0}, error));
        REQUIRE(sim::findResource(scene.resources, name)->source.find("本机网格 7/7") != std::string::npos);
        int modelTriangles = 0;
        for (const char* link : {"base", "A1", "A2", "A3", "A4", "A5", "A6"}) {
            auto part = scene.findByName(std::string(name) + "_" + link);
            REQUIRE(part != nullptr);
            modelTriangles += static_cast<int>(scene.mesh(part->meshId)->triangleCount());
        }
        const int expected = name == std::string("KR6_R900") ? 32288 : name == std::string("LRMate200iD") ? 25132 : 72498;
        REQUIRE(modelTriangles == expected);
    }
}
