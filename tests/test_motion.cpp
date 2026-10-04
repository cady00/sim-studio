#include "sim/ai/Commands.hpp"
#include "sim/collision/Query.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/motion/Motion.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace {

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
    REQUIRE(trap > 0.2);
    bool done = false;
    REQUIRE(sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, 0, &done) == 0);
    REQUIRE_FALSE(done);
    const double end = sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, trap, &done);
    REQUIRE(done);
    REQUIRE(std::abs(end - 90) < 1e-6);
    const double mid = sim::profileSample(sim::ProfileKind::Trapezoid, 0, 90, 120, 480, trap * 0.5, &done);
    REQUIRE(mid > 10);
    REQUIRE(mid < 90);

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
    REQUIRE_FALSE(empty.logicBlocks.empty());
}

TEST_CASE("robot path reports cycle time and a collision pair") {
    sim::Scene scene;
    sim::buildScaraDemo(scene);
    auto* resource = sim::findResource(scene.resources, "scara");
    REQUIRE(resource != nullptr);
    const sim::RobotPath& path = scene.robotPaths.at(0);
    const sim::MotionPlan fine = sim::planRobotPath(scene, *resource, path);
    REQUIRE(fine.seconds > 0.5);
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
    for (const auto& hit : hits) {
        UNSCOPED_INFO(hit.first << " vs " << hit.second);
    }
    REQUIRE_FALSE(hits.empty());
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
    REQUIRE(plan.value.at("seconds").get<double>() > 0);
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
    REQUIRE(std::abs(resource->joints[0].maxAccel - 180) < 1e-6);
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
    for (const std::string& warning : plan.warnings) {
        std::cerr << "plan warning " << warning << "\n";
    }
    REQUIRE(plan.seconds > 0);
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
}
