#include "sim/core/DemoScene.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using Catch::Approx;

namespace {

long procKilobytes(const char* field) {
    std::ifstream status("/proc/self/status");
    std::string line;
    const std::string prefix(field);
    while (std::getline(status, line)) {
        if (line.compare(0, prefix.size(), prefix) == 0) {
            return std::stol(line.substr(prefix.size()));
        }
    }
    return -1;
}

}  // namespace

TEST_CASE("example line simulates one production hour") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    scene.physicsDoc.engine = "builtin";
    std::vector<std::shared_ptr<sim::SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->mech.kind == sim::MechKind::Source) {
            node->mech.prototype.clear();
        }
    }
    const int nodesAtStart = scene.nodeCount();

    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    sim::LogicMachine logic;
    sim::BuiltinRobotSolver robot;
    sim::Simulation simulation(scene, *engine);
    simulation.setSpeed(1);
    simulation.setHooks(
        [&] {
            std::string error;
            for (const sim::LogicBlock& block : scene.logicBlocks) {
                logic.start(scene, block.name, error);
            }
            robot.build(scene, error);
        },
        [&](double dt) {
            logic.step(scene, dt);
            robot.step(dt);
        },
        nullptr);

    simulation.play();
    REQUIRE(simulation.state() == sim::SimState::Playing);
    REQUIRE(simulation.lastError().empty());

    const long rssStart = procKilobytes("VmRSS:");
    const auto started = std::chrono::steady_clock::now();
    long rssMid = rssStart;
    bool sampledMid = false;
    double marked = 0;
    while (simulation.time() < 3600.0) {
        simulation.tick(2.0);
        if (!sampledMid && simulation.time() >= 1800.0) {
            rssMid = procKilobytes("VmRSS:");
            sampledMid = true;
        }
        if (simulation.time() >= marked + 600.0) {
            marked = simulation.time();
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            std::cout << "LINE_HOUR progress sim_s=" << simulation.time() << " wall_s=" << wall << std::endl;
        }
    }
    const double wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const long rssEnd = procKilobytes("VmRSS:");
    const long rssPeak = procKilobytes("VmHWM:");
    const double factor = wallSeconds > 0 ? simulation.time() / wallSeconds : 0;
    std::cout << "LINE_HOUR sim_s=" << simulation.time() << " wall_s=" << wallSeconds << " realtime_factor=" << factor
              << " rss_start_kb=" << rssStart << " rss_mid_kb=" << rssMid << " rss_end_kb=" << rssEnd
              << " rss_peak_kb=" << rssPeak << " nodes=" << scene.nodeCount() << " backend=" << engine->backendName()
              << std::endl;

    REQUIRE(simulation.time() == Approx(3600).margin(sim::Simulation::kFixedDt));
    REQUIRE(sampledMid);
    REQUIRE(scene.nodeCount() == nodesAtStart);
    REQUIRE(rssEnd >= 0);
    REQUIRE(rssEnd - rssStart < 16 * 1024);
}

TEST_CASE("kr420 mesh line simulates one production hour") {
    const std::filesystem::path step =
        std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/KR420_R3080_KR280_R3080.stp";
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
    sim::KinematicResource* robot = sim::findResource(scene.resources, "kr420");
    REQUIRE(robot != nullptr);
    std::vector<std::string> meshIds;
    for (const sim::KinLink& link : robot->links) {
        auto linkNode = scene.find(link.nodeId);
        if (!linkNode) {
            continue;
        }
        int bestCount = -1;
        std::string bestId;
        std::vector<std::shared_ptr<sim::SceneNode>> stack{linkNode};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            const auto mesh = node->meshId.empty() ? nullptr : scene.mesh(node->meshId);
            if (!mesh || mesh->indices.size() < 3) {
                continue;
            }
            const int count = static_cast<int>(mesh->indices.size() / 3);
            if (count > bestCount) {
                bestCount = count;
                bestId = node->id;
            }
        }
        if (!bestId.empty()) {
            meshIds.push_back(bestId);
        }
    }
    int meshBodies = 0;
    sim::Vec3 probePoint{0, 0, 400};
    for (const std::string& id : meshIds) {
        auto node = scene.find(id);
        const auto mesh = node ? scene.mesh(node->meshId) : nullptr;
        if (!node || !mesh || !mesh->bounds.valid) {
            continue;
        }
        node->mech.kind = sim::MechKind::Collision;
        node->mech.shape = sim::CollisionShape::Mesh;
        node->mech.motion = sim::BodyMotion::Anchored;
        node->mech.kinematic = true;
        node->mech.halfExtents = {(mesh->bounds.max.x - mesh->bounds.min.x) * 0.5,
                                  (mesh->bounds.max.y - mesh->bounds.min.y) * 0.5,
                                  (mesh->bounds.max.z - mesh->bounds.min.z) * 0.5};
        probePoint = sim::transformPoint(node->worldMatrix(), mesh->bounds.center());
        ++meshBodies;
    }
    REQUIRE(meshBodies >= 6);

    sim::Mech floor;
    floor.kind = sim::MechKind::Collision;
    floor.motion = sim::BodyMotion::Anchored;
    floor.halfExtents = {4000, 4000, 20};
    auto floorNode = scene.makeNode(sim::NodeKind::Part, "Floor");
    floorNode->local.translation = {0, 0, -20};
    floorNode->mech = floor;
    floorNode->meshId = scene.addMesh(sim::makeBoxMesh(floor.halfExtents));
    scene.addChild(scene.root(), floorNode);

    sim::Mech blank;
    blank.kind = sim::MechKind::Collision;
    blank.motion = sim::BodyMotion::Anchored;
    blank.halfExtents = {40, 40, 40};
    auto prototype = scene.makeNode(sim::NodeKind::Part, "Blank");
    prototype->local.translation = {0, 8000, 80};
    prototype->visible = false;
    prototype->mech = blank;
    prototype->meshId = scene.addMesh(sim::makeBoxMesh(blank.halfExtents));
    scene.addChild(scene.root(), prototype);

    sim::Mech feeder;
    feeder.kind = sim::MechKind::Source;
    feeder.motion = sim::BodyMotion::Anchored;
    feeder.interval = 12;
    feeder.prototype = "Blank";
    feeder.halfExtents = {30, 30, 30};
    auto feederNode = scene.makeNode(sim::NodeKind::Part, "Feeder");
    feederNode->local.translation = {4000, 0, 200};
    feederNode->mech = feeder;
    scene.addChild(scene.root(), feederNode);

    sim::Mech sink;
    sink.kind = sim::MechKind::Sink;
    sink.motion = sim::BodyMotion::Anchored;
    sink.valueSignal = "shipped";
    sink.halfExtents = {200, 200, 200};
    auto sinkNode = scene.makeNode(sim::NodeKind::Part, "Ship");
    sinkNode->local.translation = {4000, 0, 40};
    sinkNode->mech = sink;
    scene.addChild(scene.root(), sinkNode);

    sim::Mech probeMech;
    probeMech.kind = sim::MechKind::Rigid;
    probeMech.motion = sim::BodyMotion::Free;
    probeMech.allowSleep = false;
    probeMech.halfExtents = {25, 25, 25};
    auto probe = scene.makeNode(sim::NodeKind::Part, "Probe");
    probe->local.translation = probePoint;
    probe->mech = probeMech;
    probe->meshId = scene.addMesh(sim::makeBoxMesh(probeMech.halfExtents));
    scene.addChild(scene.root(), probe);

    sim::Station weld;
    weld.id = "st_weld";
    weld.name = "焊接";
    weld.x = 0;
    weld.kind = sim::StationKind::Robot;
    weld.resource = "kr420";
    scene.layout.stations.push_back(weld);
    scene.layout.cyclic = true;
    sim::Operation line;
    line.id = "line";
    line.kind = sim::OpKind::Sequence;
    sim::Operation task;
    task.id = "weld";
    task.parent = "line";
    task.kind = sim::OpKind::Task;
    task.station = "st_weld";
    task.seconds = 12;
    scene.operations = {line, task};
    for (sim::LogicBlock& block : scene.logicBlocks) {
        block.repeat = true;
    }

    sim::addKr420Demonstration(scene);
    const sim::RobotPath* cycle = nullptr;
    for (const sim::RobotPath& path : scene.robotPaths) {
        if (path.name == "cycle") {
            cycle = &path;
        }
    }
    REQUIRE(cycle != nullptr);
    const sim::MotionPlan plan = sim::planRobotPath(scene, *robot, *cycle);
    REQUIRE(plan.seconds > 1.0);
    REQUIRE(!plan.samples.empty());

    sim::LineProduction production;
    REQUIRE(production.start(scene, error));
    auto engine = sim::createPhysicsEngine(sim::PhysicsBackend::Builtin);
    sim::LogicMachine logic;
    sim::Simulation simulation(scene, *engine);
    simulation.setSpeed(1);
    const std::vector<std::vector<double>> samples = plan.samples;
    const double cycleSeconds = plan.seconds;
    simulation.setHooks(
        [&] {
            for (const sim::LogicBlock& block : scene.logicBlocks) {
                std::string ignored;
                logic.start(scene, block.name, ignored);
            }
        },
        [&](double dt) {
            logic.step(scene, dt);
            production.step(dt);
            if (cycleSeconds <= 1e-6) {
                return;
            }
            const double along = std::fmod(simulation.time(), cycleSeconds);
            const double unit = along / cycleSeconds;
            const int index = std::min(static_cast<int>(samples.size()) - 1,
                                       static_cast<int>(unit * static_cast<double>(samples.size() - 1)));
            sim::setJointVector(*robot, samples[static_cast<std::size_t>(std::max(index, 0))]);
            sim::applyResource(scene, *robot);
            if (auto shell = scene.find(meshIds.back())) {
                if (const auto shellMesh = scene.mesh(shell->meshId); shellMesh && shellMesh->bounds.valid) {
                    probe->local.translation = sim::transformPoint(shell->worldMatrix(), shellMesh->bounds.center());
                }
            }
            probe->mech.asleep = false;
        },
        nullptr);
    int meshContacts = 0;
    int meshContactSteps = 0;
    simulation.setAfterStep([&] {
        if (scene.physicsDoc.stats.contacts > 0) {
            ++meshContactSteps;
            meshContacts += scene.physicsDoc.stats.contacts;
        }
    });
    simulation.play();
    REQUIRE(simulation.state() == sim::SimState::Playing);

    const int nodesAtStart = scene.nodeCount();
    const long rssStart = procKilobytes("VmRSS:");
    const auto started = std::chrono::steady_clock::now();
    long rssMid = rssStart;
    bool sampledMid = false;
    double marked = 0;
    while (simulation.time() < 3600.0) {
        simulation.tick(2.0);
        if (!sampledMid && simulation.time() >= 1800.0) {
            rssMid = procKilobytes("VmRSS:");
            sampledMid = true;
        }
        if (simulation.time() >= marked + 300.0) {
            marked = simulation.time();
            const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            std::cerr << "KR420_HOUR progress sim_s=" << simulation.time() << " wall_s=" << wall
                      << " nodes=" << scene.nodeCount() << " contact_steps=" << meshContactSteps << std::endl;
        }
    }
    const double wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const long rssEnd = procKilobytes("VmRSS:");
    const sim::ProductionStats stats = production.stats();
    const sim::Signal* shipped = scene.findSignal("shipped");
    const double factor = wallSeconds > 0 ? simulation.time() / wallSeconds : 0;
    std::cerr << "KR420_HOUR sim_s=" << simulation.time() << " wall_s=" << wallSeconds << " realtime_factor=" << factor
              << " rss_start_kb=" << rssStart << " rss_mid_kb=" << rssMid << " rss_end_kb=" << rssEnd
              << " mesh_bodies=" << meshBodies << " contact_steps=" << meshContactSteps << " contacts=" << meshContacts
              << " output=" << stats.output
              << " jph=" << stats.jobsPerHour
              << " shipped=" << (shipped != nullptr ? shipped->integer : 0) << " nodes=" << scene.nodeCount()
              << std::endl;
    REQUIRE(simulation.time() == Approx(3600).margin(sim::Simulation::kFixedDt));
    REQUIRE(stats.output > 100);
    REQUIRE(shipped != nullptr);
    REQUIRE(shipped->integer > 50);
    REQUIRE(meshContacts > 0);
    REQUIRE(scene.nodeCount() < nodesAtStart + 20);
    REQUIRE(rssEnd - rssStart < 64 * 1024);
}
