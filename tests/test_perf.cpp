#include "sim/core/DemoScene.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/physics/PhysicsEngine.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
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
