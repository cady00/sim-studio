#include "sim/core/Mesh.hpp"
#include "sim/core/Weld.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/motion/Envelope.hpp"
#include "sim/motion/WeldPlan.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

namespace {

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

}  // namespace

TEST_CASE("weld csv and xml keep layers and the electrode group") {
    std::string error;
    std::vector<sim::WeldSpot> csvSpots;
    const char* csv = "id,x,y,z,nx,ny,nz,layers,electrode,example\n"
                      "w1,10,20,30,0,0,1,panel_a|panel_b,gun_a,示例\n";
    REQUIRE(sim::importWeldCsv(csv, csvSpots, error));
    REQUIRE(csvSpots.size() == 1);
    REQUIRE(csvSpots[0].id == "w1");
    REQUIRE(csvSpots[0].position.x == Catch::Approx(10));
    REQUIRE(csvSpots[0].layers.size() == 2);
    REQUIRE(csvSpots[0].layers[1] == "panel_b");
    REQUIRE(csvSpots[0].electrode == "gun_a");
    REQUIRE(csvSpots[0].example);

    std::vector<sim::WeldSpot> xmlSpots;
    const char* xml = "<welds><spot id=\"w2\" electrode=\"gun_b\" example=\"1\">"
                      "<point x=\"1\" y=\"2\" z=\"3\"/><normal x=\"0\" y=\"1\" z=\"0\"/>"
                      "<layer>outer</layer><layer>inner</layer></spot></welds>";
    REQUIRE(sim::importWeldXml(xml, xmlSpots, error));
    REQUIRE(xmlSpots.size() == 1);
    REQUIRE(xmlSpots[0].electrode == "gun_b");
    REQUIRE(xmlSpots[0].normal.y == Catch::Approx(1));
    REQUIRE(xmlSpots[0].layers.size() == 2);
    REQUIRE(xmlSpots[0].example);
}

TEST_CASE("a weld point projects onto the sheet and the gun aligns around the normal") {
    sim::Scene scene;
    auto sheet = scene.makeNode(sim::NodeKind::Part, "panel_a");
    sheet->meshId = scene.addMesh(sim::makeBoxMesh({100, 80, 5}));
    sheet->local.translation = {0, 0, 5};
    scene.addChild(scene.root(), sheet);

    std::string error;
    REQUIRE(sim::importWeldCsv("w1,20,10,80,0,0,1,panel_a,gun_a,1\n", scene.welds, error));
    REQUIRE(sim::projectWeldSpots(scene) == 1);
    REQUIRE(scene.welds[0].onSurface);
    REQUIRE(scene.welds[0].position.z == Catch::Approx(10).margin(0.2));
    REQUIRE(scene.welds[0].position.x == Catch::Approx(20).margin(0.2));
    REQUIRE(scene.welds[0].normal.z == Catch::Approx(1).margin(0.05));

    sim::KinematicResource robot = wristResource();
    std::vector<double> seed = {15, -40, 55, 10, 40, -20};
    sim::setJointVector(robot, seed);
    const sim::CartesianPose posed = sim::forwardModel(robot);
    sim::WeldSpot spot;
    spot.position = posed.position;
    spot.normal = -(posed.rotation.rotate({0, 0, 1}));
    const sim::SpinSearch aligned = sim::searchWeldSpin(nullptr, robot, spot, 24, false);
    REQUIRE(aligned.ok);
    REQUIRE_FALSE(aligned.singular);
    sim::setJointVector(robot, aligned.joints);
    const sim::CartesianPose tcp = sim::forwardModel(robot);
    REQUIRE(sim::length(tcp.position - spot.position) < 1.5);
    const sim::Vec3 toolZ = tcp.rotation.rotate({0, 0, 1});
    REQUIRE(sim::dot(toolZ, -spot.normal) > 0.98);

    const sim::CartesianPose canonical = sim::weldTcpPose(spot, 0);
    REQUIRE(sim::dot(canonical.rotation.rotate({0, 0, 1}), -spot.normal) > 0.999);

    scene.welds[0].reachable = true;
    const auto saved = sim::ProjectStore::save(scene, "/tmp/simstudio_welds.simproj");
    REQUIRE(saved.ok);
    sim::Scene loaded;
    const auto opened = sim::ProjectStore::load("/tmp/simstudio_welds.simproj", loaded);
    REQUIRE(opened.ok);
    REQUIRE(loaded.welds.size() == 1);
    REQUIRE(loaded.welds[0].electrode == "gun_a");
    REQUIRE(loaded.welds[0].onSurface);
    REQUIRE(loaded.welds[0].reachable);
}

TEST_CASE("example weld spots are marked and land on a plate") {
    sim::Scene scene;
    sim::prepareExampleWelds(scene);
    REQUIRE(scene.welds.size() >= 2);
    int onPlate = 0;
    int reachable = 0;
    for (const sim::WeldSpot& spot : scene.welds) {
        REQUIRE(spot.example);
        REQUIRE(spot.electrode == "gun_a");
        REQUIRE(spot.layers.size() == 2);
        if (spot.onSurface) {
            ++onPlate;
        }
        if (spot.reachable) {
            ++reachable;
        }
    }
    REQUIRE(onPlate == static_cast<int>(scene.welds.size()));
    REQUIRE(reachable >= 2);
    REQUIRE(scene.findByName("WeldPlate"));
}

TEST_CASE("reachability, base coverage, weld order, and rrt avoid a joint slab") {
    sim::KinematicResource robot = wristResource();
    sim::setJointVector(robot, {15, -40, 55, 10, 40, -20});
    const sim::CartesianPose posed = sim::forwardModel(robot);
    sim::WeldSpot near;
    near.id = "near";
    near.position = posed.position;
    near.normal = -(posed.rotation.rotate({0, 0, 1}));
    sim::WeldSpot far = near;
    far.id = "far";
    far.position = {8000, 8000, 8000};
    std::vector<sim::WeldSpot> spots{near, far};
    const std::vector<sim::SpotReach> reach = sim::checkWeldReach(nullptr, robot, spots, false);
    REQUIRE(reach.size() == 2);
    REQUIRE(reach[0].reachable);
    REQUIRE(reach[0].margin > 0.02);
    REQUIRE_FALSE(reach[0].singular);
    REQUIRE_FALSE(reach[1].reachable);

    const sim::BaseRegion region = sim::optimizeRobotBase(robot, std::vector<sim::WeldSpot>{near}, 1500, 3);
    REQUIRE(region.found);
    REQUIRE(region.minX <= 0);
    REQUIRE(region.maxX >= 0);
    bool farMiss = false;
    for (const sim::BaseSample& sample : region.samples) {
        if (std::abs(sample.x) > 1000 && !sample.coversAll) {
            farMiss = true;
        }
    }
    REQUIRE(farMiss);

    const std::vector<sim::Vec3> square{{0, 0, 0}, {0, 10, 0}, {10, 10, 0}, {10, 0, 0}};
    const std::vector<int> order = sim::orderWeldSpots(square);
    REQUIRE(order.size() == 4);
    double tour = 0;
    for (std::size_t i = 1; i < order.size(); ++i) {
        tour += sim::length(square[static_cast<std::size_t>(order[i])] - square[static_cast<std::size_t>(order[i - 1])]);
    }
    REQUIRE(tour == Catch::Approx(30));

    const std::vector<double> start{0, -30, 40, 0, 30, 0};
    const std::vector<double> goal{80, -30, 40, 0, 30, 0};
    const auto blocked = [](const std::vector<double>& q) {
        return q.size() > 1 && q[0] > 25 && q[0] < 55 && std::abs(q[1] + 30) < 12;
    };
    sim::RrtOptions options;
    options.iterations = 500;
    options.stepDeg = 8;
    options.seed = 3;
    const sim::RrtResult planned = sim::planRrtConnect(robot, start, goal, blocked, options);
    REQUIRE(planned.ok);
    REQUIRE(planned.joints.size() >= 3);
    for (const std::vector<double>& sample : planned.joints) {
        REQUIRE_FALSE(blocked(sample));
    }
    bool detoured = false;
    for (const std::vector<double>& sample : planned.joints) {
        if (std::abs(sample[1] + 30) > 12) {
            detoured = true;
        }
    }
    REQUIRE(detoured);
    const std::vector<std::vector<double>> smooth = sim::smoothJointPath(planned.joints, blocked, 6);
    REQUIRE(smooth.size() >= 2);
    REQUIRE(smooth.size() <= planned.joints.size());
    for (const std::vector<double>& sample : smooth) {
        REQUIRE_FALSE(blocked(sample));
    }
}

TEST_CASE("the tcp sweep becomes a collision set and two robots interlock") {
    sim::Scene scene;
    sim::KinematicResource arm = wristResource();
    arm.name = "arm_a";
    sim::setJointVector(arm, {15, -40, 55, 10, 40, -20});
    sim::KinematicResource mate = arm;
    mate.name = "arm_b";
    scene.resources.push_back(arm);
    scene.resources.push_back(mate);
    const std::vector<std::vector<double>> joints{{15, -40, 55, 10, 40, -20}, {50, -25, 35, 0, 25, 10}};
    const sim::MotionEnvelope envelope = sim::buildTcpEnvelope(scene, scene.resources.front(), joints, "sweep", 40);
    REQUIRE(envelope.samples.size() == 2);
    REQUIRE_FALSE(envelope.nodes.empty());
    REQUIRE(scene.findByName(envelope.nodes.front()));
    REQUIRE(envelope.bounds.min.x <= envelope.samples[0].x);
    REQUIRE(envelope.bounds.max.x >= envelope.samples[0].x);
    bool listed = false;
    for (const sim::CollisionSet& set : scene.proximity.sets) {
        if (set.name == "sweep" && !set.objects.empty()) {
            listed = true;
        }
    }
    REQUIRE(listed);

    sim::InterlockZone zone;
    zone.name = "cell";
    zone.center = sim::forwardModel(scene.resources[0]).position;
    zone.radius = 40;
    zone.resources = {"arm_a", "arm_b"};
    scene.interlocks.push_back(zone);
    const std::vector<sim::InterlockHit> together = sim::evaluateInterlocks(scene);
    REQUIRE(together.size() == 1);
    REQUIRE(together[0].a == "arm_a");
    REQUIRE(together[0].b == "arm_b");
    REQUIRE(scene.findSignal("interlock_cell"));
    REQUIRE(scene.findSignal("interlock_cell")->flag);

    scene.resources[1].joints[0].position = 120;
    const std::vector<sim::InterlockHit> apart = sim::evaluateInterlocks(scene);
    REQUIRE(apart.empty());
    REQUIRE_FALSE(scene.findSignal("interlock_cell")->flag);
}
