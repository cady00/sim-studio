#include "sim/core/Mesh.hpp"
#include "sim/core/Weld.hpp"
#include "sim/io/ProjectStore.hpp"
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
