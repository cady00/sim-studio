#include "sim/core/Mesh.hpp"
#include "sim/io/AssetImport.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/render/Renderer.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <vector>

using Catch::Approx;

TEST_CASE("asset registry reads stl ply obj urdf image and leaves a jt slot", "[assets]") {
    const auto registry = sim::createBuiltinRegistry();
    const std::vector<std::string> formats = registry->formats();
    REQUIRE(std::find(formats.begin(), formats.end(), "jt") != formats.end());
    REQUIRE(std::find(formats.begin(), formats.end(), "step") != formats.end());
    REQUIRE(std::find(formats.begin(), formats.end(), "iges") != formats.end());

    const auto dir = std::filesystem::temp_directory_path();
    const auto stlPath = dir / "simstudio_tri.stl";
    {
        std::ofstream out(stlPath);
        out << "solid tri\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 10 0 0\nvertex 0 5 0\nendloop\nendfacet\nendsolid tri\n";
    }
    sim::Scene stl;
    const sim::AssetImportResult stlResult = registry->importPath(stlPath.string(), stl);
    REQUIRE(stlResult.ok);
    REQUIRE(stlResult.format == "stl");
    REQUIRE(stlResult.triangleCount == 1);

    const auto plyPath = dir / "simstudio_tri.ply";
    {
        std::ofstream out(plyPath);
        out << "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\n"
               "element face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0\n8 0 0\n0 4 0\n3 0 1 2\n";
    }
    sim::Scene ply;
    const sim::AssetImportResult plyResult = registry->importPath(plyPath.string(), ply);
    REQUIRE(plyResult.ok);
    REQUIRE(plyResult.triangleCount == 1);

    const auto objPath = dir / "simstudio_asset.obj";
    {
        std::ofstream out(objPath);
        out << "o Panel\nv 0 0 0\nv 4 0 0\nv 4 2 0\nf 1 2 3\n";
    }
    sim::Scene obj;
    REQUIRE(registry->importPath(objPath.string(), obj).triangleCount == 1);

    const auto urdfPath = dir / "simstudio_arm.urdf";
    {
        std::ofstream out(urdfPath);
        out << R"(<robot name="arm">
<link name="base"><visual><geometry><box size="0.1 0.2 0.3"/></geometry></visual></link>
<link name="tool"><visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual></link>
<joint name="j1" type="revolute">
<parent link="base"/><child link="tool"/>
<origin xyz="0 0 0.15" rpy="0 0 0"/>
<axis xyz="0 0 1"/>
<limit lower="-1" upper="1" velocity="1"/>
</joint>
</robot>)";
    }
    sim::Scene urdf;
    const sim::AssetImportResult urdfResult = registry->importPath(urdfPath.string(), urdf);
    REQUIRE(urdfResult.ok);
    REQUIRE(urdf.resources.size() == 1);
    REQUIRE(urdf.resources.front().links.size() == 2);
    REQUIRE(urdf.resources.front().joints.size() == 1);
    REQUIRE(urdf.resources.front().joints.front().motion == sim::JointMotion::Revolute);
    REQUIRE(urdfResult.triangleCount == 24);

    const auto pngPath = dir / "simstudio_plan.png";
    {
        std::ofstream out(pngPath, std::ios::binary);
        const unsigned char magic[] = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a, 1, 2, 3, 4};
        out.write(reinterpret_cast<const char*>(magic), sizeof(magic));
    }
    sim::Scene image;
    REQUIRE(registry->importPath(pngPath.string(), image).ok);
    REQUIRE(image.layout.underlay.kind == "image");
    REQUIRE(image.layout.underlay.bytes.size() == 12);
    const auto project = dir / "simstudio_image.simproj";
    REQUIRE(sim::ProjectStore::save(image, project.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(project.string(), loaded).ok);
    REQUIRE(loaded.layout.underlay.bytes == image.layout.underlay.bytes);

    const auto jtPath = dir / "simstudio_empty.jt";
    {
        std::ofstream out(jtPath, std::ios::binary);
        out << "JT placeholder";
    }
    sim::Scene jt;
    const sim::AssetImportResult jtResult = registry->importPath(jtPath.string(), jt);
    REQUIRE_FALSE(jtResult.ok);
    REQUIRE(jtResult.message.find("JT Open Toolkit") != std::string::npos);
    REQUIRE(jt.root()->children.empty());
}

TEST_CASE("stl export counts triangles and occt samples round trip", "[assets]") {
    sim::Scene scene;
    auto node = scene.makeNode(sim::NodeKind::Part, "box");
    node->meshId = scene.addMesh(sim::makeBoxMesh({5, 5, 5}));
    scene.addChild(scene.root(), node);
    const auto stlPath = std::filesystem::temp_directory_path() / "simstudio_box.stl";
    std::string error;
    REQUIRE(sim::exportSceneStl(scene, stlPath.string(), error));
    std::ifstream in(stlPath, std::ios::binary);
    in.seekg(80);
    std::uint32_t triangles = 0;
    in.read(reinterpret_cast<char*>(&triangles), 4);
    REQUIRE(triangles == 12);

    const auto registry = sim::createBuiltinRegistry();
    for (const char* format : {"step", "iges", "brep"}) {
        const auto path = std::filesystem::temp_directory_path() / (std::string("simstudio_sample.") + format);
        REQUIRE(sim::writeSampleShape(format, path.string(), error));
        sim::Scene imported;
        const sim::AssetImportResult result = registry->importPath(path.string(), imported);
        REQUIRE(result.ok);
        REQUIRE(result.triangleCount > 0);
    }
}

TEST_CASE("measured times follow the planner logic and a conveyor", "[assets]") {
    sim::Scene scene;
    sim::KinematicResource resource;
    resource.name = "axis";
    resource.ik = "none";
    sim::KinJoint joint;
    joint.name = "J1";
    joint.lower = -40;
    joint.upper = 40;
    joint.maxSpeed = 120;
    joint.maxAccel = 400;
    joint.externalName = "E1";
    resource.joints.push_back(joint);
    scene.resources.push_back(resource);

    sim::RobotPath path;
    path.name = "turn";
    path.resource = "axis";
    sim::PathPoint out;
    out.move = sim::PathMove::Ptp;
    out.joints = {40};
    out.externalValues = {40};
    out.speed = 80;
    out.zone = 0;
    sim::PathPoint back;
    back.move = sim::PathMove::Ptp;
    back.joints = {-20};
    back.externalValues = {-20};
    back.speed = 80;
    path.points = {out, back};
    scene.robotPaths.push_back(path);

    sim::LogicBlock block;
    block.name = "hold";
    sim::LogicAction action;
    action.kind = "wait";
    action.seconds = 2;
    sim::LogicStep step;
    step.actions.push_back(action);
    block.steps.push_back(step);
    scene.logicBlocks.push_back(block);

    sim::Station belt;
    belt.id = "belt";
    belt.nodeName = "belt";
    scene.layout.stations.push_back(belt);
    auto beltNode = scene.makeNode(sim::NodeKind::Part, "belt");
    beltNode->mech.kind = sim::MechKind::Conveyor;
    beltNode->mech.speed = 200;
    beltNode->mech.direction = {1, 0, 0};
    scene.addChild(scene.root(), beltNode);

    sim::Operation planned;
    planned.id = "move";
    planned.kind = sim::OpKind::Task;
    planned.path = "turn";
    planned.seconds = 1;
    sim::Operation waited;
    waited.id = "wait";
    waited.kind = sim::OpKind::Task;
    waited.logic = "hold";
    waited.seconds = 9;
    sim::Operation carried;
    carried.id = "carry";
    carried.kind = sim::OpKind::Task;
    carried.station = "belt";
    carried.seconds = 9;
    scene.operations = {planned, waited, carried};

    const sim::MeasureResult measured = sim::measureOperationTimes(scene, true);
    REQUIRE(measured.planned == 1);
    REQUIRE(measured.logic == 1);
    REQUIRE(measured.conveyor == 1);
    REQUIRE(scene.operations[0].timeSource == "plan");
    REQUIRE(scene.operations[1].seconds == Approx(2.0).margin(0.05));
    REQUIRE(scene.operations[1].timeSource == "logic");
    REQUIRE(scene.operations[2].timeSource == "conveyor");
    REQUIRE(scene.operations[2].seconds == Approx(2.5).margin(0.08));

    const double first = scene.operations[0].seconds;
    scene.operations[0].timeSource = "pinned";
    scene.operations[0].timeKey = "pinned";
    scene.operations[0].seconds = 12;
    sim::measureOperationTimes(scene, false);
    REQUIRE(scene.operations[0].seconds == Approx(12));
    scene.resources.front().joints.front().maxSpeed = 30;
    sim::measureOperationTimes(scene, false);
    REQUIRE(scene.operations[0].timeSource == "plan");
    REQUIRE(scene.operations[0].seconds > first);

    scene.robotPaths.front().points.front().zone = 25;
    const std::string zoned = sim::pathFingerprint(scene.robotPaths.front(), &scene.resources.front());
    scene.robotPaths.front().points.front().zone = 0;
    REQUIRE(zoned != sim::pathFingerprint(scene.robotPaths.front(), &scene.resources.front()));
}

TEST_CASE("frustum culling and mesh lod drop work at field scale", "[assets]") {
    const sim::Mat4 viewProjection = sim::Mat4::identity();
    int outside = 0;
    for (int i = 0; i < 100000; ++i) {
        sim::Aabb box;
        const double x = static_cast<double>(i) * 10.0;
        box.expand({x, 0, 0});
        box.expand({x + 1, 1, 1});
        if (!sim::intersectsClipFrustum(viewProjection, box)) {
            ++outside;
        }
    }
    REQUIRE(outside > 90000);

    const auto fine = sim::makeTriangleGrid(256, 40);
    const auto coarse = sim::simplifyMesh(*fine, 20);
    REQUIRE(fine->triangleCount() >= 256);
    REQUIRE(coarse->triangleCount() < fine->triangleCount());
    REQUIRE(sim::selectLodLevel(1000, 3) == 0);
    REQUIRE(sim::selectLodLevel(5000, 3) == 1);
    REQUIRE(sim::selectLodLevel(9000, 3) == 2);
}
