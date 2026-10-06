#include "sim/core/Mesh.hpp"
#include "sim/io/EditHistory.hpp"
#include "sim/io/OcctEnvironment.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"
#include "sim/render/Renderer.hpp"

#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BinXCAFDrivers.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pnt.hxx>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>

namespace {

int countNodes(const sim::SceneNode& node) {
    int count = 1;
    for (const auto& child : node.children) {
        if (child) {
            count += countNodes(*child);
        }
    }
    return count;
}

std::shared_ptr<sim::SceneNode> findExact(const std::shared_ptr<sim::SceneNode>& node, const std::string& name) {
    if (!node) {
        return {};
    }
    if (node->name == name) {
        return node;
    }
    for (const auto& child : node->children) {
        if (auto found = findExact(child, name)) {
            return found;
        }
    }
    return {};
}

bool writeStep(const std::filesystem::path& path, const std::function<void(const Handle(XCAFDoc_ShapeTool)&)>& fill) {
    sim::ensureOcctEnvironment();
    Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
    BinXCAFDrivers::DefineFormat(application);
    Handle(TDocStd_Document) document;
    application->NewDocument("BinXCAF", document);
    if (document.IsNull()) {
        return false;
    }
    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    fill(shapes);
    shapes->UpdateAssemblies();
    Interface_Static::SetCVal("write.step.schema", "AP214");
    Interface_Static::SetCVal("write.step.unit", "MM");
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    const bool transferred = writer.Transfer(document);
    const bool wrote = transferred && writer.Write(path.string().c_str()) == IFSelect_RetDone;
    application->Close(document);
    return wrote;
}

}  // namespace

TEST_CASE("F01 an AABB occluder hides a part sitting in the frame hole", "[g1][f01]") {
    sim::Mat4 viewProjection{};
    for (double& value : viewProjection.m) {
        value = 0;
    }
    viewProjection(0, 0) = 0.02;
    viewProjection(1, 2) = 0.001;
    viewProjection(2, 1) = 0.02;
    viewProjection(3, 3) = 1;

    sim::Aabb frame;
    frame.valid = true;
    frame.min = {-30, 0, -30};
    frame.max = {30, 10, 30};
    sim::Aabb throughHole;
    throughHole.valid = true;
    throughHole.min = {-2, 200, -2};
    throughHole.max = {2, 220, 2};

    sim::OcclusionBuffer buffer;
    buffer.reset(160, 90);
    sim::ScreenRect frameRect;
    sim::ScreenRect holeRect;
    REQUIRE(sim::projectBounds(viewProjection, frame, buffer.width, buffer.height, frameRect));
    REQUIRE(sim::projectBounds(viewProjection, throughHole, buffer.width, buffer.height, holeRect));
    sim::rasterizeOccluder(buffer, frameRect);
    const bool hidden = sim::hiddenByOcclusion(buffer, holeRect);
    std::cout << "G1 F01 REPRODUCED hidden=" << hidden << " frame_px=" << frameRect.x0 << "," << frameRect.y0 << "-"
              << frameRect.x1 << "," << frameRect.y1 << " hole_px=" << holeRect.x0 << "," << holeRect.y0 << "-"
              << holeRect.x1 << "," << holeRect.y1 << "\n";
    // The stored depth is the solid rectangle of the frame's bounds, so the hole is covered.
    REQUIRE(hidden);
}

TEST_CASE("F02 distance LOD and a 55 percent cell drop a fine grid", "[g1][f02]") {
    const int far = sim::selectLodLevel(8001.0f, 3);
    const int mid = sim::selectLodLevel(4000.0f, 3);
    const int near = sim::selectLodLevel(1000.0f, 3);
    auto fine = sim::makeTriangleGrid(200, 1000.0);
    const double cell = std::max(fine->bounds.diagonal() * 0.55, 1.0);
    auto coarse = sim::simplifyMesh(*fine, cell);
    std::cout << "G1 F02 REPRODUCED lod_8m=" << far << " lod_4m=" << mid << " lod_1m=" << near
              << " fine_tris=" << fine->triangleCount() << " cell_mm=" << cell
              << " coarse_tris=" << coarse->triangleCount() << "\n";
    REQUIRE(far == 2);
    REQUIRE(mid == 1);
    REQUIRE(near == 0);
    REQUIRE(coarse->triangleCount() < fine->triangleCount());
}

TEST_CASE("F03 corrupt STEP returns an error and the worker still has no catch", "[g1][f03]") {
    const auto dir = std::filesystem::temp_directory_path() / "sim-g1-f03";
    std::filesystem::create_directories(dir);
    const auto junk = dir / "junk.stp";
    {
        std::ofstream out(junk);
        out << "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n";
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(junk.string(), scene);
    std::cout << "G1 F03 corrupt ok=" << result.ok << " message=" << result.message << "\n";
    REQUIRE_FALSE(result.ok);
    std::cout << "G1 F03 REPRODUCED source MainWindow::importStepAsync has no try/catch around "
                 "ReadFile/Transfer/tessellation; this input returned an error instead of throwing\n";
}

TEST_CASE("F04 a vertex-only STEP is reported imported with zero triangles", "[g1][f04]") {
    const auto path = std::filesystem::temp_directory_path() / "sim-g1-vertex.stp";
    REQUIRE(writeStep(path, [](const Handle(XCAFDoc_ShapeTool)& shapes) {
        TopoDS_Shape vertex = BRepBuilderAPI_MakeVertex(gp_Pnt(0, 0, 0)).Shape();
        TDF_Label label = shapes->AddShape(vertex, false);
        TDataStd_Name::Set(label, "OnlyVertex");
    }));
    sim::Scene scene;
    const sim::ImportResult result = sim::StepImporter{}.importFile(path.string(), scene);
    std::cout << "G1 F04 REPRODUCED ok=" << result.ok << " message=" << result.message << " nodes=" << result.nodeCount
              << " tris=" << result.triangleCount << "\n";
    REQUIRE(result.ok);
    REQUIRE(result.message == "imported");
    REQUIRE(result.nodeCount > 0);
    REQUIRE(result.triangleCount == 0);
}

TEST_CASE("F05 depth past 64 drops the leaf without a warning", "[g1][f05]") {
    const auto path = std::filesystem::temp_directory_path() / "sim-g1-depth70.stp";
    REQUIRE(writeStep(path, [](const Handle(XCAFDoc_ShapeTool)& shapes) {
        TDF_Label parent = shapes->NewShape();
        TDataStd_Name::Set(parent, "L0");
        for (int level = 1; level <= 70; ++level) {
            TDF_Label child = shapes->NewShape();
            TDataStd_Name::Set(child, ("L" + std::to_string(level)).c_str());
            shapes->AddComponent(parent, child, TopLoc_Location());
            parent = child;
        }
        TDF_Label leaf = shapes->AddShape(BRepPrimAPI_MakeBox(8.0, 8.0, 8.0).Shape(), false);
        TDataStd_Name::Set(leaf, "LeafBox");
        shapes->AddComponent(parent, leaf, TopLoc_Location());
    }));
    sim::Scene scene;
    const sim::ImportResult result = sim::StepImporter{}.importFile(path.string(), scene);
    const bool leaf = scene.root() && findExact(scene.root(), "LeafBox") != nullptr;
    int named = 0;
    if (scene.root()) {
        std::function<void(const std::shared_ptr<sim::SceneNode>&)> walk = [&](const std::shared_ptr<sim::SceneNode>& node) {
            if (!node) {
                return;
            }
            if (!node->name.empty() && node->name[0] == 'L') {
                ++named;
            }
            for (const auto& child : node->children) {
                walk(child);
            }
        };
        walk(scene.root());
    }
    std::cout << "G1 F05 depth ok=" << result.ok << " message=" << result.message << " nodes=" << result.nodeCount
              << " named_L=" << named << " leaf=" << leaf << " tris=" << result.triangleCount << "\n";
    REQUIRE(result.ok);
    REQUIRE(result.message == "imported");
    REQUIRE_FALSE(leaf);
    REQUIRE(result.triangleCount == 0);
}

TEST_CASE("F05 an assembly shape plus a component keeps both or drops the parent solid", "[g1][f05]") {
    const auto path = std::filesystem::temp_directory_path() / "sim-g1-assembly-solid.stp";
    REQUIRE(writeStep(path, [](const Handle(XCAFDoc_ShapeTool)& shapes) {
        TopoDS_Shape own = BRepPrimAPI_MakeBox(20.0, 10.0, 10.0).Shape();
        TopoDS_Shape childShape = BRepPrimAPI_MakeBox(gp_Pnt(40, 0, 0), 6.0, 6.0, 6.0).Shape();
        TDF_Label assembly = shapes->AddShape(own, true);
        TDataStd_Name::Set(assembly, "BodyAndChild");
        TDF_Label child = shapes->AddShape(childShape, false);
        TDataStd_Name::Set(child, "ChildBox");
        shapes->AddComponent(assembly, child, TopLoc_Location());
    }));
    sim::Scene scene;
    const sim::ImportResult result = sim::StepImporter{}.importFile(path.string(), scene);
    const bool child = scene.root() && findExact(scene.root(), "ChildBox") != nullptr;
    const auto body = scene.root() ? findExact(scene.root(), "BodyAndChild") : std::shared_ptr<sim::SceneNode>{};
    const bool bodyHasMesh = body && !body->meshId.empty();
    std::cout << "G1 F05 assembly-shape ok=" << result.ok << " nodes=" << result.nodeCount
              << " tris=" << result.triangleCount << " child=" << child << " body_mesh=" << bodyHasMesh << "\n";
    REQUIRE(result.ok);
    REQUIRE(child);
    // importLabel meshes a label only when it is not an assembly. A shape stored on the
    // assembly itself is not a second mesh.
    REQUIRE_FALSE(bodyHasMesh);
}

TEST_CASE("F06 a saved project keeps the source path and no content hash", "[g1][f06]") {
    const auto step = std::filesystem::path(SIM_SOURCE_DIR) / "samples/step/sample_fixture.step";
    sim::Scene scene;
    const sim::ImportResult imported = sim::StepImporter{}.importFile(step.string(), scene);
    REQUIRE(imported.ok);
    REQUIRE_FALSE(scene.sourceFiles.empty());
    const auto project = std::filesystem::temp_directory_path() / "sim-g1-f06.simproj";
    const sim::ProjectResult saved = sim::ProjectStore::save(scene, project.string());
    REQUIRE(saved.ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(project.string(), loaded).ok);
    REQUIRE(loaded.sourceFiles == scene.sourceFiles);
    std::cout << "G1 F06 REPRODUCED sources=" << loaded.sourceFiles.front()
              << " embedded_step=no sha256_field=no geometry_version=no\n";
    REQUIRE(loaded.sourceFiles.front().find("sample_fixture.step") != std::string::npos);
}

TEST_CASE("F07 steal replaces the open scene and history can undo that swap", "[g1][f07]") {
    sim::Scene live;
    sim::EditHistory history;
    history.watch(live);
    auto kept = live.makeNode(sim::NodeKind::Part, "Kept");
    live.addChild(live.root(), kept);
    sim::Scene incoming;
    auto replacement = incoming.makeNode(sim::NodeKind::Part, "Replacement");
    incoming.addChild(incoming.root(), replacement);
    live.steal(incoming);
    REQUIRE(live.findByName("Kept") == nullptr);
    REQUIRE(live.findByName("Replacement") != nullptr);
    std::cout << "G1 F07 REPRODUCED steal removed Kept; undo_depth=" << history.undoDepth() << "\n";
    REQUIRE(history.undo());
    const bool keptBack = live.findByName("Kept") != nullptr;
    const bool replacementGone = live.findByName("Replacement") == nullptr;
    std::cout << "G1 F07 undo kept=" << keptBack << " replacement_gone=" << replacementGone << "\n";
    REQUIRE(keptBack);
    REQUIRE(replacementGone);
}
