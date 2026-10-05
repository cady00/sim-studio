#include "sim/io/ProjectStore.hpp"
#include "sim/io/StepImporter.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>

using Catch::Approx;

namespace {

std::shared_ptr<sim::SceneNode> findName(const std::shared_ptr<sim::SceneNode>& node, const std::string& name) {
    if (!node) {
        return {};
    }
    if (node->name == name || node->name.find(name) != std::string::npos) {
        return node;
    }
    for (const auto& child : node->children) {
        if (auto found = findName(child, name)) {
            return found;
        }
    }
    return {};
}

void printTree(const sim::SceneNode& node, int depth) {
    std::cout << std::string(static_cast<std::size_t>(depth) * 2, ' ') << node.name << " children=" << node.children.size()
              << " t=(" << node.local.translation.x << "," << node.local.translation.y << "," << node.local.translation.z
              << ")\n";
    for (const auto& child : node.children) {
        if (child) {
            printTree(*child, depth + 1);
        }
    }
}

}  // namespace

TEST_CASE("sample fixture STEP keeps the assembly and shared pin mesh") {
    const std::filesystem::path path = std::filesystem::path(SIM_SOURCE_DIR) / "samples/step/sample_fixture.step";
    REQUIRE(std::filesystem::exists(path));
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    INFO(result.message);
    REQUIRE(result.ok);
    REQUIRE(result.triangleCount == 136);
    REQUIRE(result.fileProductCount == 3);
    REQUIRE(result.fileOccurrenceCount == 3);
    REQUIRE(result.fileSolidCount == 2);
    REQUIRE(result.partCount == 3);
    REQUIRE(result.assemblyCount == 1);
    REQUIRE(result.emptyPartCount == 0);
    REQUIRE(result.nodeCount == 4);
    REQUIRE(result.warnings.empty());
    REQUIRE(result.report.find("AP214") != std::string::npos);
    const auto fixture = findName(scene.root(), "Fixture");
    REQUIRE(fixture);
    REQUIRE(fixture->kind == sim::NodeKind::Assembly);
    const auto base = findName(fixture, "BasePlate");
    const auto pinA = findName(fixture, "Pin_A");
    const auto pinB = findName(fixture, "Pin_B");
    REQUIRE(base);
    REQUIRE(pinA);
    REQUIRE(pinB);
    REQUIRE(pinA->meshId == pinB->meshId);
    const auto pinMesh = scene.mesh(pinA->meshId);
    REQUIRE(pinMesh);
    REQUIRE(pinMesh->triangleCount() == 124);
    const auto baseMesh = scene.mesh(base->meshId);
    REQUIRE(baseMesh);
    REQUIRE(baseMesh->bounds.max.x == Approx(120).margin(2));
    REQUIRE(baseMesh->bounds.max.y == Approx(80).margin(2));
    REQUIRE(baseMesh->bounds.max.z == Approx(12).margin(2));
    REQUIRE(pinA->local.translation.x == Approx(30).margin(0.5));
    REQUIRE(pinA->local.translation.y == Approx(25).margin(0.5));
    REQUIRE(pinA->local.translation.z == Approx(12).margin(0.5));

    const auto project = std::filesystem::temp_directory_path() / "simstudio_fixture.simproj";
    REQUIRE(sim::ProjectStore::save(scene, project.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(project.string(), loaded).ok);
    REQUIRE(findName(loaded.root(), "Pin_B"));
    REQUIRE(loaded.meshes().size() == scene.meshes().size());
    std::filesystem::remove(project);
}

TEST_CASE("optional local robot STEP exposes the NX assembly") {
    const char* env = std::getenv("SIM_LOCAL_STEP");
    if (env == nullptr || env[0] == '\0') {
        SUCCEED("SIM_LOCAL_STEP is not set; vendor model stays outside the repo");
        return;
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(env, scene, [](const sim::ImportProgress& progress) {
        std::cout << progress.stageKey << " " << progress.percent << " " << progress.current << "/" << progress.total
                  << "\n";
    }, nullptr);
    std::cout << "local import nodes=" << result.nodeCount << " meshes=" << result.meshCount
              << " triangles=" << result.triangleCount << " ms=" << result.elapsedMs << "\n";
    if (scene.root()) {
        printTree(*scene.root(), 0);
    }
    INFO(result.message);
    REQUIRE(result.ok);
    const auto rootProduct = findName(scene.root(), "394.088-10.104.00");
    REQUIRE(rootProduct);
    REQUIRE(rootProduct->kind == sim::NodeKind::Assembly);
    REQUIRE(findName(rootProduct, "394.088-10.105.10"));
    REQUIRE(findName(rootProduct, "394.088-10.104.20"));
    REQUIRE(findName(rootProduct, "394.088-10.105.30"));
    REQUIRE(findName(rootProduct, "394.088-10.105.40"));
    REQUIRE(findName(rootProduct, "394.088-10.104.50"));
    REQUIRE(result.triangleCount > 1000);
}

TEST_CASE("nx prt teaching parts are not step files", "[mcd]") {
    const auto root = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/mcd";
    const auto lift = root / "cylinder" / "气缸举升.prt";
    const auto blank = root / "stamping" / "工件.prt";
    const auto cup = root / "suction" / "使用吸盘吸取产品.prt";
    const auto eye = root / "stamping" / "光电传感器.prt";
    if (!std::filesystem::exists(lift) || !std::filesystem::exists(cup)) {
        SKIP("testdata/local/mcd is not present");
    }
    auto head = [](const std::filesystem::path& path) {
        std::ifstream input(path, std::ios::binary);
        std::string bytes(16, '\0');
        input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        bytes.resize(static_cast<std::size_t>(input.gcount()));
        return bytes;
    };
    REQUIRE(head(lift).rfind("SPLMSSTR", 0) == 0);
    REQUIRE(head(cup).rfind("SPLMSSTR", 0) == 0);
    const std::string compound = head(blank);
    REQUIRE(static_cast<unsigned char>(compound[0]) == 0xD0);
    REQUIRE(static_cast<unsigned char>(compound[1]) == 0xCF);
    const std::string sensor = head(eye);
    REQUIRE(sensor.find("ISO-10303-21") == std::string::npos);
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(cup.string(), scene);
    REQUIRE_FALSE(result.ok);
    REQUIRE(result.triangleCount == 0);
    REQUIRE(result.meshCount == 0);
}

TEST_CASE("step header dollar and a placement-only product stay visible") {
    const auto source = std::filesystem::path(SIM_SOURCE_DIR) / "samples/step/sample_fixture.step";
    std::ifstream input(source);
    REQUIRE(input);
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    const auto header = text.find(",'Unknown');");
    REQUIRE(header != std::string::npos);
    text.replace(header, std::string(",'Unknown');").size(), ",$);");
    const auto end = text.rfind("ENDSEC;");
    REQUIRE(end != std::string::npos);
    text.insert(end, R"(#600=SHAPE_REPRESENTATION('GhostBearing',(#601),#27);
#601=AXIS2_PLACEMENT_3D('',#12,#13,#14);
#602=SHAPE_DEFINITION_REPRESENTATION(#603,#600);
#603=PRODUCT_DEFINITION_SHAPE('','',#604);
#604=PRODUCT_DEFINITION('design','',#605,#9);
#605=PRODUCT_DEFINITION_FORMATION('','',#606);
#606=PRODUCT('GhostBearing','GhostBearing','',(#8));
#607=NEXT_ASSEMBLY_USAGE_OCCURRENCE('4','GhostBearing','',#5,#604,$);
#608=PRODUCT_DEFINITION_SHAPE('Placement','Placement of an item',#607);
#609=ITEM_DEFINED_TRANSFORMATION('','',#11,#601);
#610=( REPRESENTATION_RELATIONSHIP('','',#600,#10) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#609) SHAPE_REPRESENTATION_RELATIONSHIP() );
#611=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#610,#608);
#612=PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#606));
)");
    const auto path = std::filesystem::temp_directory_path() / "simstudio_step_gap.step";
    {
        std::ofstream output(path);
        output << text;
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    std::filesystem::remove(path);
    INFO(result.report);
    for (const std::string& warning : result.warnings) {
        INFO(warning);
    }
    REQUIRE(result.ok);
    REQUIRE(result.triangleCount == 136);
    REQUIRE(result.fileProductCount == 4);
    REQUIRE(result.fileOccurrenceCount == 4);
    REQUIRE(result.fileSolidCount == 2);
    REQUIRE(result.emptyPartCount == 1);
    const auto ghost = findName(scene.root(), "GhostBearing");
    REQUIRE(ghost);
    REQUIRE(ghost->kind == sim::NodeKind::Part);
    REQUIRE(ghost->meshId.empty());
    REQUIRE(findName(scene.root(), "Pin_A")->meshId == findName(scene.root(), "Pin_B")->meshId);
    bool named = false;
    bool headerWarning = false;
    for (const std::string& warning : result.warnings) {
        if (warning.find("GhostBearing") != std::string::npos && warning.find("没有面") != std::string::npos) {
            named = true;
        }
        if (warning.find("authorisation") != std::string::npos) {
            headerWarning = true;
        }
    }
    REQUIRE(named);
    REQUIRE(headerWarning);
}

TEST_CASE("cartesian gantry step imports a real mesh", "[mcd]") {
    const auto path = std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/mcd/palletizer" / "Cartesian Gantry Palletizing.stp";
    if (!std::filesystem::exists(path)) {
        SKIP("testdata/local/mcd gantry step is not present");
    }
    sim::Scene scene;
    sim::StepImporter importer;
    const sim::ImportResult result = importer.importFile(path.string(), scene);
    std::cout << "gantry " << result.report << " ms=" << result.elapsedMs << "\n";
    for (const std::string& warning : result.warnings) {
        std::cout << "warn " << warning << "\n";
    }
    INFO(result.report);
    REQUIRE(result.ok);
    REQUIRE(result.fileProductCount == 118);
    REQUIRE(result.fileOccurrenceCount == 323);
    REQUIRE(result.fileSolidCount == 106);
    REQUIRE(result.nodeCount == 696);
    REQUIRE(result.partCount == 590);
    REQUIRE(result.assemblyCount == 106);
    REQUIRE(result.meshCount == 106);
    REQUIRE(result.triangleCount == 289932);
    REQUIRE(result.emptyPartCount == 4);
    REQUIRE(scene.nodeCount() == result.nodeCount + 1);
    REQUIRE(scene.root()->children.size() == 1);
    const auto gantry = scene.root()->children.front();
    REQUIRE(gantry->name == "Cartesian Gantry Palletizing");
    REQUIRE(gantry->kind == sim::NodeKind::Assembly);
    REQUIRE(gantry->children.size() == 42);
    const auto frame = findName(gantry, "99MM2018A_E101");
    const auto camera = findName(gantry, "Camera Inspection");
    const auto conveyor = findName(gantry, "pick up cnv 2");
    REQUIRE(frame);
    REQUIRE(camera);
    REQUIRE(conveyor);
    REQUIRE(frame->children.size() == 59);
    REQUIRE(camera->children.size() == 8);
    REQUIRE(conveyor->children.size() == 107);
    std::function<std::shared_ptr<sim::SceneNode>(const std::shared_ptr<sim::SceneNode>&)> exactPallet =
        [&](const std::shared_ptr<sim::SceneNode>& node) -> std::shared_ptr<sim::SceneNode> {
        if (!node) {
            return {};
        }
        if (node->name == "Pallet") {
            return node;
        }
        for (const auto& child : node->children) {
            if (auto found = exactPallet(child)) {
                return found;
            }
        }
        return {};
    };
    const auto pallet = exactPallet(gantry);
    REQUIRE(pallet);
    REQUIRE(pallet->local.translation.x == Approx(743.282).margin(0.01));
    REQUIRE(pallet->local.translation.y == Approx(-2411.961).margin(0.01));
    REQUIRE(pallet->local.translation.z == Approx(2107.0).margin(0.01));
    int empty = 0;
    std::function<void(const std::shared_ptr<sim::SceneNode>&)> walk = [&](const std::shared_ptr<sim::SceneNode>& node) {
        if (!node) {
            return;
        }
        if (node->kind == sim::NodeKind::Part && node->meshId.empty()) {
            ++empty;
            REQUIRE(node->name == "BGHFB6206ZZ-80");
        }
        for (const auto& child : node->children) {
            walk(child);
        }
    };
    walk(gantry);
    REQUIRE(empty == 4);
    bool bearing = false;
    bool headerWarning = false;
    for (const std::string& warning : result.warnings) {
        if (warning.find("BGHFB6206ZZ-80") != std::string::npos && warning.find("×4") != std::string::npos) {
            bearing = true;
        }
        if (warning.find("authorisation") != std::string::npos) {
            headerWarning = true;
        }
    }
    REQUIRE(bearing);
    REQUIRE(headerWarning);
    REQUIRE(result.report.find("AP214") != std::string::npos);
}
