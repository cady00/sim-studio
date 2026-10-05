#include "sim/ai/Commands.hpp"
#include "sim/core/Scene.hpp"
#include "sim/io/EditHistory.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;

namespace {

sim::SceneNode* partNamed(sim::Scene& scene, const char* name) {
    auto node = scene.findByName(name);
    REQUIRE(node);
    return node.get();
}

}  // namespace

TEST_CASE("undo restores a transform, a weld, and a signal") {
    sim::Scene scene;
    auto part = scene.makeNode(sim::NodeKind::Part, "Plate");
    scene.addChild(scene.root(), part);

    sim::EditHistory history;
    history.watch(scene);
    REQUIRE(history.undoDepth() == 0);

    part->local.translation = {10, 0, 0};
    scene.notify(sim::ChangeKind::Transform);
    REQUIRE(history.undoDepth() == 1);

    sim::WeldSpot spot;
    spot.id = "w1";
    spot.position = {1, 2, 3};
    scene.welds.push_back(spot);
    scene.notify(sim::ChangeKind::Structure);

    scene.upsertSignal("go", sim::SignalType::Bool);
    scene.findSignal("go")->flag = true;
    scene.notify(sim::ChangeKind::Structure);
    REQUIRE(history.undoDepth() == 3);

    scene.notify(sim::ChangeKind::Selection);
    REQUIRE(history.undoDepth() == 3);

    REQUIRE(history.undo());
    REQUIRE(scene.findSignal("go") == nullptr);
    REQUIRE(scene.welds.size() == 1);
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(10));

    REQUIRE(history.undo());
    REQUIRE(scene.welds.empty());
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(10));

    REQUIRE(history.undo());
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(0));
    REQUIRE(history.redoDepth() == 3);

    REQUIRE(history.redo());
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(10));
    REQUIRE(scene.welds.empty());

    partNamed(scene, "Plate")->local.translation.x = 4;
    scene.notify(sim::ChangeKind::Transform);
    REQUIRE(history.redoDepth() == 0);
    REQUIRE(history.undo());
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(10));
}

TEST_CASE("undo ignores simulation ticks and caps the stack") {
    sim::Scene scene;
    auto part = scene.makeNode(sim::NodeKind::Part, "Plate");
    scene.addChild(scene.root(), part);
    sim::EditHistory history;
    history.watch(scene);

    for (int i = 0; i < 40; ++i) {
        scene.notify(sim::ChangeKind::Simulation);
    }
    REQUIRE(history.undoDepth() == 0);

    history.setRecording(false);
    part->local.translation.x = 50;
    scene.notify(sim::ChangeKind::Transform);
    REQUIRE(history.undoDepth() == 0);
    history.setRecording(true);

    for (int i = 1; i <= 20; ++i) {
        part->local.translation.x = i;
        scene.notify(sim::ChangeKind::Transform);
    }
    REQUIRE(history.undoDepth() == sim::EditHistory::kDepth);

    REQUIRE(history.undo());
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(19));
    int steps = 1;
    while (history.undo()) {
        ++steps;
    }
    REQUIRE(steps == sim::EditHistory::kDepth);
    REQUIRE(partNamed(scene, "Plate")->local.translation.x == Approx(4));
    REQUIRE_FALSE(history.undo());
}

TEST_CASE("undo restores command edits for transform, signal, and motor") {
    sim::Scene scene;
    auto part = scene.makeNode(sim::NodeKind::Part, "Slider");
    scene.addChild(scene.root(), part);
    sim::EditHistory history;
    history.watch(scene);

    sim::CommandExecutor commands(scene, nullptr, nullptr);
    REQUIRE(commands.execute({{"op", "set_transform"}, {"name", "Slider"}, {"x", 8}, {"y", 0}, {"z", 1}}).ok);
    REQUIRE(commands.execute({{"op", "set_bool"}, {"name", "go"}, {"value", true}}).ok);
    REQUIRE(commands.execute({{"op", "motor_set"}, {"name", "drive"}, {"target", 12}, {"running", true}}).ok);
    REQUIRE(commands.execute({{"op", "set_joint_target"}, {"name", "Slider"}, {"value", 30}}).ok);
    REQUIRE(history.undoDepth() == 4);
    REQUIRE(scene.motors.size() == 1);

    REQUIRE(history.undo());
    REQUIRE(partNamed(scene, "Slider")->mech.target == Approx(0));
    REQUIRE(scene.motors.size() == 1);

    REQUIRE(history.undo());
    REQUIRE(scene.motors.empty());
    REQUIRE(scene.findSignal("go") != nullptr);
    REQUIRE(scene.findSignal("go")->flag);

    REQUIRE(history.undo());
    REQUIRE(scene.findSignal("go") == nullptr);
    REQUIRE(partNamed(scene, "Slider")->local.translation.x == Approx(8));

    REQUIRE(history.undo());
    REQUIRE(partNamed(scene, "Slider")->local.translation.x == Approx(0));
    REQUIRE(history.redo());
    REQUIRE(partNamed(scene, "Slider")->local.translation.x == Approx(8));
    REQUIRE(partNamed(scene, "Slider")->local.translation.z == Approx(1));
}
