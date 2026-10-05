#include "sim/ai/CommandRegistry.hpp"
#include "sim/ai/Commands.hpp"
#include "sim/ai/Provider.hpp"
#include "sim/io/EditHistory.hpp"
#include "sim/render/Renderer.hpp"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("command catalog is executable without a model") {
    sim::Scene scene;
    auto part = scene.makeNode(sim::NodeKind::Part, "Slider");
    scene.addChild(scene.root(), part);
    part->mech.kind = sim::MechKind::Slider;

    sim::CommandExecutor commands(scene, nullptr, nullptr);
    const auto catalog = sim::CommandExecutor::catalog();
    REQUIRE(catalog.is_array());
    REQUIRE(catalog.size() >= 12);

    REQUIRE(commands.execute({{"op", "set_bool"}, {"name", "go"}, {"value", true}}).ok);
    REQUIRE(commands.execute({{"op", "get_bool"}, {"name", "go"}}).value == true);
    REQUIRE(commands.execute({{"op", "set_joint_target"}, {"name", "Slider"}, {"value", 12}}).ok);
    REQUIRE(part->mech.target == 12);
    REQUIRE(part->mech.drive == sim::DriveMode::Position);

    sim::EditHistory history;
    history.watch(scene);
    const auto future = commands.execute({{"op", "nl_to_logic"}, {"text", "when barrier then push"}});
    REQUIRE(future.ok);
    REQUIRE(future.value.at("commands").is_array());
    REQUIRE(scene.logicBlocks.size() == 1);
    REQUIRE(scene.logicBlocks.front().name == "nl");
    REQUIRE(scene.logicBlocks.front().steps.front().actions.front().signal == "barrier");
    REQUIRE(history.undo());
    REQUIRE(scene.logicBlocks.empty());
}

TEST_CASE("offline hooks edit through commands and a remote model is refused") {
    sim::Scene scene;
    sim::CommandExecutor commands(scene, nullptr, nullptr);
    const auto layout = commands.execute({{"op", "layout_suggest"}, {"text", "焊接工位"}});
    REQUIRE(layout.ok);
    REQUIRE(scene.layout.stations.size() == 1);
    REQUIRE(scene.layout.stations.front().id == "st_weld");

    sim::Operation slow;
    slow.id = "slow";
    slow.kind = sim::OpKind::Task;
    slow.seconds = 30;
    sim::Operation fast;
    fast.id = "fast";
    fast.kind = sim::OpKind::Task;
    fast.seconds = 5;
    scene.operations = {slow, fast};
    const auto cycle = commands.execute({{"op", "cycle_optimize"}, {"text", "优化节拍"}});
    REQUIRE(cycle.ok);
    REQUIRE(scene.layout.cyclic);
    const sim::Operation* updated = nullptr;
    for (const sim::Operation& operation : scene.operations) {
        if (operation.id == "slow") {
            updated = &operation;
        }
    }
    REQUIRE(updated != nullptr);
    REQUIRE(updated->seconds == 5);

    scene.upsertSignal("part_seen", sim::SignalType::Bool).flag = true;
    const auto explained = commands.execute({{"op", "explain_anomaly"}, {"text", ""}});
    REQUIRE(explained.ok);
    REQUIRE(explained.value.at("note").get<std::string>().find("part_seen") != std::string::npos);
    REQUIRE(scene.findSignal("anomaly_explained")->flag);

    const auto remote = commands.execute({{"op", "nl_to_logic"},
                                          {"text", "when barrier then push"},
                                          {"use_model", true},
                                          {"endpoint", "http://example.com/v1/chat/completions"}});
    REQUIRE(remote.ok);
    REQUIRE(remote.value.at("note").get<std::string>().find("localhost") != std::string::npos);
    REQUIRE(scene.logicBlocks.size() == 1);
}

TEST_CASE("ai provider stays offline unless a future transport is compiled") {
    sim::ProviderConfig off;
    auto provider = sim::createAiProvider(off);
    REQUIRE_FALSE(provider->enabled());
    REQUIRE_FALSE(provider->usesNetwork());
    std::string error;
    provider->complete({{"messages", {"hello"}}}, error);
    REQUIRE(error.find("off") != std::string::npos);

    sim::ProviderConfig configured;
    configured.enabled = true;
    configured.kind = "openai-compatible";
    configured.endpoint = "http://127.0.0.1:11434/v1/chat/completions";
    configured.model = "local";
    auto local = sim::createAiProvider(configured);
    REQUIRE(local->enabled());
    REQUIRE_FALSE(local->usesNetwork());
    REQUIRE(local->capabilities().size() == 4);
    local->complete({{"messages", nlohmann::json::array()}}, error);
    REQUIRE(error.find("not reached") != std::string::npos);

    sim::ProviderConfig remote;
    remote.enabled = true;
    remote.kind = "openai-compatible";
    remote.endpoint = "http://example.com/v1/chat/completions";
    auto blocked = sim::createAiProvider(remote);
    blocked->complete({}, error);
    REQUIRE(error.find("only connects to localhost") != std::string::npos);
}

TEST_CASE("lod and frustum hooks keep a near box and drop a far one") {
    REQUIRE(sim::selectLodLevel(100.0f, 3) == 0);
    REQUIRE(sim::selectLodLevel(4000.0f, 3) == 1);
    REQUIRE(sim::selectLodLevel(9000.0f, 3) == 2);

    sim::Mat4 identity = sim::Mat4::identity();
    sim::Aabb nearBox;
    nearBox.expand({-1, -1, -1});
    nearBox.expand({1, 1, 1});
    REQUIRE(sim::intersectsClipFrustum(identity, nearBox));

    sim::Aabb farBox;
    farBox.expand({100, 100, 100});
    farBox.expand({120, 120, 120});
    REQUIRE_FALSE(sim::intersectsClipFrustum(identity, farBox));
}

TEST_CASE("command palette matches chinese, pinyin initials, english, objects, and signals") {
    REQUIRE(sim::CommandRegistry::pinyinInitials("示教点") == "sjd");
    sim::Scene scene;
    auto part = scene.makeNode(sim::NodeKind::Part, "示教点");
    scene.addChild(scene.root(), part);
    scene.upsertSignal("go", sim::SignalType::Bool);

    const auto byInitial = sim::CommandRegistry::search("sjd", &scene);
    REQUIRE(byInitial.size() == 2);
    REQUIRE(byInitial.front().entry.id == "ui.teach");
    REQUIRE(byInitial.back().entry.group == "object");

    const auto byEnglish = sim::CommandRegistry::search("Teach", &scene);
    REQUIRE(byEnglish.size() == 1);
    REQUIRE(byEnglish.front().entry.id == "ui.teach");

    const auto objects = sim::CommandRegistry::search("@示教", &scene);
    REQUIRE(objects.size() == 2);
    bool teachObject = false;
    for (const auto& hit : objects) {
        REQUIRE(hit.entry.group == "object");
        teachObject = teachObject || hit.entry.titleZh == "示教点";
    }
    REQUIRE(teachObject);

    const auto signals = sim::CommandRegistry::search("#go", &scene);
    REQUIRE(signals.size() == 1);
    REQUIRE(signals.front().entry.group == "signal");
    REQUIRE(signals.front().entry.id == "signal:go");

    const sim::CommandPlan vague = sim::CommandRegistry::planIntent("把节拍变得更好");
    REQUIRE(vague.needsClarify);
    REQUIRE(vague.steps.empty());

    const sim::CommandPlan play = sim::CommandRegistry::planIntent("播放并查询近碰");
    REQUIRE_FALSE(play.needsClarify);
    REQUIRE(play.steps.size() == 2);
    REQUIRE(play.steps[0].op == "sim_play");
    REQUIRE(play.steps[1].op == "collision_query");
}
