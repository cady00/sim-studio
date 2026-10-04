#include "sim/ai/Commands.hpp"
#include "sim/ai/Provider.hpp"
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

    const auto future = commands.execute({{"op", "nl_to_logic"}, {"text", "when barrier then push"}});
    REQUIRE_FALSE(future.ok);
    REQUIRE(future.error.find("not implemented") != std::string::npos);
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
    configured.endpoint = "http://127.0.0.1:11434";
    configured.model = "local";
    auto local = sim::createAiProvider(configured);
    REQUIRE(local->enabled());
    REQUIRE_FALSE(local->usesNetwork());
    local->complete({}, error);
    REQUIRE(error.find("not contacted") != std::string::npos);
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
