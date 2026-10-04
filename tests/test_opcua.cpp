#include "sim/core/Scene.hpp"
#include "sim/io/OpcUa.hpp"
#include "sim/motion/Motion.hpp"

#include <catch2/catch_test_macros.hpp>

#include <thread>

TEST_CASE("opc ua none session browses reads writes and publishes") {
    sim::OpcUaServer server;
    std::string error;
    REQUIRE(server.listen(0, error));
    REQUIRE(error.empty());
    std::string serverError;
    std::thread worker([&] { REQUIRE(server.serve(serverError)); });

    sim::OpcUaClient client;
    REQUIRE(client.connect("127.0.0.1", server.port(), error));
    REQUIRE(client.hello("opc.tcp://127.0.0.1:" + std::to_string(server.port()), error));
    REQUIRE(client.openSecureChannel(error));
    REQUIRE(client.createSession(error));
    REQUIRE(client.activateSession(error));

    std::vector<sim::OpcUaReference> refs;
    REQUIRE(client.browse(85, refs, error));
    REQUIRE(refs.size() == 1);
    REQUIRE(refs[0].nodeId == 10001);
    REQUIRE(refs[0].browseName == "PartSeen");

    bool value = true;
    REQUIRE(client.readBool(10001, value, error));
    REQUIRE_FALSE(value);
    REQUIRE(client.writeBool(10001, true, error));
    REQUIRE(client.readBool(10001, value, error));
    REQUIRE(value);

    std::uint32_t subscription = 0;
    REQUIRE(client.createSubscription(subscription, error));
    REQUIRE(subscription == 1);
    REQUIRE(client.monitorBool(subscription, 10001, 7, error));
    std::uint32_t handle = 0;
    bool published = false;
    REQUIRE(client.publish(handle, published, error));
    REQUIRE(handle == 7);
    REQUIRE(published);

    sim::Scene scene;
    sim::LogicBlock block;
    block.name = "inspect_seq";
    sim::LogicAction seen;
    seen.kind = "wait_signal";
    seen.signal = "part_seen";
    sim::LogicStep step;
    step.actions.push_back(seen);
    block.steps.push_back(std::move(step));
    scene.logicBlocks.push_back(std::move(block));
    sim::LogicMachine machine;
    REQUIRE(machine.start(scene, "inspect_seq", error));
    machine.step(scene, 0.01);
    REQUIRE_FALSE(machine.finished("inspect_seq"));
    sim::publishPlcBool(scene, "part_seen", published);
    machine.step(scene, 0.01);
    REQUIRE(machine.finished("inspect_seq"));

    client.close();
    worker.join();
}
