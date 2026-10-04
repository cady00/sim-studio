#include "sim/core/Scene.hpp"
#include "sim/io/OpcUa.hpp"
#include "sim/motion/Motion.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <thread>

using Catch::Approx;

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

void handshake(sim::OpcUaClient& client, sim::OpcUaServer& server, const std::string& endpoint, std::string& error) {
    REQUIRE(client.connect("127.0.0.1", server.port(), error));
    REQUIRE(client.hello(endpoint, error));
    REQUIRE(client.openSecureChannel(error));
    REQUIRE(client.createSession(error));
    REQUIRE(client.activateSession(error));
}

TEST_CASE("opc ua basic256 signs encrypts logs in and reconnects") {
    sim::OpcUaCertificate serverCert;
    sim::OpcUaCertificate clientCert;
    std::string error;
    REQUIRE(serverCert.createSelfSigned("simstudio-server", error));
    REQUIRE(clientCert.createSelfSigned("simstudio-client", error));
    const auto directory = std::filesystem::temp_directory_path();
    const auto certPath = directory / "simstudio-server.pem";
    const auto keyPath = directory / "simstudio-server.key";
    REQUIRE(serverCert.savePem(certPath.string(), keyPath.string(), error));
    sim::OpcUaCertificate loaded;
    REQUIRE(loaded.loadPem(certPath.string(), keyPath.string(), error));
    REQUIRE(loaded.der == serverCert.der);
    std::filesystem::remove(certPath);
    std::filesystem::remove(keyPath);

    sim::OpcUaServer server;
    server.useSecurity(sim::OpcUaSecurity::SignAndEncrypt, serverCert);
    server.setUser("operator", "secret");
    REQUIRE(server.listen(0, error));
    std::string serverError;
    std::thread worker([&] { REQUIRE(server.serve(serverError)); });
    const std::string endpoint = "opc.tcp://127.0.0.1:" + std::to_string(server.port());

    sim::OpcUaClient client;
    client.useSecurity(sim::OpcUaSecurity::SignAndEncrypt, clientCert, serverCert);
    client.setUser("operator", "nope");
    REQUIRE(client.connect("127.0.0.1", server.port(), error));
    REQUIRE(client.hello(endpoint, error));
    REQUIRE(client.openSecureChannel(error));
    REQUIRE(client.createSession(error));
    REQUIRE_FALSE(client.activateSession(error));
    client.close();
    worker.join();

    std::thread again([&] { REQUIRE(server.serve(serverError)); });
    client.setUser("operator", "secret");
    REQUIRE(client.reconnect(error));
    std::vector<sim::OpcUaReference> refs;
    REQUIRE(client.browse(85, refs, error));
    REQUIRE(refs.size() == 1);
    REQUIRE(refs[0].browseName == "PartSeen");
    bool value = true;
    REQUIRE(client.readBool(10001, value, error));
    REQUIRE_FALSE(value);
    REQUIRE(client.writeBool(10001, true, error));
    REQUIRE(client.readBool(10001, value, error));
    REQUIRE(value);
    client.close();
    again.join();

    sim::OpcUaServer signedServer;
    signedServer.useSecurity(sim::OpcUaSecurity::Sign, serverCert);
    signedServer.setUser("operator", "secret");
    REQUIRE(signedServer.listen(0, error));
    std::thread signedWorker([&] { REQUIRE(signedServer.serve(serverError)); });
    sim::OpcUaClient signedClient;
    signedClient.useSecurity(sim::OpcUaSecurity::Sign, clientCert, serverCert);
    signedClient.setUser("operator", "secret");
    handshake(signedClient, signedServer, "opc.tcp://127.0.0.1:" + std::to_string(signedServer.port()), error);
    REQUIRE(signedClient.readBool(10001, value, error));
    REQUIRE_FALSE(value);
    signedClient.close();
    signedWorker.join();
}

TEST_CASE("opc ua signal table imports csv and tia exports") {
    const char* csv = "name,type,value\nextend,bool,true\ncount,int,4\ngap,real,12.5\n";
    const char* tia =
        "\"Name\";\"Path\";\"Data Type\";\"Logical Address\";\"Comment\"\n"
        "\"extend\";\"PLC.Tags\";\"Bool\";\"%M0.0\";\"气缸\"\n"
        "\"gap\";\"PLC.Tags\";\"Real\";\"%MD10\";\"\"\n"
        "\"count\";\"PLC.Tags\";\"Int\";\"%MW4\";\"\"\n";
    std::vector<sim::ImportedSignal> rows;
    std::string error;
    REQUIRE(sim::importSignalTable(csv, rows, error));
    REQUIRE(rows.size() == 3);
    REQUIRE(rows[0].name == "extend");
    REQUIRE(rows[0].flag);
    REQUIRE(rows[1].integer == 4);
    REQUIRE(rows[2].real == Approx(12.5));
    REQUIRE(sim::importSignalTable(tia, rows, error));
    REQUIRE(rows.size() == 3);
    REQUIRE(rows[0].type == sim::SignalType::Bool);
    REQUIRE(rows[1].type == sim::SignalType::Real);
    REQUIRE(rows[2].type == sim::SignalType::Int);
    sim::Scene scene;
    sim::applyImportedSignals(scene, rows);
    REQUIRE(scene.findSignal("extend")->type == sim::SignalType::Bool);
    REQUIRE(scene.findSignal("count")->type == sim::SignalType::Int);
    REQUIRE(scene.findSignal("gap")->type == sim::SignalType::Real);
}
