#include "sim/ai/Commands.hpp"

#include <vector>

namespace sim {
namespace {

CommandResult okValue(nlohmann::json value) {
    CommandResult result;
    result.ok = true;
    result.value = std::move(value);
    return result;
}

CommandResult fail(std::string error) {
    CommandResult result;
    result.error = std::move(error);
    return result;
}

std::shared_ptr<SceneNode> named(Scene& scene, const nlohmann::json& command, CommandResult& error) {
    const std::string name = command.value("name", "");
    auto node = scene.findByName(name);
    if (!node) {
        error = fail("no object named " + name);
    }
    return node;
}

}  // namespace

CommandExecutor::CommandExecutor(Scene& scene, Simulation* simulation, IPhysicsEngine* physics)
    : scene_(scene), simulation_(simulation), physics_(physics) {}

nlohmann::json CommandExecutor::catalog() {
    const auto op = [](const char* name, const char* summary, nlohmann::json params, bool executable) {
        return nlohmann::json{{"op", name}, {"summary", summary}, {"params", std::move(params)}, {"executable", executable}};
    };
    return nlohmann::json::array({
        op("get_bool", "Read a boolean signal. Missing signals read as false.", {"name"}, true),
        op("set_bool", "Write a boolean signal, creating it if needed.", {"name", "value"}, true),
        op("get_int", "Read an integer signal.", {"name"}, true),
        op("set_int", "Write an integer signal.", {"name", "value"}, true),
        op("get_real", "Read a real signal.", {"name"}, true),
        op("set_real", "Write a real signal.", {"name", "value"}, true),
        op("set_joint_target", "Same as sim.set_joint_target. Position units are mm or degrees.", {"name", "value"}, true),
        op("set_joint_speed", "Same as sim.set_joint_speed.", {"name", "value"}, true),
        op("get_transform", "Local pose as tx,ty,tz,qw,qx,qy,qz in millimeters.", {"name"}, true),
        op("set_transform", "Set local translation in millimeters.", {"name", "x", "y", "z"}, true),
        op("spawn", "Clone a named object. Returns the new name.", {"name"}, true),
        op("delete", "Remove a named object and its rigid body.", {"name"}, true),
        op("list_nodes", "Names and mechanism kinds in the scene.", nlohmann::json::array(), true),
        op("sim_play", "Start or resume the fixed-step simulation.", nlohmann::json::array(), true),
        op("sim_pause", "Pause. Time does not advance.", nlohmann::json::array(), true),
        op("sim_stop", "Stop, restore rest poses, delete transient bodies.", nlohmann::json::array(), true),
        op("sim_state", "state, time, and speed.", nlohmann::json::array(), true),
        op("nl_to_logic", "Future: natural language to a logic block or script.", {"text"}, false),
        op("layout_suggest", "Future: suggest a line layout. Does not edit the scene.", {"text"}, false),
        op("cycle_optimize", "Future: suggest cycle-time changes.", {"text"}, false),
        op("explain_anomaly", "Future: explain a signal or cycle anomaly.", {"text"}, false),
    });
}

CommandResult CommandExecutor::execute(const nlohmann::json& command) const {
    if (!command.is_object() || !command.contains("op")) {
        return fail("command needs an op field");
    }
    const std::string op = command.value("op", "");
    if (op == "get_bool") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal != nullptr && signal->flag);
    }
    if (op == "set_bool") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Bool);
        signal.flag = command.value("value", false);
        return okValue(signal.flag);
    }
    if (op == "get_int") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0 : signal->integer);
    }
    if (op == "set_int") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Int);
        signal.integer = command.value("value", 0);
        return okValue(signal.integer);
    }
    if (op == "get_real") {
        const Signal* signal = scene_.findSignal(command.value("name", ""));
        return okValue(signal == nullptr ? 0.0 : signal->real);
    }
    if (op == "set_real") {
        Signal& signal = scene_.upsertSignal(command.value("name", ""), SignalType::Real);
        signal.real = command.value("value", 0.0);
        return okValue(signal.real);
    }
    if (op == "set_joint_target" || op == "set_joint_speed") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        node->mech.target = command.value("value", 0.0);
        node->mech.drive = op == "set_joint_speed" ? DriveMode::Speed : node->mech.drive == DriveMode::None ? DriveMode::Position : node->mech.drive;
        if (op == "set_joint_speed") {
            node->mech.drive = DriveMode::Speed;
        }
        return okValue(node->mech.target);
    }
    if (op == "get_transform") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        const Transform& local = node->local;
        return okValue(nlohmann::json::array({local.translation.x, local.translation.y, local.translation.z, local.rotation.w,
                                              local.rotation.x, local.rotation.y, local.rotation.z}));
    }
    if (op == "set_transform") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        node->local.translation = {command.value("x", 0.0), command.value("y", 0.0), command.value("z", 0.0)};
        return okValue(true);
    }
    if (op == "spawn") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        auto clone = scene_.spawnClone(node, node->local);
        if (!clone) {
            return fail("could not spawn " + command.value("name", ""));
        }
        if (physics_ != nullptr) {
            physics_->adoptSpawned(clone->id);
        }
        return okValue(clone->name);
    }
    if (op == "delete") {
        CommandResult error;
        auto node = named(scene_, command, error);
        if (!node) {
            return error;
        }
        const std::string id = node->id;
        if (physics_ != nullptr) {
            physics_->releaseBody(id);
        }
        scene_.removeNode(id);
        return okValue(true);
    }
    if (op == "list_nodes") {
        nlohmann::json names = nlohmann::json::array();
        std::vector<std::shared_ptr<SceneNode>> stack{scene_.root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->id == "root") {
                continue;
            }
            names.push_back({{"name", node->name}, {"kind", mechKindName(node->mech.kind)}});
        }
        return okValue(std::move(names));
    }
    if (op == "sim_state") {
        if (simulation_ == nullptr) {
            return fail("no simulation");
        }
        const char* state = "stopped";
        if (simulation_->state() == SimState::Playing) {
            state = "playing";
        } else if (simulation_->state() == SimState::Paused) {
            state = "paused";
        }
        return okValue({{"state", state}, {"time", simulation_->time()}, {"speed", simulation_->speed()}});
    }
    if (op == "sim_play" || op == "sim_pause" || op == "sim_stop") {
        if (simulation_ == nullptr) {
            return fail("no simulation");
        }
        if (op == "sim_play") {
            simulation_->play();
        } else if (op == "sim_pause") {
            simulation_->pause();
        } else {
            simulation_->stop();
        }
        if (!simulation_->lastError().empty()) {
            return fail(simulation_->lastError());
        }
        return execute(nlohmann::json{{"op", "sim_state"}});
    }
    if (op == "nl_to_logic" || op == "layout_suggest" || op == "cycle_optimize" || op == "explain_anomaly") {
        return fail(op + " is a documented hook and is not implemented");
    }
    return fail("unknown op " + op);
}

}  // namespace sim
