#include "sim/scripting/ScriptHost.hpp"

#include "sim/collision/Query.hpp"

#include <pybind11/embed.h>

#include <utility>

namespace py = pybind11;

namespace sim {
namespace {

struct Bridge {
    Scene* scene = nullptr;
    std::string log;
    std::function<void(const std::string&)> onSpawned;
    std::function<void(const std::string&)> onRemoved;
};

Bridge gBridge;

Scene* sceneOrThrow() {
    if (gBridge.scene == nullptr) {
        throw std::runtime_error("no scene is bound");
    }
    return gBridge.scene;
}

std::shared_ptr<SceneNode> nodeOrThrow(const std::string& name) {
    auto node = sceneOrThrow()->findByName(name);
    if (!node) {
        throw std::runtime_error("no object named " + name);
    }
    return node;
}

PYBIND11_EMBEDDED_MODULE(simstudio, module) {
    module.def("_log", [](const std::string& text) { gBridge.log += text; });
    module.def("get_bool", [](const std::string& name) {
        const Signal* signal = sceneOrThrow()->findSignal(name);
        return signal != nullptr && signal->flag;
    });
    module.def("set_bool", [](const std::string& name, bool value) {
        Signal& signal = sceneOrThrow()->upsertSignal(name, SignalType::Bool);
        signal.flag = value;
    });
    module.def("get_int", [](const std::string& name) {
        const Signal* signal = sceneOrThrow()->findSignal(name);
        return signal == nullptr ? std::int64_t{0} : signal->integer;
    });
    module.def("set_int", [](const std::string& name, std::int64_t value) {
        Signal& signal = sceneOrThrow()->upsertSignal(name, SignalType::Int);
        signal.integer = value;
    });
    module.def("get_real", [](const std::string& name) {
        const Signal* signal = sceneOrThrow()->findSignal(name);
        return signal == nullptr ? 0.0 : signal->real;
    });
    module.def("set_real", [](const std::string& name, double value) {
        Signal& signal = sceneOrThrow()->upsertSignal(name, SignalType::Real);
        signal.real = value;
    });
    module.def("set_joint_target", [](const std::string& name, double target) {
        auto node = nodeOrThrow(name);
        node->mech.target = target;
        if (node->mech.drive == DriveMode::None) {
            node->mech.drive = DriveMode::Position;
        }
    });
    module.def("set_joint_speed", [](const std::string& name, double speed) {
        auto node = nodeOrThrow(name);
        node->mech.drive = DriveMode::Speed;
        node->mech.target = speed;
    });
    module.def("get_transform", [](const std::string& name) {
        const Transform& local = nodeOrThrow(name)->local;
        return py::make_tuple(local.translation.x, local.translation.y, local.translation.z, local.rotation.w,
                              local.rotation.x, local.rotation.y, local.rotation.z);
    });
    module.def("set_transform", [](const std::string& name, double x, double y, double z) {
        nodeOrThrow(name)->local.translation = {x, y, z};
    });
    module.def("spawn", [](const std::string& name) {
        Scene* scene = sceneOrThrow();
        auto prototype = scene->findByName(name);
        if (!prototype) {
            throw std::runtime_error("no object named " + name);
        }
        auto clone = scene->spawnClone(prototype, prototype->local);
        if (!clone) {
            throw std::runtime_error("could not spawn " + name);
        }
        if (gBridge.onSpawned) {
            gBridge.onSpawned(clone->id);
        }
        return clone->name;
    });
    module.def("delete", [](const std::string& name) {
        Scene* scene = sceneOrThrow();
        auto node = scene->findByName(name);
        if (!node) {
            throw std::runtime_error("no object named " + name);
        }
        const std::string id = node->id;
        if (gBridge.onRemoved) {
            gBridge.onRemoved(id);
        }
        scene->removeNode(id);
    });
    module.def("collision_query", [](double clearance) {
        auto query = createCollisionQuery();
        Scene* scene = sceneOrThrow();
        query->rebuild(*scene);
        py::list hits;
        for (const ContactHit& hit : query->query(*scene, clearance)) {
            py::dict item;
            item["a"] = hit.a;
            item["b"] = hit.b;
            item["kind"] = hit.kind == HitKind::Overlap ? "overlap" : "clearance";
            item["distance"] = hit.distance;
            item["penetration"] = hit.penetration;
            item["volume"] = hit.volume;
            item["volumeEstimated"] = hit.volumeEstimated;
            hits.append(item);
        }
        query->commitCache(*scene);
        return hits;
    });
    module.def("min_distance", [](const std::string& a, const std::string& b) {
        auto query = createCollisionQuery();
        Scene* scene = sceneOrThrow();
        query->rebuild(*scene);
        const DistanceResult distance = query->minDistance(*scene, a, b);
        if (!distance.ok) {
            throw std::runtime_error(distance.error.empty() ? "distance failed" : distance.error);
        }
        py::dict item;
        item["distance"] = distance.distance;
        item["gradient_from_sdf"] = distance.gradientFromSdf;
        item["normal"] = py::make_tuple(distance.normal.x, distance.normal.y, distance.normal.z);
        item["gradient"] = py::make_tuple(distance.gradient.x, distance.gradient.y, distance.gradient.z);
        return item;
    });
    module.def("set_body_motion", [](const std::string& name, const std::string& motionName) {
        BodyMotion motion = BodyMotion::Free;
        if (!bodyMotionFromName(motionName, motion)) {
            throw std::runtime_error("unknown body motion " + motionName);
        }
        auto node = nodeOrThrow(name);
        node->mech.motion = motion;
        syncMotionFlags(node->mech);
    });
    module.def("set_surface_grade", [](const std::string& name, const std::string& grade) {
        nodeOrThrow(name)->mech.surface = grade;
    });
    module.def("set_friction_pair", [](const std::string& a, const std::string& b, double friction, double restitution) {
        PhysicsDocument& document = sceneOrThrow()->physicsDoc;
        for (GradePair& pair : document.pairs) {
            if ((pair.a == a && pair.b == b) || (pair.a == b && pair.b == a)) {
                pair.friction = friction;
                pair.restitution = restitution;
                return;
            }
        }
        document.pairs.push_back(GradePair{a, b, friction, restitution});
    });
    module.def("set_motor", [](const std::string& name, double targetSpeed, bool running) {
        Scene* scene = sceneOrThrow();
        for (MotorAspect& motor : scene->motors) {
            if (motor.name == name) {
                motor.targetSpeed = targetSpeed;
                motor.running = running;
                return;
            }
        }
        MotorAspect motor;
        motor.name = name;
        motor.targetSpeed = targetSpeed;
        motor.running = running;
        scene->motors.push_back(std::move(motor));
    });
    module.def("motor_profile", [](double speed, double decel) {
        py::dict item;
        item["distance"] = stoppingDistance(speed, decel);
        item["time"] = stoppingTime(speed, decel);
        item["time_to_speed"] = timeToSpeed(0, speed, decel);
        return item;
    });
    module.def("encoder_count", [](const std::string& name) {
        for (const MotorAspect& motor : sceneOrThrow()->motors) {
            if (motor.name == name) {
                return sampleEncoder(motor).count;
            }
        }
        throw std::runtime_error("no motor named " + name);
    });
    module.def("sweep_check", [](const std::string& name, double x0, double y0, double z0, double x1, double y1, double z1) {
        auto query = createCollisionQuery();
        Scene* scene = sceneOrThrow();
        const SweepHit hit = query->sweep(*scene, name, {x0, y0, z0}, {x1, y1, z1});
        if (!hit.error.empty()) {
            throw std::runtime_error(hit.error);
        }
        py::dict item;
        item["hit"] = hit.hit;
        item["time"] = hit.time;
        item["a"] = hit.a;
        item["b"] = hit.b;
        return item;
    });
}

class PythonScriptHost final : public IScriptHost {
public:
    PythonScriptHost() {
        static bool ready = false;
        if (!ready) {
            py::initialize_interpreter();
            ready = true;
        }
        resetScope();
    }

    const char* backendName() const override { return "python"; }
    bool available() const override { return true; }

    void bind(Scene* scene) override { gBridge.scene = scene; }

    void setPhysicsHooks(std::function<void(const std::string&)> onSpawned,
                         std::function<void(const std::string&)> onRemoved) override {
        gBridge.onSpawned = std::move(onSpawned);
        gBridge.onRemoved = std::move(onRemoved);
    }

    bool load(const std::string& source, std::string& error) override {
        resetScope();
        hasStart_ = false;
        hasStep_ = false;
        hasStop_ = false;
        if (source.empty()) {
            return true;
        }
        if (!exec(source, error)) {
            return false;
        }
        hasStart_ = scopeContains("on_start");
        hasStep_ = scopeContains("on_step");
        hasStop_ = scopeContains("on_stop");
        return true;
    }

    void onStart() override { call("on_start", hasStart_); }
    void onStep(double dtSeconds) override {
        if (!hasStep_) {
            return;
        }
        try {
            scope_["on_step"](dtSeconds);
        } catch (const py::error_already_set& ex) {
            gBridge.log += ex.what();
            gBridge.log.push_back('\n');
            hasStep_ = false;
        }
    }
    void onStop() override { call("on_stop", hasStop_); }

    bool execute(const std::string& code, std::string& output, std::string& error) override {
        gBridge.log.clear();
        if (!exec(code, error)) {
            output = takeLog();
            return false;
        }
        hasStart_ = scopeContains("on_start");
        hasStep_ = scopeContains("on_step");
        hasStop_ = scopeContains("on_stop");
        output = takeLog();
        return true;
    }

    std::string takeLog() override {
        std::string out = std::move(gBridge.log);
        gBridge.log.clear();
        return out;
    }

private:
    void resetScope() {
        py::gil_scoped_acquire gil;
        py::dict dict;
        scope_ = dict;
        py::exec(R"py(
import sys
import simstudio as sim
simstudio = sim
class _Stream:
    def write(self, text):
        sim._log(str(text))
    def flush(self):
        pass
sys.stdout = _Stream()
sys.stderr = _Stream()
)py",
                 dict);
        scope_ = dict;
    }

    bool exec(const std::string& code, std::string& error) {
        try {
            py::exec(code, scope_.cast<py::dict>());
            return true;
        } catch (const py::error_already_set& ex) {
            error = ex.what();
            return false;
        }
    }

    bool scopeContains(const char* name) const {
        const py::dict dict = scope_.cast<py::dict>();
        return dict.contains(name) && !dict[name].is_none();
    }

    void call(const char* name, bool& enabled) {
        if (!enabled) {
            return;
        }
        try {
            scope_[name]();
        } catch (const py::error_already_set& ex) {
            gBridge.log += ex.what();
            gBridge.log.push_back('\n');
            enabled = false;
        }
    }

    py::object scope_;
    bool hasStart_ = false;
    bool hasStep_ = false;
    bool hasStop_ = false;
};

}  // namespace

std::unique_ptr<IScriptHost> createScriptHost() { return std::make_unique<PythonScriptHost>(); }

}  // namespace sim
