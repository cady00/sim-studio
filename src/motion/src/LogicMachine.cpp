#include "sim/motion/Motion.hpp"

#include <cmath>

namespace sim {
namespace {

ProfileKind profileKind(const std::string& name) {
    return name == "scurve" ? ProfileKind::SCurve : ProfileKind::Trapezoid;
}

bool nearly(double a, double b) { return std::abs(a - b) <= 1e-6; }

double signalNumber(const Scene& scene, const LogicBlock& block, const std::string& name, bool& isBool, bool& flag) {
    if (const Signal* signal = scene.findSignal(name)) {
        isBool = signal->type == SignalType::Bool;
        flag = signal->flag;
        if (signal->type == SignalType::Int) {
            return static_cast<double>(signal->integer);
        }
        if (signal->type == SignalType::Real) {
            return signal->real;
        }
        return signal->flag ? 1.0 : 0.0;
    }
    for (const LogicParam& param : block.parameters) {
        if (param.name == name) {
            isBool = param.type == "bool";
            flag = param.flag;
            if (param.type == "int") {
                return static_cast<double>(param.integer);
            }
            return param.real;
        }
    }
    isBool = true;
    flag = false;
    return 0;
}

bool conditionHolds(const Scene& scene, const LogicBlock& block, const LogicCondition& condition) {
    bool isBool = false;
    bool flag = false;
    const double value = signalNumber(scene, block, condition.signal, isBool, flag);
    if (condition.asBool && isBool) {
        const bool expected = condition.flag;
        if (condition.op == "!=") {
            return flag != expected;
        }
        return flag == expected;
    }
    const double rhs = condition.number;
    if (condition.op == "!=") {
        return !nearly(value, rhs);
    }
    if (condition.op == ">") {
        return value > rhs;
    }
    if (condition.op == "<") {
        return value < rhs;
    }
    if (condition.op == ">=") {
        return value > rhs || nearly(value, rhs);
    }
    if (condition.op == "<=") {
        return value < rhs || nearly(value, rhs);
    }
    return nearly(value, rhs);
}

bool stepReady(const Scene& scene, const LogicBlock& block, const LogicStep& step) {
    for (const LogicCondition& condition : step.when) {
        if (!conditionHolds(scene, block, condition)) {
            return false;
        }
    }
    return true;
}

}  // namespace

void LogicMachine::reset() {
    runs_.clear();
    lastError_.clear();
}

bool LogicMachine::start(Scene& scene, const std::string& blockName, std::string& error) {
    bool found = false;
    for (const LogicBlock& block : scene.logicBlocks) {
        if (block.name != blockName) {
            continue;
        }
        found = true;
        break;
    }
    if (!found) {
        error = "no logic block named " + blockName;
        return false;
    }
    Run run;
    run.active = true;
    run.sawLow = false;
    runs_.push_back({blockName, run});
    error.clear();
    return true;
}

bool LogicMachine::finished(const std::string& blockName) const {
    for (const auto& [name, run] : runs_) {
        if (name == blockName) {
            return run.finished;
        }
    }
    return false;
}

void LogicMachine::step(Scene& scene, double dt) {
    dt = std::max(dt, 0.0);
    for (LogicBlock& block : scene.logicBlocks) {
        Run* existing = nullptr;
        for (auto& [id, candidate] : runs_) {
            if (id == block.name) {
                existing = &candidate;
                break;
            }
        }
        const bool gate = block.runSignal.empty() || (scene.findSignal(block.runSignal) != nullptr && scene.findSignal(block.runSignal)->flag);
        if (!block.runSignal.empty() && !gate) {
            if (existing != nullptr) {
                existing->sawLow = true;
            }
            continue;
        }
        if (existing == nullptr && !block.runSignal.empty() && gate) {
            Run run;
            run.active = true;
            run.sawLow = false;
            runs_.push_back({block.name, run});
            existing = &runs_.back().second;
        }
        if (existing == nullptr || !existing->active || existing->finished) {
            continue;
        }
        Run& run = *existing;
        if (run.waiting) {
            if (run.waitSignal) {
                bool isBool = false;
                bool flag = false;
                signalNumber(scene, block, run.signal, isBool, flag);
                if (!flag) {
                    continue;
                }
                run.waiting = false;
                run.waitSignal = false;
                ++run.action;
            } else {
                run.waitLeft -= dt;
                if (run.waitLeft > 0) {
                    continue;
                }
                run.waiting = false;
                ++run.action;
            }
        } else if (run.move.active) {
            KinematicResource* resource = findResource(scene.resources, block.resource);
            if (resource == nullptr) {
                lastError_ = "logic resource missing";
                run.finished = true;
                continue;
            }
            if (!advanceMove(*resource, run.move, dt)) {
                ++run.action;
            }
            applyResource(scene, *resource);
            if (run.move.active) {
                continue;
            }
        } else if (run.jogging) {
            KinematicResource* resource = findResource(scene.resources, block.resource);
            KinJoint* joint = resource == nullptr ? nullptr : findJoint(*resource, run.jogJoint);
            if (joint == nullptr) {
                lastError_ = "jog joint missing";
                run.finished = true;
                run.jogging = false;
                continue;
            }
            const double step = std::clamp(run.jogSpeed, -joint->maxSpeed, joint->maxSpeed) * dt;
            joint->position = clampJoint(*joint, joint->position + step);
            applyResource(scene, *resource);
            run.jogLeft -= dt;
            if (run.jogLeft > 0) {
                continue;
            }
            run.jogging = false;
            ++run.action;
        }
        while (run.step < block.steps.size()) {
            LogicStep& step = block.steps[run.step];
            if (!stepReady(scene, block, step)) {
                break;
            }
            if (run.action >= step.actions.size()) {
                ++run.step;
                run.action = 0;
                continue;
            }
            const LogicAction& action = step.actions[run.action];
            if (action.kind == "set_output" || action.kind == "reset_output") {
                Signal& signal = scene.upsertSignal(action.signal, SignalType::Bool);
                signal.flag = action.kind == "set_output";
                ++run.action;
                continue;
            }
            if (action.kind == "set_value") {
                Signal& signal = scene.upsertSignal(action.signal, SignalType::Real);
                signal.real = action.speed;
                ++run.action;
                continue;
            }
            if (action.kind == "wait") {
                run.waiting = true;
                run.waitSignal = false;
                run.waitLeft = action.seconds;
                break;
            }
            if (action.kind == "wait_signal") {
                run.waiting = true;
                run.waitSignal = true;
                run.signal = action.signal;
                break;
            }
            const std::string& resourceName = action.resource.empty() ? block.resource : action.resource;
            KinematicResource* resource = findResource(scene.resources, resourceName);
            if (resource == nullptr) {
                lastError_ = "logic resource missing";
                run.finished = true;
                break;
            }
            if (action.kind == "jog") {
                run.jogging = true;
                run.jogJoint = action.joint;
                run.jogSpeed = action.speed;
                run.jogLeft = action.seconds;
                break;
            }
            if (action.kind == "move_pose") {
                std::string error;
                if (!beginPoseMove(*resource, action.pose, profileKind(action.profile), run.move, error)) {
                    lastError_ = error;
                    run.finished = true;
                    break;
                }
                if (!run.move.active || run.move.duration <= 0) {
                    run.move.active = false;
                    ++run.action;
                    continue;
                }
                break;
            }
            lastError_ = "unknown logic action " + action.kind;
            run.finished = true;
            break;
        }
        if (run.step >= block.steps.size() && !run.waiting && !run.jogging && !run.move.active && !run.waitSignal) {
            if (block.repeat) {
                run.step = 0;
                run.action = 0;
                run.finished = false;
                run.active = true;
            } else {
                run.finished = true;
                run.active = false;
            }
        }
    }
}

}  // namespace sim
