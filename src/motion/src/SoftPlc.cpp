#include "sim/motion/SoftPlc.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sim {
namespace {

bool forcedBool(const PlcTag& tag) {
    if (tag.kind == PlcValueKind::Int) {
        return tag.forceInteger != 0;
    }
    if (tag.kind == PlcValueKind::Real) {
        return tag.forceReal != 0.0;
    }
    return tag.forceFlag;
}

bool tagBool(const PlcTag* tag) {
    if (tag == nullptr) {
        return false;
    }
    if (tag->forced) {
        return forcedBool(*tag);
    }
    if (tag->kind == PlcValueKind::Timer) {
        return tag->done;
    }
    if (tag->kind == PlcValueKind::Int) {
        return tag->integer != 0;
    }
    if (tag->kind == PlcValueKind::Real) {
        return tag->real != 0.0;
    }
    return tag->flag;
}

const char* bit(bool value) { return value ? "1" : "0"; }

}  // namespace

void resetSoftPlcTimers(SoftPlcDocument& doc) {
    for (PlcTag& tag : doc.tags) {
        if (tag.kind != PlcValueKind::Timer) {
            continue;
        }
        tag.elapsed = 0;
        tag.done = false;
        if (!tag.forced) {
            tag.flag = false;
        }
    }
}

void SoftPlcRuntime::setOpcExchange(PlcOpcExchange exchange) { exchange_ = std::move(exchange); }

bool SoftPlcRuntime::start(const SoftPlcDocument& doc, std::string& error) {
    (void)doc;
    running_ = true;
    acc_ = 0;
    scans_ = 0;
    log_.clear();
    opcWarned_.clear();
    previous_.clear();
    error.clear();
    note("softplc started");
    return true;
}

void SoftPlcRuntime::stop() {
    running_ = false;
    acc_ = 0;
    scans_ = 0;
    log_.clear();
    opcWarned_.clear();
    previous_.clear();
}

void SoftPlcRuntime::note(const std::string& line) {
    if (log_.size() >= 200) {
        return;
    }
    log_.push_back(line);
}

bool SoftPlcRuntime::readOpc(const std::string& node, bool& value) {
    std::string error;
    if (!exchange_ || !exchange_(node, false, value, error)) {
        if (std::find(opcWarned_.begin(), opcWarned_.end(), node) == opcWarned_.end()) {
            opcWarned_.push_back(node);
            note("OPC UA node " + node + " is not connected");
        }
        return false;
    }
    return true;
}

bool SoftPlcRuntime::writeOpc(const std::string& node, bool value) {
    std::string error;
    if (!exchange_ || !exchange_(node, true, value, error)) {
        if (std::find(opcWarned_.begin(), opcWarned_.end(), node) == opcWarned_.end()) {
            opcWarned_.push_back(node);
            note("OPC UA node " + node + " is not connected");
        }
        return false;
    }
    return true;
}

const SoftPlcRuntime::Image* SoftPlcRuntime::previous(const std::string& address) const {
    for (const Image& image : previous_) {
        if (image.address == address) {
            return &image;
        }
    }
    return nullptr;
}

void SoftPlcRuntime::oneScan(Scene& scene, double simTime, double period) {
    SoftPlcDocument& doc = scene.softPlc;
    ++scans_;
    for (const PlcBinding& binding : doc.bindings) {
        if (binding.direction != "in") {
            continue;
        }
        PlcTag* tag = findPlcTag(doc, binding.address);
        if (tag == nullptr || tag->forced || tag->kind == PlcValueKind::Timer) {
            continue;
        }
        bool got = false;
        if (!binding.device.empty()) {
            if (const Signal* signal = scene.findSignal(binding.device)) {
                got = true;
                if (tag->kind == PlcValueKind::Int) {
                    if (signal->type == SignalType::Int) {
                        tag->integer = signal->integer;
                    } else if (signal->type == SignalType::Real) {
                        tag->integer = static_cast<std::int64_t>(std::llround(signal->real));
                    } else {
                        tag->integer = signal->flag ? 1 : 0;
                    }
                } else if (tag->kind == PlcValueKind::Real) {
                    if (signal->type == SignalType::Real) {
                        tag->real = signal->real;
                    } else if (signal->type == SignalType::Int) {
                        tag->real = static_cast<double>(signal->integer);
                    } else {
                        tag->real = signal->flag ? 1.0 : 0.0;
                    }
                } else {
                    tag->flag = signal->type == SignalType::Bool ? signal->flag : (signal->integer != 0 || signal->real != 0.0);
                }
            }
        } else if (!binding.opcNode.empty() && tag->kind == PlcValueKind::Bool) {
            bool value = false;
            if (readOpc(binding.opcNode, value)) {
                tag->flag = value;
                got = true;
            }
            (void)got;
        }
    }

    for (const PlcRung& rung : doc.rungs) {
        bool power = true;
        for (const PlcContact& contact : rung.series) {
            const bool on = tagBool(findPlcTag(doc, contact.address));
            power = power && (contact.normallyOpen ? on : !on);
        }
        PlcTag* coil = findPlcTag(doc, rung.coil);
        if (coil == nullptr || coil->forced) {
            continue;
        }
        if (rung.action == "ton" && coil->kind == PlcValueKind::Timer) {
            if (!power) {
                coil->elapsed = 0;
                coil->done = false;
            } else {
                coil->elapsed += period;
                coil->done = coil->elapsed + 1e-6 >= std::max(coil->preset, 0.0);
            }
            coil->flag = coil->done;
            continue;
        }
        if (rung.action == "move") {
            if (!power) {
                continue;
            }
            if (coil->kind == PlcValueKind::Int) {
                coil->integer = static_cast<std::int64_t>(std::llround(rung.literal));
            } else if (coil->kind == PlcValueKind::Real) {
                coil->real = rung.literal;
            } else if (coil->kind == PlcValueKind::Bool) {
                coil->flag = rung.literal != 0.0;
            }
            continue;
        }
        if (coil->kind == PlcValueKind::Timer) {
            continue;
        }
        if (rung.action == "set") {
            if (power) {
                coil->flag = true;
            }
        } else if (rung.action == "reset") {
            if (power) {
                coil->flag = false;
            }
        } else if (rung.action == "out") {
            coil->flag = power;
        }
    }

    std::string changes;
    for (const PlcTag& tag : doc.tags) {
        const Image* before = previous(tag.address);
        const std::int64_t integer = tag.forced ? tag.forceInteger : tag.integer;
        const double real = tag.forced ? tag.forceReal : tag.real;
        const bool now = tagBool(&tag);
        bool changed = before == nullptr;
        if (before != nullptr) {
            if (tag.kind == PlcValueKind::Int) {
                changed = before->integer != integer;
            } else if (tag.kind == PlcValueKind::Real) {
                changed = std::abs(before->real - real) > 1e-9;
            } else if (tag.kind == PlcValueKind::Timer) {
                changed = before->done != tag.done;
            } else {
                changed = before->flag != now;
            }
        }
        if (!changed) {
            continue;
        }
        if (!changes.empty()) {
            changes += ' ';
        }
        changes += tag.address;
        changes += '=';
        if (tag.kind == PlcValueKind::Int) {
            changes += std::to_string(integer);
        } else if (tag.kind == PlcValueKind::Real) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.4g", real);
            changes += buf;
        } else {
            changes += bit(now);
        }
    }
    if (scans_ == 1 || !changes.empty()) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "t=%.3f #%d", simTime, scans_);
        note(changes.empty() ? std::string(buf) : std::string(buf) + " " + changes);
    }

    previous_.clear();
    for (const PlcTag& tag : doc.tags) {
        Image image;
        image.address = tag.address;
        image.flag = tag.kind == PlcValueKind::Bool ? (tag.forced ? tag.forceFlag : tag.flag) : tag.flag;
        image.integer = tag.forced ? tag.forceInteger : tag.integer;
        image.real = tag.forced ? tag.forceReal : tag.real;
        image.done = tag.done;
        previous_.push_back(std::move(image));
    }

    for (const PlcBinding& binding : doc.bindings) {
        if (binding.direction != "out") {
            continue;
        }
        const PlcTag* tag = findPlcTag(doc, binding.address);
        if (tag == nullptr) {
            continue;
        }
        if (!binding.device.empty()) {
            if (tag->kind == PlcValueKind::Int) {
                scene.upsertSignal(binding.device, SignalType::Int).integer = tag->forced ? tag->forceInteger : tag->integer;
            } else if (tag->kind == PlcValueKind::Real) {
                scene.upsertSignal(binding.device, SignalType::Real).real = tag->forced ? tag->forceReal : tag->real;
            } else {
                scene.upsertSignal(binding.device, SignalType::Bool).flag = tagBool(tag);
            }
        }
        if (!binding.opcNode.empty() && (tag->kind == PlcValueKind::Bool || tag->kind == PlcValueKind::Timer)) {
            const bool value = tagBool(tag);
            writeOpc(binding.opcNode, value);
        }
    }
    for (const PlcTag& tag : doc.tags) {
        if (tag.kind != PlcValueKind::Timer) {
            continue;
        }
        scene.upsertSignal("plc_" + tag.address, SignalType::Bool).flag = tagBool(&tag);
    }
}

void SoftPlcRuntime::scan(Scene& scene, double simTime, double dt) {
    if (!running_ || !scene.softPlc.enabled || scene.softPlc.backend == "plcsim") {
        return;
    }
    const double period = std::max(scene.softPlc.scanSeconds, 1e-4);
    acc_ += std::max(dt, 0.0);
    int burst = 0;
    while (acc_ + 1e-12 >= period && burst < 8) {
        acc_ -= period;
        oneScan(scene, simTime, period);
        ++burst;
    }
}

bool PlcSimAdvancedBackend::start(const SoftPlcDocument&, std::string& error) {
    error = "PLCSIM Advanced Simulation Runtime is not implemented";
    log_.clear();
    log_.push_back(error);
    return false;
}

void PlcSimAdvancedBackend::stop() { log_.clear(); }

void PlcSimAdvancedBackend::scan(Scene&, double, double) {}

}  // namespace sim
