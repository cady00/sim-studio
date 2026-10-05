#include "sim/motion/SoftPlc.hpp"

#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cctype>
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
    if (tag->kind == PlcValueKind::Timer || tag->kind == PlcValueKind::Counter) {
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
        if (tag.kind == PlcValueKind::Timer) {
            tag.elapsed = 0;
            tag.done = false;
            if (!tag.forced) {
                tag.flag = false;
            }
            continue;
        }
        if (tag.kind == PlcValueKind::Counter) {
            tag.integer = 0;
            tag.done = false;
            if (!tag.forced) {
                tag.flag = false;
            }
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
    rungPower_.clear();
    cycles_.clear();
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
    rungPower_.clear();
    cycles_.clear();
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

namespace {

std::string zoneAddress(const PlcZoneWatch& zone, std::size_t index) {
    if (!zone.address.empty()) {
        return zone.address;
    }
    if (!zone.name.empty()) {
        return zone.name + "_Occupied";
    }
    return "Z" + std::to_string(index + 1) + "_Occupied";
}

std::string collisionAddress(const PlcCollisionWatch& watch, std::size_t index) {
    if (!watch.address.empty()) {
        return watch.address;
    }
    return "R" + std::to_string(index + 1) + "_Collision";
}

PlcTag* ensureBoolTag(SoftPlcDocument& doc, const std::string& address) {
    if (PlcTag* tag = findPlcTag(doc, address)) {
        return tag;
    }
    PlcTag created;
    created.address = address;
    created.kind = PlcValueKind::Bool;
    doc.tags.push_back(std::move(created));
    return findPlcTag(doc, address);
}

bool pointInZone(Vec3 point, const PlcZoneWatch& zone) {
    const Vec3 delta = point - zone.center;
    if (zone.shape == "box") {
        return std::abs(delta.x) <= std::max(zone.size.x, 0.0) && std::abs(delta.y) <= std::max(zone.size.y, 0.0) &&
               std::abs(delta.z) <= std::max(zone.size.z, 0.0);
    }
    if (zone.shape == "cylinder") {
        const double radius = std::max(zone.size.x, 0.0);
        const double halfHeight = std::max(zone.size.z, 0.0);
        return delta.x * delta.x + delta.y * delta.y <= radius * radius && std::abs(delta.z) <= halfHeight;
    }
    return length(delta) <= std::max(zone.radius, 0.0);
}

bool zoneOccupied(const Scene& scene, const PlcZoneWatch& zone) {
    if (zone.watch.empty()) {
        for (const KinematicResource& resource : scene.resources) {
            if (pointInZone(forwardModel(resource).position, zone)) {
                return true;
            }
        }
        return false;
    }
    for (const std::string& name : zone.watch) {
        if (const KinematicResource* resource = findResource(scene.resources, name)) {
            if (pointInZone(forwardModel(*resource).position, zone)) {
                return true;
            }
        }
        if (const auto node = scene.findByName(name)) {
            if (pointInZone(transformPoint(node->worldMatrix(), {}), zone)) {
                return true;
            }
        }
    }
    return false;
}

void writeSafetyBit(Scene& scene, SoftPlcDocument& doc, const std::string& address, bool value) {
    PlcTag* tag = ensureBoolTag(doc, address);
    if (tag != nullptr && !tag->forced) {
        tag->flag = value;
    }
    const bool published = tag == nullptr ? value : tagBool(tag);
    scene.upsertSignal(address, SignalType::Bool).flag = published;
}

}  // namespace

void SoftPlcRuntime::oneScan(Scene& scene, double simTime, double period) {
    SoftPlcDocument& doc = scene.softPlc;
    ++scans_;
    for (const PlcBinding& binding : doc.bindings) {
        if (binding.direction != "in") {
            continue;
        }
        PlcTag* tag = findPlcTag(doc, binding.address);
        if (tag == nullptr || tag->forced || tag->kind == PlcValueKind::Timer || tag->kind == PlcValueKind::Counter) {
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
    for (std::size_t index = 0; index < doc.zones.size(); ++index) {
        writeSafetyBit(scene, doc, zoneAddress(doc.zones[index], index), zoneOccupied(scene, doc.zones[index]));
    }
    for (std::size_t index = 0; index < doc.collisions.size(); ++index) {
        const bool hit = index < collisionLatch_.size() && collisionLatch_[index] != 0;
        writeSafetyBit(scene, doc, collisionAddress(doc.collisions[index], index), hit);
    }

    auto previousLevel = [this](const std::string& address) {
        const Image* image = previous(address);
        return image != nullptr && image->flag;
    };
    auto contactOn = [&](const PlcContact& contact) {
        const bool now = tagBool(findPlcTag(doc, contact.address));
        const bool was = previousLevel(contact.address);
        if (contact.edge == "rising") {
            return !was && now;
        }
        if (contact.edge == "falling") {
            return was && !now;
        }
        return contact.normallyOpen ? now : !now;
    };
    auto termOn = [&](const PlcContact& contact) {
        if (contactOn(contact)) {
            return true;
        }
        for (const PlcContact& alternative : contact.alternatives) {
            if (contactOn(alternative)) {
                return true;
            }
        }
        return false;
    };
    auto branchOn = [&](const std::vector<PlcContact>& series) {
        for (const PlcContact& contact : series) {
            if (!termOn(contact)) {
                return false;
            }
        }
        return true;
    };
    auto rungOn = [&](const PlcRung& rung) {
        if (rung.parallel.empty()) {
            return branchOn(rung.series);
        }
        bool any = false;
        bool saw = false;
        if (!rung.series.empty()) {
            any = branchOn(rung.series);
            saw = true;
        }
        for (const std::vector<PlcContact>& branch : rung.parallel) {
            if (branch.empty()) {
                continue;
            }
            any = any || branchOn(branch);
            saw = true;
        }
        return saw ? any : true;
    };

    std::size_t rungIndex = 0;
    for (const PlcRung& rung : doc.rungs) {
        const bool power = rungOn(rung);
        const bool wasPower = rungIndex < rungPower_.size() && rungPower_[rungIndex] != 0;
        if (rungPower_.size() <= rungIndex) {
            rungPower_.resize(rungIndex + 1, 0);
        }
        rungPower_[rungIndex] = power ? 1 : 0;
        ++rungIndex;
        PlcTag* coil = findPlcTag(doc, rung.coil);
        if (coil == nullptr || coil->forced) {
            continue;
        }
        if (rung.action == "ctu" && coil->kind == PlcValueKind::Counter) {
            if (power && !wasPower) {
                coil->integer += 1;
            }
            const auto preset = static_cast<std::int64_t>(std::llround(std::max(coil->preset, 0.0)));
            coil->done = coil->integer >= preset;
            coil->flag = coil->done;
            continue;
        }
        if (rung.action == "ctd" && coil->kind == PlcValueKind::Counter) {
            if (power && !wasPower) {
                coil->integer -= 1;
            }
            coil->done = coil->integer <= 0;
            coil->flag = coil->done;
            continue;
        }
        if (rung.action == "ctr" && coil->kind == PlcValueKind::Counter) {
            if (power) {
                coil->integer = 0;
                const auto preset = static_cast<std::int64_t>(std::llround(std::max(coil->preset, 0.0)));
                coil->done = coil->integer >= preset;
                coil->flag = coil->done;
            }
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
        if (rung.action == "tof" && coil->kind == PlcValueKind::Timer) {
            if (power) {
                coil->elapsed = 0;
                coil->done = true;
            } else if (coil->done) {
                coil->elapsed += period;
                if (coil->elapsed + 1e-6 >= std::max(coil->preset, 0.0)) {
                    coil->done = false;
                }
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
            } else if (tag.kind == PlcValueKind::Counter) {
                changed = before->integer != integer || before->done != tag.done;
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
        if (tag.kind == PlcValueKind::Int || tag.kind == PlcValueKind::Counter) {
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

    trackCycles(scene, simTime);

    previous_.clear();
    for (const PlcTag& tag : doc.tags) {
        Image image;
        image.address = tag.address;
        image.flag = tagBool(&tag);
        image.integer = tag.forced && tag.kind != PlcValueKind::Bool ? tag.forceInteger : tag.integer;
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
            if (tag->kind == PlcValueKind::Int || tag->kind == PlcValueKind::Counter) {
                scene.upsertSignal(binding.device, SignalType::Int).integer = tag->forced ? tag->forceInteger : tag->integer;
            } else if (tag->kind == PlcValueKind::Real) {
                scene.upsertSignal(binding.device, SignalType::Real).real = tag->forced ? tag->forceReal : tag->real;
            } else {
                scene.upsertSignal(binding.device, SignalType::Bool).flag = tagBool(tag);
            }
        }
        if (!binding.opcNode.empty() &&
            (tag->kind == PlcValueKind::Bool || tag->kind == PlcValueKind::Timer || tag->kind == PlcValueKind::Counter)) {
            const bool value = tagBool(tag);
            writeOpc(binding.opcNode, value);
        }
    }
    for (const PlcTag& tag : doc.tags) {
        if (tag.kind != PlcValueKind::Timer && tag.kind != PlcValueKind::Counter) {
            continue;
        }
        scene.upsertSignal("plc_" + tag.address, SignalType::Bool).flag = tagBool(&tag);
    }
}

void SoftPlcRuntime::trackCycles(Scene& scene, double simTime) {
    auto level = [&](const std::string& address) { return tagBool(findPlcTag(scene.softPlc, address)); };
    auto edged = [&](const std::string& address, const std::string& which) {
        if (address.empty()) {
            return false;
        }
        const bool now = level(address);
        const Image* before = previous(address);
        const bool was = before != nullptr && before->flag;
        if (which == "falling") {
            return was && !now;
        }
        return !was && now;
    };
    auto publish = [&](const PlcCycleRule& rule, const CycleTrace& trace) {
        const std::string prefix = rule.id + "_";
        scene.operations.erase(std::remove_if(scene.operations.begin(), scene.operations.end(),
                                              [&](const Operation& operation) {
                                                  return operation.id == rule.id ||
                                                         (operation.id.size() > prefix.size() &&
                                                          operation.id.compare(0, prefix.size(), prefix) == 0);
                                              }),
                               scene.operations.end());
        if (!trace.open && !trace.finished) {
            return;
        }
        const std::string stationId = rule.station.empty() ? "st_plc" : rule.station;
        if (findStation(scene.layout, stationId) == nullptr) {
            Station station;
            station.id = stationId;
            station.name = rule.stationName.empty() ? (rule.name.empty() ? stationId : rule.name) : rule.stationName;
            station.kind = StationKind::Robot;
            scene.layout.stations.push_back(std::move(station));
        }
        if (scene.layout.title.empty()) {
            scene.layout.title = "软 PLC 节拍";
        }
        const double end = trace.finished ? trace.endTime : simTime;
        std::vector<CycleSpan> blocks = trace.blocks;
        if (trace.blocking && end > trace.blockStart + 1e-4) {
            blocks.push_back(CycleSpan{trace.blockStart, end});
        }
        struct Segment {
            double start = 0;
            double finish = 0;
            bool block = false;
        };
        std::vector<Segment> segments;
        double cursor = trace.startTime;
        for (const CycleSpan& block : blocks) {
            if (block.start > cursor + 1e-4) {
                segments.push_back(Segment{cursor, block.start, false});
            }
            if (block.end > block.start + 1e-4) {
                segments.push_back(Segment{block.start, block.end, true});
            }
            cursor = std::max(cursor, block.end);
        }
        if (end > cursor + 1e-4) {
            segments.push_back(Segment{cursor, end, false});
        }
        if (segments.empty()) {
            return;
        }
        const std::string jobName = rule.name.empty() ? "取放" : rule.name;
        Operation parent;
        parent.id = rule.id;
        parent.name = jobName;
        parent.kind = OpKind::Sequence;
        parent.station = stationId;
        parent.timeSource = "plc";
        parent.timeNote = "软 PLC 边沿";
        scene.operations.push_back(parent);
        for (std::size_t index = 0; index < segments.size(); ++index) {
            const Segment& segment = segments[index];
            Operation task;
            task.id = prefix + std::to_string(index);
            task.name = segment.block ? "干涉阻塞" : jobName;
            task.parent = rule.id;
            task.kind = OpKind::Task;
            task.station = stationId;
            task.index = static_cast<int>(index);
            task.seconds = segment.finish - segment.start;
            task.wait = index == 0 ? segment.start : 0;
            task.timeSource = "plc";
            task.timeNote = segment.block ? "干涉阻塞" : "软 PLC 边沿";
            scene.operations.push_back(std::move(task));
        }
    };

    for (const PlcCycleRule& rule : scene.softPlc.cycles) {
        if (rule.id.empty() || rule.startSignal.empty()) {
            continue;
        }
        CycleTrace* trace = nullptr;
        for (CycleTrace& candidate : cycles_) {
            if (candidate.id == rule.id) {
                trace = &candidate;
                break;
            }
        }
        if (trace == nullptr) {
            cycles_.push_back(CycleTrace{});
            cycles_.back().id = rule.id;
            trace = &cycles_.back();
        }
        const bool busyNow = !rule.busySignal.empty() && level(rule.busySignal);
        if (trace->open && busyNow) {
            trace->sawBusy = true;
        }
        if (!trace->open && !trace->finished && edged(rule.startSignal, rule.startEdge)) {
            trace->open = true;
            trace->finished = false;
            trace->startTime = simTime;
            trace->sawBusy = busyNow;
            trace->blocking = false;
            trace->blocks.clear();
        }
        if (trace->open && !rule.blockSignal.empty()) {
            const bool blocked = level(rule.blockSignal);
            if (blocked && !trace->blocking) {
                trace->blocking = true;
                trace->blockStart = simTime;
            } else if (!blocked && trace->blocking) {
                trace->blocks.push_back(CycleSpan{trace->blockStart, simTime});
                trace->blocking = false;
            }
        }
        const bool endReady = rule.busySignal.empty() || trace->sawBusy;
        if (trace->open && endReady && edged(rule.endSignal, rule.endEdge)) {
            if (trace->blocking) {
                trace->blocks.push_back(CycleSpan{trace->blockStart, simTime});
                trace->blocking = false;
            }
            trace->open = false;
            trace->finished = true;
            trace->endTime = simTime;
        }
        publish(rule, *trace);
    }
}

void SoftPlcRuntime::scan(Scene& scene, double simTime, double dt) {
    if (!running_ || !scene.softPlc.enabled || scene.softPlc.backend == "plcsim") {
        return;
    }
    collisionLatch_.assign(scene.softPlc.collisions.size(), 0);
    for (std::size_t index = 0; index < scene.softPlc.collisions.size(); ++index) {
        const PlcCollisionWatch& watch = scene.softPlc.collisions[index];
        bool hit = false;
        if (watch.resource.empty()) {
            for (const KinematicResource& resource : scene.resources) {
                if (!collidingPairs(scene, resource).empty()) {
                    hit = true;
                    break;
                }
            }
        } else if (const KinematicResource* resource = findResource(scene.resources, watch.resource)) {
            hit = !collidingPairs(scene, *resource).empty();
        }
        collisionLatch_[index] = hit ? 1 : 0;
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

namespace {

bool jointIsExternal(const KinJoint& joint) {
    return !joint.externalName.empty() || joint.externalKind == "track" || joint.externalKind == "positioner";
}

int nextRobotIndex(const SoftPlcDocument& doc) {
    int next = 1;
    for (const PlcTag& tag : doc.tags) {
        if (tag.address.size() < 4 || tag.address[0] != 'R') {
            continue;
        }
        std::size_t cursor = 1;
        int value = 0;
        bool digits = false;
        while (cursor < tag.address.size() && std::isdigit(static_cast<unsigned char>(tag.address[cursor]))) {
            digits = true;
            value = value * 10 + (tag.address[cursor] - '0');
            ++cursor;
        }
        if (digits && cursor < tag.address.size() && tag.address[cursor] == '_') {
            next = std::max(next, value + 1);
        }
    }
    return next;
}

void ensureDraft(SoftPlcDocument& doc, const std::string& address, const std::string& direction) {
    if (findPlcTag(doc, address) == nullptr) {
        PlcTag tag;
        tag.address = address;
        tag.kind = PlcValueKind::Bool;
        doc.tags.push_back(std::move(tag));
    }
    for (const PlcBinding& binding : doc.bindings) {
        if (binding.address == address) {
            return;
        }
    }
    PlcBinding binding;
    binding.address = address;
    binding.direction = direction;
    doc.bindings.push_back(std::move(binding));
}

std::string lowerName(std::string text) {
    for (char& letter : text) {
        letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
    }
    return text;
}

}  // namespace

void appendRobotPlcDraft(SoftPlcDocument& doc, const KinematicResource& resource) {
    std::vector<std::string> axes;
    bool arm = false;
    for (const KinJoint& joint : resource.joints) {
        if (!jointIsExternal(joint)) {
            arm = true;
            continue;
        }
        const std::string name = joint.externalName.empty() ? std::string("E1") : joint.externalName;
        if (std::find(axes.begin(), axes.end(), name) == axes.end()) {
            axes.push_back(name);
        }
    }
    const std::string lower = lowerName(resource.name);
    const bool namedTrack = lower.rfind("irbt", 0) == 0;
    if (axes.empty() && namedTrack) {
        axes.push_back("E1");
    }
    const bool externalOnly = !arm && !axes.empty();
    if (!externalOnly) {
        const std::string prefix = "R" + std::to_string(nextRobotIndex(doc)) + "_";
        ensureDraft(doc, prefix + "AtHome", "in");
        ensureDraft(doc, prefix + "Busy", "in");
        ensureDraft(doc, prefix + "Fault", "in");
        ensureDraft(doc, prefix + "Start", "out");
        ensureDraft(doc, prefix + "Grip", "out");
    }
    for (const std::string& axis : axes) {
        ensureDraft(doc, axis + "_InPos", "in");
    }
}

namespace {

std::string csvCell(const std::string& text) {
    if (text.find_first_of(",\"\n\r") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (char letter : text) {
        if (letter == '"') {
            escaped += "\"\"";
        } else {
            escaped += letter;
        }
    }
    escaped += '"';
    return escaped;
}

}  // namespace

std::string softPlcScanCsv(const std::vector<std::string>& log) {
    std::string csv = "time,scan,signal,value\n";
    for (const std::string& line : log) {
        if (line.size() < 4 || line.compare(0, 2, "t=") != 0) {
            continue;
        }
        const std::size_t scanMark = line.find(" #");
        if (scanMark == std::string::npos) {
            continue;
        }
        const std::string time = line.substr(2, scanMark - 2);
        std::size_t cursor = scanMark + 2;
        std::size_t scanEnd = cursor;
        while (scanEnd < line.size() && line[scanEnd] != ' ') {
            ++scanEnd;
        }
        const std::string scan = line.substr(cursor, scanEnd - cursor);
        cursor = scanEnd;
        while (cursor < line.size()) {
            while (cursor < line.size() && line[cursor] == ' ') {
                ++cursor;
            }
            if (cursor >= line.size()) {
                break;
            }
            const std::size_t tokenEnd = line.find(' ', cursor);
            const std::size_t end = tokenEnd == std::string::npos ? line.size() : tokenEnd;
            const std::string token = line.substr(cursor, end - cursor);
            const std::size_t eq = token.find('=');
            if (eq != std::string::npos && eq > 0) {
                csv += csvCell(time);
                csv += ',';
                csv += csvCell(scan);
                csv += ',';
                csv += csvCell(token.substr(0, eq));
                csv += ',';
                csv += csvCell(token.substr(eq + 1));
                csv += '\n';
            }
            cursor = end;
        }
    }
    return csv;
}

}  // namespace sim
