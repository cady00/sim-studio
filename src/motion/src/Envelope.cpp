#include "sim/motion/Envelope.hpp"

#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

Quat alignZ(Vec3 zAxis) {
    zAxis = normalize(zAxis);
    if (length(zAxis) < 0.5) {
        return Quat::identity();
    }
    const Vec3 from{0, 0, 1};
    const Vec3 axis = cross(from, zAxis);
    const double sine = length(axis);
    const double cosine = dot(from, zAxis);
    if (sine < 1e-8) {
        return cosine > 0 ? Quat::identity() : Quat::fromAxisAngle({1, 0, 0}, kPi);
    }
    return Quat::fromAxisAngle(axis / sine, std::atan2(sine, cosine));
}

void upsertPair(ProximitySetup& setup, const std::string& a, const std::string& b) {
    for (const auto& pair : setup.pairs) {
        if ((pair.first == a && pair.second == b) || (pair.first == b && pair.second == a)) {
            return;
        }
    }
    setup.pairs.emplace_back(a, b);
}

}  // namespace

MotionEnvelope buildTcpEnvelope(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& joints,
                                const std::string& name, double radiusMm) {
    MotionEnvelope envelope;
    envelope.name = name;
    envelope.resource = resource.name;
    const std::vector<double> saved = jointVector(resource);
    const double radius = std::max(10.0, radiusMm);
    for (const std::vector<double>& sample : joints) {
        if (sample.empty()) {
            continue;
        }
        setJointVector(resource, sample);
        envelope.samples.push_back(forwardModel(resource).position);
        envelope.bounds.expand(envelope.samples.back());
    }
    setJointVector(resource, saved);
    if (envelope.samples.empty()) {
        return envelope;
    }
    CollisionSet set;
    set.name = name;
    for (std::size_t i = 1; i < envelope.samples.size(); ++i) {
        const Vec3 from = envelope.samples[i - 1];
        const Vec3 to = envelope.samples[i];
        Vec3 direction = to - from;
        const double span = length(direction);
        if (span < 1.0) {
            direction = {0, 0, 1};
        } else {
            direction = direction / span;
        }
        const std::string nodeName = name + "_" + std::to_string(i);
        if (auto existing = scene.findByName(nodeName)) {
            scene.removeNode(existing->id);
        }
        auto node = scene.makeNode(NodeKind::Part, nodeName);
        node->meshId = scene.addMesh(makeBoxMesh({radius, radius, span * 0.5 + radius}));
        node->local.translation = (from + to) * 0.5;
        node->local.rotation = alignZ(direction);
        node->color = {0.25, 0.48, 0.86};
        scene.addChild(scene.root(), node);
        envelope.nodes.push_back(nodeName);
        set.objects.push_back(nodeName);
        envelope.bounds.expand(from);
        envelope.bounds.expand(to);
    }
    if (envelope.nodes.empty() && !envelope.samples.empty()) {
        const std::string nodeName = name + "_0";
        auto node = scene.makeNode(NodeKind::Part, nodeName);
        node->meshId = scene.addMesh(makeBoxMesh({radius, radius, radius}));
        node->local.translation = envelope.samples.front();
        node->color = {0.25, 0.48, 0.86};
        scene.addChild(scene.root(), node);
        envelope.nodes.push_back(nodeName);
        set.objects.push_back(nodeName);
    }
    if (!set.objects.empty()) {
        bool replaced = false;
        for (CollisionSet& existing : scene.proximity.sets) {
            if (existing.name == set.name) {
                existing = set;
                replaced = true;
            }
        }
        if (!replaced) {
            scene.proximity.sets.push_back(set);
        }
        bool fixture = false;
        for (const CollisionSet& existing : scene.proximity.sets) {
            if (existing.name == "fixture") {
                fixture = true;
            }
        }
        if (fixture) {
            upsertPair(scene.proximity, set.name, "fixture");
        }
    }
    bool stored = false;
    for (MotionEnvelope& existing : scene.envelopes) {
        if (existing.name == envelope.name) {
            existing = envelope;
            stored = true;
        }
    }
    if (!stored) {
        scene.envelopes.push_back(envelope);
    }
    return envelope;
}

std::vector<InterlockHit> evaluateInterlocks(Scene& scene) {
    std::vector<InterlockHit> hits;
    for (const InterlockZone& zone : scene.interlocks) {
        std::vector<std::string> inside;
        for (const std::string& name : zone.resources) {
            const KinematicResource* resource = findResource(scene.resources, name);
            if (resource == nullptr) {
                continue;
            }
            const Vec3 tcp = forwardModel(*resource).position;
            if (length(tcp - zone.center) <= zone.radius) {
                inside.push_back(name);
            }
        }
        Signal& signal = scene.upsertSignal("interlock_" + zone.name, SignalType::Bool);
        signal.flag = inside.size() >= 2;
        for (std::size_t i = 0; i < inside.size(); ++i) {
            for (std::size_t j = i + 1; j < inside.size(); ++j) {
                hits.push_back({zone.name, inside[i], inside[j]});
            }
        }
    }
    return hits;
}

}  // namespace sim
