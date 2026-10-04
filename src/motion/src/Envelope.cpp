#include "sim/motion/Envelope.hpp"

#include "sim/core/LineModel.hpp"
#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

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

namespace {

struct HullFace {
    int a = 0;
    int b = 0;
    int c = 0;
};

Vec3 hullNormal(const std::vector<Vec3>& points, HullFace face) {
    return cross(points[static_cast<std::size_t>(face.b)] - points[static_cast<std::size_t>(face.a)],
                 points[static_cast<std::size_t>(face.c)] - points[static_cast<std::size_t>(face.a)]);
}

bool hullOf(const std::vector<Vec3>& raw, std::shared_ptr<MeshData>& mesh) {
    std::vector<Vec3> points;
    for (const Vec3& point : raw) {
        bool duplicate = false;
        for (const Vec3& kept : points) {
            if (length(point - kept) < 0.5) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            points.push_back(point);
        }
    }
    if (points.size() < 4) {
        return false;
    }
    int seed[4] = {-1, -1, -1, -1};
    seed[0] = 0;
    double best = 0;
    for (int i = 1; i < static_cast<int>(points.size()); ++i) {
        const double distance = length(points[static_cast<std::size_t>(i)] - points[0]);
        if (distance > best) {
            best = distance;
            seed[1] = i;
        }
    }
    best = 0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (i == seed[1]) {
            continue;
        }
        const double area = length(cross(points[static_cast<std::size_t>(seed[1])] - points[0], points[static_cast<std::size_t>(i)] - points[0]));
        if (area > best) {
            best = area;
            seed[2] = i;
        }
    }
    best = 0;
    const Vec3 base = cross(points[static_cast<std::size_t>(seed[1])] - points[0], points[static_cast<std::size_t>(seed[2])] - points[0]);
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        const double volume = std::abs(dot(base, points[static_cast<std::size_t>(i)] - points[0]));
        if (volume > best) {
            best = volume;
            seed[3] = i;
        }
    }
    if (seed[1] < 0 || seed[2] < 0 || seed[3] < 0 || best < 1.0) {
        return false;
    }
    std::vector<HullFace> faces = {{seed[0], seed[1], seed[2]}, {seed[0], seed[3], seed[1]}, {seed[0], seed[2], seed[3]}, {seed[1], seed[3], seed[2]}};
    Vec3 center{};
    for (int index : seed) {
        center = center + points[static_cast<std::size_t>(index)];
    }
    center = center / 4.0;
    for (HullFace& face : faces) {
        if (dot(hullNormal(points, face), center - points[static_cast<std::size_t>(face.a)]) > 0) {
            std::swap(face.b, face.c);
        }
    }
    std::vector<char> used(points.size(), 0);
    for (int index : seed) {
        used[static_cast<std::size_t>(index)] = 1;
    }
    for (int index = 0; index < static_cast<int>(points.size()); ++index) {
        if (used[static_cast<std::size_t>(index)]) {
            continue;
        }
        std::vector<char> visible(faces.size(), 0);
        bool outside = false;
        for (std::size_t face = 0; face < faces.size(); ++face) {
            const Vec3 normal = hullNormal(points, faces[face]);
            if (dot(normal, points[static_cast<std::size_t>(index)] - points[static_cast<std::size_t>(faces[face].a)]) > 1e-4) {
                visible[face] = 1;
                outside = true;
            }
        }
        if (!outside) {
            used[static_cast<std::size_t>(index)] = 1;
            continue;
        }
        std::map<std::pair<int, int>, int> edges;
        for (std::size_t face = 0; face < faces.size(); ++face) {
            if (!visible[face]) {
                continue;
            }
            const HullFace& item = faces[face];
            const int loop[4] = {item.a, item.b, item.c, item.a};
            for (int edge = 0; edge < 3; ++edge) {
                edges[{loop[edge], loop[edge + 1]}]++;
            }
        }
        std::vector<HullFace> kept;
        for (std::size_t face = 0; face < faces.size(); ++face) {
            if (!visible[face]) {
                kept.push_back(faces[face]);
            }
        }
        for (const auto& edge : edges) {
            const std::pair<int, int> reverse{edge.first.second, edge.first.first};
            if (edges.count(reverse) != 0) {
                continue;
            }
            kept.push_back({edge.first.second, edge.first.first, index});
        }
        faces = std::move(kept);
        used[static_cast<std::size_t>(index)] = 1;
    }
    if (faces.size() < 4) {
        return false;
    }
    mesh = std::make_shared<MeshData>();
    for (const HullFace& face : faces) {
        Vec3 normal = hullNormal(points, face);
        if (length(normal) < 1e-8) {
            normal = {0, 0, 1};
        } else {
            normal = normal / length(normal);
        }
        const std::size_t baseIndex = mesh->positions.size() / 3;
        for (int index : {face.a, face.b, face.c}) {
            const Vec3& point = points[static_cast<std::size_t>(index)];
            mesh->positions.push_back(static_cast<float>(point.x));
            mesh->positions.push_back(static_cast<float>(point.y));
            mesh->positions.push_back(static_cast<float>(point.z));
            mesh->normals.push_back(static_cast<float>(normal.x));
            mesh->normals.push_back(static_cast<float>(normal.y));
            mesh->normals.push_back(static_cast<float>(normal.z));
        }
        mesh->indices.push_back(static_cast<std::uint32_t>(baseIndex));
        mesh->indices.push_back(static_cast<std::uint32_t>(baseIndex + 1));
        mesh->indices.push_back(static_cast<std::uint32_t>(baseIndex + 2));
    }
    mesh->recomputeBounds();
    return true;
}

std::vector<Vec3> aabbCorners(Vec3 lo, Vec3 hi) {
    std::vector<Vec3> corners;
    for (int i = 0; i < 8; ++i) {
        corners.push_back({(i & 1) != 0 ? hi.x : lo.x, (i & 2) != 0 ? hi.y : lo.y, (i & 4) != 0 ? hi.z : lo.z});
    }
    return corners;
}

void expandLocal(const Scene& scene, const std::shared_ptr<SceneNode>& node, const Mat4& inverseRoot, Aabb& bounds) {
    if (!node) {
        return;
    }
    const auto found = scene.meshes().find(node->meshId);
    if (found != scene.meshes().end() && found->second && found->second->bounds.valid) {
        const Mat4 local = mul(inverseRoot, node->worldMatrix());
        const Aabb& box = found->second->bounds;
        for (const Vec3& corner : aabbCorners(box.min, box.max)) {
            bounds.expand(transformPoint(local, corner));
        }
    }
    for (const auto& child : node->children) {
        expandLocal(scene, child, inverseRoot, bounds);
    }
}

void segmentCorners(Vec3 from, Vec3 to, double radius, std::vector<Vec3>& out) {
    Vec3 direction = to - from;
    const double span = length(direction);
    if (span < 1.0) {
        direction = {0, 0, 1};
    } else {
        direction = direction / span;
    }
    Vec3 side = std::abs(direction.z) < 0.9 ? cross(direction, {0, 0, 1}) : cross(direction, {1, 0, 0});
    side = normalize(side);
    const Vec3 up = normalize(cross(direction, side));
    const Vec3 mid = (from + to) * 0.5;
    const double half = std::max(span * 0.5, 1.0);
    for (int i = 0; i < 8; ++i) {
        Vec3 point = mid;
        point = point + direction * ((i & 1) != 0 ? half : -half);
        point = point + side * ((i & 2) != 0 ? radius : -radius);
        point = point + up * ((i & 4) != 0 ? radius : -radius);
        out.push_back(point);
    }
}

void analyticSegments(const KinematicResource& resource, std::vector<std::vector<Vec3>>& posed) {
    if (resource.ik != "spherical_wrist" || resource.joints.size() < 6) {
        return;
    }
    double q[6];
    for (int i = 0; i < 6; ++i) {
        q[i] = deg2rad(jointCommand(resource, resource.joints[static_cast<std::size_t>(i)]));
    }
    const SphericalWrist& arm = resource.wrist;
    const double c1 = std::cos(q[0]);
    const double s1 = std::sin(q[0]);
    const double c2 = std::cos(q[1]);
    const double s2 = std::sin(q[1]);
    const double c23 = std::cos(q[1] + q[2]);
    const double s23 = std::sin(q[1] + q[2]);
    const Vec3 shoulder{c1 * arm.shoulder, s1 * arm.shoulder, arm.d1};
    const double elbowRadial = arm.shoulder + arm.a2 * c2;
    const Vec3 elbow{c1 * elbowRadial, s1 * elbowRadial, arm.d1 - arm.a2 * s2};
    const double wristRadial = arm.shoulder + arm.a2 * c2 + arm.a3 * c23;
    const Vec3 wrist{c1 * wristRadial, s1 * wristRadial, arm.d1 - (arm.a2 * s2 + arm.a3 * s23)};
    const Quat rotation = Quat::fromAxisAngle({0, 0, 1}, q[0]) * Quat::fromAxisAngle({0, 1, 0}, q[1] + q[2]) *
                          Quat::fromAxisAngle({1, 0, 0}, q[3]) * Quat::fromAxisAngle({0, 1, 0}, q[4]) *
                          Quat::fromAxisAngle({0, 0, 1}, q[5]);
    const Vec3 flange = wrist + rotation.rotate({0, 0, arm.d6});
    const Vec3 pairs[4][2] = {{{0, 0, 0}, {0, 0, arm.d1}}, {shoulder, elbow}, {elbow, wrist}, {wrist, flange}};
    const char* names[] = {"base", "upper", "fore", "tool"};
    (void)names;
    for (const auto& pair : pairs) {
        std::vector<Vec3> corners;
        segmentCorners(pair[0], pair[1], 70.0, corners);
        posed.push_back(std::move(corners));
    }
}

std::vector<std::vector<double>> densifyJoints(const std::vector<std::vector<double>>& joints) {
    std::vector<std::vector<double>> dense;
    if (joints.empty()) {
        return dense;
    }
    dense.push_back(joints.front());
    for (std::size_t i = 1; i < joints.size(); ++i) {
        const std::vector<double>& a = joints[i - 1];
        const std::vector<double>& b = joints[i];
        const std::size_t n = std::min(a.size(), b.size());
        double maxDelta = 0;
        for (std::size_t k = 0; k < n; ++k) {
            maxDelta = std::max(maxDelta, std::abs(a[k] - b[k]));
        }
        const int steps = std::max(1, static_cast<int>(std::ceil(maxDelta / 20.0)));
        for (int step = 1; step <= steps; ++step) {
            const double t = static_cast<double>(step) / static_cast<double>(steps);
            std::vector<double> sample(std::max(a.size(), b.size()), 0);
            for (std::size_t k = 0; k < sample.size(); ++k) {
                const double av = k < a.size() ? a[k] : 0;
                const double bv = k < b.size() ? b[k] : 0;
                sample[k] = av + (bv - av) * t;
            }
            dense.push_back(std::move(sample));
        }
    }
    return dense;
}

void rememberEnvelope(Scene& scene, MotionEnvelope& envelope, CollisionSet set) {
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
}

}  // namespace

MotionEnvelope buildLinkSweep(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& joints,
                              const std::string& name) {
    MotionEnvelope envelope;
    envelope.name = name;
    envelope.resource = resource.name;
    const std::vector<double> saved = jointVector(resource);
    struct Solid {
        std::string name;
        std::string nodeId;
        std::vector<Vec3> local;
    };
    std::vector<Solid> solids;
    for (const KinLink& link : resource.links) {
        auto node = scene.find(link.nodeId);
        if (!node) {
            continue;
        }
        Aabb local;
        expandLocal(scene, node, inverseAffine(node->worldMatrix()), local);
        if (!local.valid) {
            continue;
        }
        Solid solid;
        solid.name = link.name.empty() ? node->name : link.name;
        solid.nodeId = link.nodeId;
        solid.local = aabbCorners(local.min, local.max);
        solids.push_back(std::move(solid));
    }
    const bool analytic = solids.empty();
    const std::vector<std::vector<double>> samples = densifyJoints(joints);
    std::vector<std::vector<std::vector<Vec3>>> posed(std::max<std::size_t>(solids.size(), 1));
    if (!analytic) {
        posed.assign(solids.size(), {});
    }
    for (const std::vector<double>& sample : samples) {
        if (sample.empty()) {
            continue;
        }
        setJointVector(resource, sample);
        envelope.samples.push_back(forwardModel(resource).position);
        envelope.bounds.expand(envelope.samples.back());
        if (analytic) {
            std::vector<std::vector<Vec3>> segments;
            analyticSegments(resource, segments);
            if (posed.size() < segments.size()) {
                posed.resize(segments.size());
            }
            for (std::size_t i = 0; i < segments.size(); ++i) {
                posed[i].push_back(std::move(segments[i]));
            }
            continue;
        }
        applyResource(scene, resource);
        for (std::size_t i = 0; i < solids.size(); ++i) {
            auto node = scene.find(solids[i].nodeId);
            if (!node) {
                posed[i].push_back({});
                continue;
            }
            const Mat4 world = node->worldMatrix();
            std::vector<Vec3> corners;
            for (const Vec3& corner : solids[i].local) {
                corners.push_back(transformPoint(world, corner));
            }
            posed[i].push_back(std::move(corners));
        }
    }
    setJointVector(resource, saved);
    applyResource(scene, resource);
    CollisionSet set;
    set.name = name;
    const char* analyticNames[] = {"base", "upper", "fore", "tool"};
    for (std::size_t link = 0; link < posed.size(); ++link) {
        if (posed[link].size() < 2) {
            continue;
        }
        const std::string linkName = analytic ? analyticNames[std::min<std::size_t>(link, 3)] : solids[link].name;
        for (std::size_t i = 1; i < posed[link].size(); ++i) {
            std::vector<Vec3> cloud = posed[link][i - 1];
            cloud.insert(cloud.end(), posed[link][i].begin(), posed[link][i].end());
            std::shared_ptr<MeshData> hull;
            const std::string nodeName = name + "_" + linkName + "_" + std::to_string(i);
            if (auto existing = scene.findByName(nodeName)) {
                scene.removeNode(existing->id);
            }
            auto node = scene.makeNode(NodeKind::Part, nodeName);
            if (!hullOf(cloud, hull)) {
                Aabb box;
                for (const Vec3& point : cloud) {
                    box.expand(point);
                }
                const Vec3 half = box.valid ? box.size() * 0.5 : Vec3{20, 20, 20};
                node->meshId = scene.addMesh(makeBoxMesh({std::max(half.x, 5.0), std::max(half.y, 5.0), std::max(half.z, 5.0)}));
                node->local.translation = box.valid ? box.center() : Vec3{};
            } else {
                node->meshId = scene.addMesh(hull);
            }
            node->color = {0.18, 0.42, 0.78};
            scene.addChild(scene.root(), node);
            envelope.nodes.push_back(nodeName);
            set.objects.push_back(nodeName);
            for (const Vec3& point : cloud) {
                envelope.bounds.expand(point);
            }
        }
    }
    rememberEnvelope(scene, envelope, set);
    return envelope;
}

double applyInterlockHold(Scene& scene) {
    const std::vector<InterlockHit> hits = evaluateInterlocks(scene);
    if (hits.empty()) {
        return 0;
    }
    double release = 1.5;
    for (const InterlockZone& zone : scene.interlocks) {
        if (zone.name == hits.front().zone) {
            release = zone.releaseSeconds;
        }
    }
    for (Operation& operation : scene.operations) {
        if (operation.kind != OpKind::Task) {
            continue;
        }
        const Station* station = findStation(scene.layout, operation.station);
        if (station == nullptr) {
            continue;
        }
        bool involved = false;
        for (const InterlockHit& hit : hits) {
            if (station->resource == hit.a || station->resource == hit.b) {
                involved = true;
            }
        }
        if (!involved) {
            continue;
        }
        if (operation.timeNote.find("interlock") != std::string::npos) {
            return 0;
        }
        operation.wait += release;
        if (!operation.timeNote.empty()) {
            operation.timeNote += "; ";
        }
        operation.timeNote += "interlock hold";
        return release;
    }
    return 0;
}

}  // namespace sim
