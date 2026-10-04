#include "sim/motion/Motion.hpp"

#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace sim {
namespace {

struct PartInfo {
    std::string id;
    Vec3 center;
    double diag = 0;
    bool large = false;
};

struct Cluster {
    Vec3 direction{0, 0, 1};
    Vec3 anchor;
    Vec3 centroid;
    double radius = 0;
    int count = 0;
};

double lineDistance(Vec3 point, Vec3 origin, Vec3 direction) {
    const Vec3 delta = point - origin;
    return length(delta - direction * dot(delta, direction));
}

Vec3 closestOnLine(Vec3 point, Vec3 origin, Vec3 direction) {
    return origin + direction * dot(point - origin, direction);
}

std::vector<PartInfo> collectParts(const Scene& scene) {
    std::vector<PartInfo> parts;
    if (!scene.root()) {
        return parts;
    }
    std::vector<std::shared_ptr<SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->kind != NodeKind::Part || node->meshId.empty()) {
            continue;
        }
        const auto mesh = scene.mesh(node->meshId);
        if (!mesh || !mesh->bounds.valid) {
            continue;
        }
        PartInfo info;
        info.id = node->id;
        info.center = transformPoint(node->worldMatrix(), mesh->bounds.center());
        info.diag = length(mesh->bounds.size());
        info.large = info.diag >= 80.0;
        parts.push_back(info);
    }
    return parts;
}

std::vector<Cluster> collectAxes(const Scene& scene, const std::vector<PartInfo>& parts) {
    struct Sample {
        Vec3 origin;
        Vec3 direction;
        double radius = 0;
        Vec3 center;
    };
    std::vector<Sample> samples;
    for (const PartInfo& part : parts) {
        auto node = scene.find(part.id);
        if (!node) {
            continue;
        }
        const Mat4 world = node->worldMatrix();
        for (const FaceAxis& axis : node->faceAxes) {
            if (axis.radius < 40.0) {
                continue;
            }
            Sample sample;
            sample.origin = transformPoint(world, axis.origin);
            sample.direction = normalize(transformVector(world, axis.direction));
            sample.radius = axis.radius;
            sample.center = part.center;
            if (length(sample.direction) < 0.5) {
                continue;
            }
            samples.push_back(sample);
        }
    }
    std::vector<Cluster> clusters;
    std::vector<std::vector<Vec3>> centers;
    for (const Sample& sample : samples) {
        int found = -1;
        for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
            const double align = std::abs(dot(sample.direction, clusters[static_cast<std::size_t>(i)].direction));
            if (align < 0.985) {
                continue;
            }
            if (lineDistance(sample.origin, clusters[static_cast<std::size_t>(i)].anchor, clusters[static_cast<std::size_t>(i)].direction) < 25.0) {
                found = i;
                break;
            }
        }
        if (found < 0) {
            Cluster cluster;
            cluster.direction = sample.direction;
            cluster.anchor = sample.origin;
            cluster.centroid = sample.center;
            cluster.radius = sample.radius;
            cluster.count = 1;
            clusters.push_back(cluster);
            centers.push_back({sample.center});
        } else {
            Cluster& cluster = clusters[static_cast<std::size_t>(found)];
            cluster.radius = std::max(cluster.radius, sample.radius);
            ++cluster.count;
            centers[static_cast<std::size_t>(found)].push_back(sample.center);
        }
    }
    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        Vec3 sum;
        for (const Vec3& center : centers[static_cast<std::size_t>(i)]) {
            sum = sum + center;
        }
        if (!centers[static_cast<std::size_t>(i)].empty()) {
            clusters[static_cast<std::size_t>(i)].centroid = sum / static_cast<double>(centers[static_cast<std::size_t>(i)].size());
        }
        clusters[static_cast<std::size_t>(i)].anchor =
            closestOnLine(clusters[static_cast<std::size_t>(i)].centroid, clusters[static_cast<std::size_t>(i)].anchor,
                          clusters[static_cast<std::size_t>(i)].direction);
        if (clusters[static_cast<std::size_t>(i)].direction.z < 0 ||
            (std::abs(clusters[static_cast<std::size_t>(i)].direction.z) < 0.5 &&
             clusters[static_cast<std::size_t>(i)].direction.y < 0) ||
            (std::abs(clusters[static_cast<std::size_t>(i)].direction.z) < 0.5 &&
             std::abs(clusters[static_cast<std::size_t>(i)].direction.y) < 0.5 &&
             clusters[static_cast<std::size_t>(i)].direction.x < 0)) {
            clusters[static_cast<std::size_t>(i)].direction = -clusters[static_cast<std::size_t>(i)].direction;
        }
    }
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [](const Cluster& cluster) {
                       const double aligned = std::max(std::abs(cluster.direction.x),
                                                       std::max(std::abs(cluster.direction.y), std::abs(cluster.direction.z)));
                       return cluster.count < 4 || cluster.radius < 40.0 || aligned < 0.97;
                   }),
                   clusters.end());
    return clusters;
}

double parallelGap(const Cluster& a, const Cluster& b) {
    if (std::abs(dot(a.direction, b.direction)) < 0.985) {
        return 1e9;
    }
    const Vec3 delta = b.anchor - a.anchor;
    return length(delta - a.direction * dot(delta, a.direction));
}

// Keeps the strongest axis in a parallel bundle. A counterbalance beside a joint is dropped;
// the elbow, which is parallel to the shoulder but about a metre away, stays.
std::vector<Cluster> selectJoints(std::vector<Cluster> clusters) {
    std::vector<int> rank(clusters.size());
    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        rank[static_cast<std::size_t>(i)] = i;
    }
    std::sort(rank.begin(), rank.end(), [&](int a, int b) {
        const Cluster& left = clusters[static_cast<std::size_t>(a)];
        const Cluster& right = clusters[static_cast<std::size_t>(b)];
        return left.count * left.radius > right.count * right.radius;
    });
    std::vector<Cluster> chosen;
    for (int index : rank) {
        const Cluster& candidate = clusters[static_cast<std::size_t>(index)];
        bool suppressed = false;
        for (const Cluster& kept : chosen) {
            if (parallelGap(candidate, kept) < 220.0) {
                suppressed = true;
                break;
            }
        }
        if (suppressed) {
            continue;
        }
        chosen.push_back(candidate);
        if (chosen.size() == 6) {
            break;
        }
    }
    return chosen;
}

std::vector<Cluster> orderChain(std::vector<Cluster> clusters) {
    if (clusters.empty()) {
        return clusters;
    }
    int vertical = -1;
    double best = -1;
    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        if (std::abs(clusters[static_cast<std::size_t>(i)].direction.z) < 0.85) {
            continue;
        }
        const double score = clusters[static_cast<std::size_t>(i)].count * clusters[static_cast<std::size_t>(i)].radius;
        if (score > best) {
            best = score;
            vertical = i;
        }
    }
    if (vertical < 0) {
        vertical = static_cast<int>(std::max_element(clusters.begin(), clusters.end(), [](const Cluster& a, const Cluster& b) {
                                        return a.count * a.radius < b.count * b.radius;
                                    }) -
                                    clusters.begin());
    }
    std::vector<Cluster> chain;
    std::vector<int> used(clusters.size(), 0);
    chain.push_back(clusters[static_cast<std::size_t>(vertical)]);
    used[static_cast<std::size_t>(vertical)] = 1;
    while (chain.size() < clusters.size() && chain.size() < 6) {
        int pick = -1;
        double nearest = 1e300;
        for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
            if (used[static_cast<std::size_t>(i)]) {
                continue;
            }
            const double distance = length(clusters[static_cast<std::size_t>(i)].anchor - chain.back().anchor);
            if (distance < 30.0) {
                continue;
            }
            if (distance < nearest) {
                nearest = distance;
                pick = i;
            }
        }
        if (pick < 0) {
            break;
        }
        used[static_cast<std::size_t>(pick)] = 1;
        chain.push_back(clusters[static_cast<std::size_t>(pick)]);
    }
    return chain;
}

}  // namespace

SuggestResult suggestLinkGroups(const Scene& scene) {
    SuggestResult result;
    result.note = "分组按半径不小于 40 mm、且接近坐标轴方向的圆柱。平行且相距小于 220 mm 的弱轴并进更强的那根。A1 是最强竖直轴，其余从底座往外逐个接最近的锚点。包围盒相邻不能切开这台机器人。";
    const std::vector<PartInfo> parts = collectParts(scene);
    std::vector<Cluster> chain = orderChain(selectJoints(collectAxes(scene, parts)));
    if (chain.empty()) {
        result.note = "没有足够大的圆柱面，无法建议关节。";
        return result;
    }
    result.links.push_back(LinkSuggestion{});
    result.links.back().name = "base";
    for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
        LinkSuggestion link;
        link.name = "A" + std::to_string(i + 1);
        link.hasJoint = true;
        link.jointAxis = chain[static_cast<std::size_t>(i)].direction;
        link.jointOrigin = chain[static_cast<std::size_t>(i)].anchor;
        link.axisCount = chain[static_cast<std::size_t>(i)].count;
        link.axisRadius = chain[static_cast<std::size_t>(i)].radius;
        result.links.push_back(std::move(link));
    }
    std::unordered_map<std::string, int> owner;
    for (const PartInfo& part : parts) {
        if (!part.large) {
            continue;
        }
        int best = -1;
        double bestDistance = 1e300;
        for (int i = 0; i < static_cast<int>(chain.size()); ++i) {
            const Cluster& cluster = chain[static_cast<std::size_t>(i)];
            const Vec3 onLine = closestOnLine(part.center, cluster.anchor, cluster.direction);
            const double radial = lineDistance(part.center, cluster.anchor, cluster.direction);
            // A second machine in the same file sits metres off the chosen axes. Leave it put.
            if (radial > std::max(600.0, cluster.radius * 5.0)) {
                continue;
            }
            const double distance = radial + 0.35 * length(onLine - cluster.anchor);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = i;
            }
        }
        if (best < 0) {
            continue;
        }
        int linkIndex = best + 1;
        if (best == 0 && part.center.z < chain[0].anchor.z) {
            linkIndex = 0;
        }
        result.links[static_cast<std::size_t>(linkIndex)].partIds.push_back(part.id);
        owner[part.id] = linkIndex;
    }
    for (const PartInfo& part : parts) {
        if (part.large) {
            continue;
        }
        int linkIndex = -1;
        double best = 1e300;
        for (const PartInfo& other : parts) {
            if (!other.large || !owner.count(other.id)) {
                continue;
            }
            const double distance = length(part.center - other.center);
            if (distance < best) {
                best = distance;
                linkIndex = owner[other.id];
            }
        }
        if (linkIndex < 0 || best > 800.0) {
            continue;
        }
        result.links[static_cast<std::size_t>(linkIndex)].partIds.push_back(part.id);
    }
    if (chain.size() >= 2) {
        result.a1ToA2Mm = lineDistance(chain[1].anchor, chain[0].anchor, chain[0].direction);
        result.fitted.shoulder = result.a1ToA2Mm;
        result.fitted.d1 = std::abs(chain[1].anchor.z - chain[0].anchor.z);
        if (result.fitted.d1 < 1.0) {
            result.fitted.d1 = length(chain[1].anchor - chain[0].anchor);
        }
    }
    if (chain.size() >= 3) {
        result.fitted.a2 = length(chain[2].anchor - chain[1].anchor);
    }
    if (chain.size() >= 5) {
        result.fitted.a3 = length(chain[4].anchor - chain[2].anchor);
    } else if (chain.size() >= 4) {
        result.fitted.a3 = length(chain[3].anchor - chain[2].anchor);
    }
    if (chain.size() >= 2) {
        result.fitted.d6 = length(chain.back().anchor - chain[chain.size() - 2].anchor);
    }
    if (chain.size() != 6) {
        result.note += " 建议轴数是 " + std::to_string(chain.size()) + "，不是 6。";
    }
    return result;
}

bool axisFromCylindricalFace(const Scene& scene, const std::string& nodeId, Vec3 point, Vec3 normal, Vec3& origin,
                             Vec3& direction, double& radius) {
    auto node = scene.find(nodeId);
    if (!node || node->faceAxes.empty()) {
        return false;
    }
    const Mat4 world = node->worldMatrix();
    const Vec3 n = length(normal) > 1e-8 ? normalize(normal) : Vec3{0, 0, 1};
    double best = 1e300;
    bool found = false;
    for (const FaceAxis& axis : node->faceAxes) {
        const Vec3 worldOrigin = transformPoint(world, axis.origin);
        Vec3 worldDirection = normalize(transformVector(world, axis.direction));
        if (length(worldDirection) < 0.5 || axis.radius <= 0) {
            continue;
        }
        const double radial = lineDistance(point, worldOrigin, worldDirection);
        const double error = std::abs(radial - axis.radius);
        if (error > std::max(3.0, 0.12 * axis.radius)) {
            continue;
        }
        if (std::abs(dot(n, worldDirection)) > 0.45) {
            continue;
        }
        if (error < best) {
            best = error;
            origin = closestOnLine(point, worldOrigin, worldDirection);
            direction = worldDirection;
            radius = axis.radius;
            found = true;
        }
    }
    return found;
}

bool applyLinkSuggestions(Scene& scene, const std::string& resourceName, const SuggestResult& suggestion,
                          const std::vector<RevoluteLimit>& limits, std::string& error) {
    if (suggestion.links.size() < 2) {
        error = suggestion.note.empty() ? "no link suggestion" : suggestion.note;
        return false;
    }
    KinematicResource& resource = ensureResource(scene, resourceName);
    resource.links.clear();
    resource.joints.clear();
    resource.frames.clear();
    resource.ik = "numerical";
    resource.wrist = suggestion.fitted;
    const Vec3 palette[] = {{0.45, 0.48, 0.52}, {0.72, 0.55, 0.28}, {0.35, 0.62, 0.78}, {0.42, 0.66, 0.42},
                            {0.62, 0.46, 0.72}, {0.78, 0.58, 0.32}, {0.70, 0.70, 0.42}};
    std::string parent;
    int jointIndex = 0;
    for (std::size_t i = 0; i < suggestion.links.size(); ++i) {
        const LinkSuggestion& link = suggestion.links[i];
        addLink(scene, resource, link.name, parent, {});
        std::string adoptError;
        if (!link.partIds.empty() && !adoptParts(scene, resource, link.name, link.partIds, adoptError)) {
            error = adoptError;
            return false;
        }
        const Vec3 tint = palette[i % 7];
        for (const std::string& id : link.partIds) {
            if (auto node = scene.find(id)) {
                node->color = tint;
            }
        }
        if (link.hasJoint && !parent.empty()) {
            KinJoint& joint = addJoint(resource, scene, link.name, parent, link.name, JointMotion::Revolute, link.jointAxis,
                                       link.jointOrigin);
            if (jointIndex < static_cast<int>(limits.size())) {
                const RevoluteLimit& limit = limits[static_cast<std::size_t>(jointIndex)];
                joint.lower = limit.lower;
                joint.upper = limit.upper;
                joint.maxSpeed = limit.speed;
            }
            joint.maxAccel = 180;
            joint.position = 0;
            ++jointIndex;
        }
        parent = link.name;
    }
    NamedFrame tcp;
    tcp.name = "tcp";
    if (!suggestion.links.empty() && suggestion.links.back().hasJoint) {
        tcp.origin = suggestion.links.back().jointOrigin;
    }
    resource.frames.push_back(tcp);
    error.clear();
    return true;
}

std::vector<std::shared_ptr<SceneNode>> hidePartsOutsideProximity(Scene& scene) {
    std::vector<std::shared_ptr<SceneNode>> hidden;
    if (!scene.root() || scene.proximity.sets.empty()) {
        return hidden;
    }
    std::unordered_set<std::string> keep;
    for (const CollisionSet& set : scene.proximity.sets) {
        for (const std::string& name : set.objects) {
            keep.insert(name);
        }
    }
    std::vector<std::shared_ptr<SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->kind == NodeKind::Part && !node->meshId.empty() && node->visible && keep.count(node->name) == 0) {
            node->visible = false;
            hidden.push_back(node);
        }
    }
    return hidden;
}

std::vector<RevoluteLimit> kr420DatasheetLimits() {
    return {{-185, 185, 90}, {-130, 20, 80}, {-100, 144, 75}, {-350, 350, 90}, {-120, 120, 83}, {-350, 350, 130}};
}

void addKr420Demonstration(Scene& scene) {
    KinematicResource* resource = findResource(scene.resources, "kr420");
    if (resource == nullptr || resource->links.empty()) {
        return;
    }
    const KinLink& tip = resource->links.back();
    auto tipNode = scene.find(tip.nodeId);
    Aabb bounds;
    if (tipNode) {
        std::vector<std::shared_ptr<SceneNode>> stack{tipNode};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->meshId.empty()) {
                continue;
            }
            if (const auto mesh = scene.mesh(node->meshId); mesh && mesh->bounds.valid) {
                const Mat4 world = node->worldMatrix();
                const Vec3 corners[2] = {mesh->bounds.min, mesh->bounds.max};
                for (int i = 0; i < 8; ++i) {
                    const Vec3 local{(i & 1) ? corners[1].x : corners[0].x, (i & 2) ? corners[1].y : corners[0].y,
                                     (i & 4) ? corners[1].z : corners[0].z};
                    bounds.expand(transformPoint(world, local));
                }
            }
        }
    }
    auto obstacle = scene.makeNode(NodeKind::Part, "Fixture");
    const Vec3 center = bounds.valid ? Vec3{bounds.center().x, bounds.max.y + 12.0 + 30.0, bounds.center().z}
                                    : Vec3{800, 400, 1500};
    obstacle->local.translation = center;
    obstacle->meshId = scene.addMesh(makeBoxMesh({30, 30, 80}));
    obstacle->color = {0.75, 0.75, 0.78};
    scene.addChild(scene.root(), obstacle);
    scene.proximity.clearanceMm = 20;
    CollisionSet arm;
    arm.name = "wrist";
    if (tipNode) {
        std::vector<std::shared_ptr<SceneNode>> stack{tipNode};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            if (node->kind == NodeKind::Part && !node->meshId.empty()) {
                arm.objects.push_back(node->name);
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
        }
    }
    CollisionSet fixture;
    fixture.name = "fixture";
    fixture.objects = {"Fixture"};
    scene.proximity.sets = {arm, fixture};
    scene.proximity.pairs = {{"wrist", "fixture"}};
    const auto hidden = hidePartsOutsideProximity(scene);
    {
        auto query = createCollisionQuery();
        query->rebuild(scene);
        ContactHit closest;
        bool found = false;
        for (const ContactHit& hit : query->query(scene, 800)) {
            const bool touches = hit.a == "Fixture" || hit.b == "Fixture";
            if (!touches) {
                continue;
            }
            if (!found || hit.distance < closest.distance) {
                closest = hit;
                found = true;
            }
        }
        if (found) {
            const Vec3 onFixture = closest.a == "Fixture" ? closest.pointA : closest.pointB;
            const Vec3 onRobot = closest.a == "Fixture" ? closest.pointB : closest.pointA;
            const Vec3 toward = onRobot - onFixture;
            const double span = length(toward);
            if (span > 1e-3) {
                obstacle->local.translation = obstacle->local.translation + toward / span * (closest.distance - 12.0);
            }
        }
        query->commitCache(scene);
    }
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }

    RobotPath path;
    path.name = "cycle";
    path.resource = resource->name;
    const std::size_t n = resource->joints.size();
    PathPoint home;
    home.name = "home";
    home.move = PathMove::Ptp;
    home.joints.assign(n, 0);
    home.speed = 50;
    PathPoint away;
    away.name = "away";
    away.move = PathMove::Ptp;
    away.joints.assign(n, 0);
    const double nudge[] = {18, -12, 15, 25, -10, 20};
    for (std::size_t i = 0; i < n && i < 6; ++i) {
        away.joints[i] = nudge[i];
    }
    away.speed = 40;
    away.zone = 30;
    home.zone = 20;
    PathPoint back;
    back.name = "return";
    back.move = PathMove::Lin;
    back.cartesian = true;
    const CartesianPose tcp = forwardModel(*resource);
    back.position = tcp.position;
    back.orientation = tcp.rotation;
    back.speed = 200;
    path.points = {home, away, back};
    scene.robotPaths.push_back(std::move(path));
}

}  // namespace sim
