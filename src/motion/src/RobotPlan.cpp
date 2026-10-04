#include "sim/motion/Motion.hpp"

#include "sim/collision/Query.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <unordered_map>

namespace sim {

Quat slerpPublic(Quat a, Quat b, double t);

namespace {

struct Chunk {
    std::vector<std::vector<double>> frames;
};

bool adjacentLinks(const KinematicResource& resource, const std::string& a, const std::string& b) {
    for (const KinJoint& joint : resource.joints) {
        if ((joint.parentLink == a && joint.childLink == b) || (joint.parentLink == b && joint.childLink == a)) {
            return true;
        }
    }
    return false;
}

void pushPtp(KinematicResource& resource, const std::vector<double>& target, double dt, Chunk& chunk) {
    ActiveMove move;
    std::string error;
    beginJointMove(resource, target, ProfileKind::Trapezoid, move, error);
    chunk.frames.push_back(jointVector(resource));
    for (int guard = 0; guard < 20000 && move.active; ++guard) {
        advanceMove(resource, move, dt);
        chunk.frames.push_back(jointVector(resource));
    }
}

CartesianPose poseNow(const KinematicResource& resource) { return forwardModel(resource); }

std::vector<double> ikOrHold(KinematicResource& resource, const CartesianPose& pose, std::vector<std::string>& warnings) {
    const std::vector<double> seed = jointVector(resource);
    const IkResult solved = inverseModel(resource, pose, seed);
    if (!solved.ok) {
        warnings.push_back(solved.warning.empty() ? "unreachable" : solved.warning);
        return seed;
    }
    if (solved.approximated && !solved.warning.empty()) {
        warnings.push_back(solved.warning);
    }
    std::vector<double> next = seed;
    for (std::size_t i = 0; i < solved.joints.size() && i < next.size(); ++i) {
        next[i] = solved.joints[i];
    }
    if (next.size() < solved.joints.size()) {
        next = solved.joints;
    }
    return next;
}

void pushCartesian(KinematicResource& resource, const CartesianPose& goal, double speed, double dt, Chunk& chunk,
                   std::vector<std::string>& warnings) {
    const CartesianPose start = poseNow(resource);
    const double distance = std::max(length(goal.position - start.position), 1e-6);
    const double accel = std::max(speed * 4.0, 1.0);
    const double duration = profileDuration(ProfileKind::Trapezoid, 0, distance, std::max(speed, 1e-3), accel);
    chunk.frames.push_back(jointVector(resource));
    if (duration <= 1e-6) {
        return;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 1; i <= steps; ++i) {
        bool done = false;
        const double along = profileSample(ProfileKind::Trapezoid, 0, distance, std::max(speed, 1e-3), accel, i * dt, &done);
        const double u = std::clamp(along / distance, 0.0, 1.0);
        CartesianPose pose;
        pose.position = start.position + (goal.position - start.position) * u;
        pose.rotation = slerpPublic(start.rotation, goal.rotation, u);
        const std::vector<double> next = ikOrHold(resource, pose, warnings);
        setJointVector(resource, next);
        chunk.frames.push_back(jointVector(resource));
        if (done) {
            break;
        }
    }
}

Vec3 rotateAbout(Vec3 point, Vec3 center, Vec3 axis, double radians) {
    return center + Quat::fromAxisAngle(axis, radians).rotate(point - center);
}

bool circleThrough(Vec3 a, Vec3 b, Vec3 c, Vec3& center, Vec3& axis, double& sweep) {
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 normal = cross(ab, ac);
    const double denom = 2.0 * dot(normal, normal);
    if (denom < 1e-6) {
        return false;
    }
    center = a + (cross(normal, ab) * dot(ac, ac) + cross(ac, normal) * dot(ab, ab)) / denom;
    axis = normalize(normal);
    const Vec3 ua = a - center;
    const Vec3 ub = b - center;
    const Vec3 uc = c - center;
    const double abAngle = std::atan2(dot(axis, cross(ua, ub)), dot(ua, ub));
    const double bcAngle = std::atan2(dot(axis, cross(ub, uc)), dot(ub, uc));
    sweep = abAngle + bcAngle;
    return std::abs(sweep) > 1e-6;
}

std::string externalModeOf(const RobotPath& path, const PathPoint& point) {
    if (!point.external.empty()) {
        return point.external;
    }
    if (!path.external.empty()) {
        return path.external;
    }
    return "locked";
}

void writeExternal(KinematicResource& resource, const std::vector<int>& index, const std::vector<double>& values) {
    for (std::size_t k = 0; k < index.size() && k < values.size(); ++k) {
        KinJoint& joint = resource.joints[static_cast<std::size_t>(index[k])];
        if (!joint.coupledTo.empty()) {
            continue;
        }
        joint.position = values[k];
    }
}

void pushCoupled(KinematicResource& resource, const RobotPath& path, const PathPoint& point, const CartesianPose& goal, double dt,
                 Chunk& chunk, std::vector<std::string>& warnings) {
    const std::string mode = externalModeOf(path, point);
    const CartesianPose start = poseNow(resource);
    const std::vector<int> external = externalJointIndices(resource);
    std::vector<double> ext0(external.size());
    std::vector<double> ext1(external.size());
    for (std::size_t k = 0; k < external.size(); ++k) {
        const KinJoint& joint = resource.joints[static_cast<std::size_t>(external[k])];
        ext0[k] = jointCommand(resource, joint);
        ext1[k] = ext0[k];
    }
    bool circular = false;
    Vec3 center;
    Vec3 circleAxis{0, 0, 1};
    double sweep = 0;
    if (mode == "sync" && point.move == PathMove::Circ) {
        if (circleThrough(start.position, point.via, goal.position, center, circleAxis, sweep)) {
            circular = true;
        } else {
            warnings.push_back(point.name.empty() ? "circ fell back to lin" : point.name + " fell back to lin");
        }
    }
    if (mode == "sync") {
        for (std::size_t k = 0; k < external.size(); ++k) {
            const KinJoint& joint = resource.joints[static_cast<std::size_t>(external[k])];
            if (joint.externalKind != "track" || joint.motion != JointMotion::Prismatic) {
                continue;
            }
            Vec3 axis = normalize(joint.axis);
            if (!joint.parentLink.empty()) {
                Mat4 parent = Mat4::identity();
                if (linkWorld(resource, joint.parentLink, parent)) {
                    axis = normalize(transformVector(parent, joint.axis));
                }
            }
            if (length(axis) < 1e-12) {
                axis = {1, 0, 0};
            }
            const double along = dot(goal.position - (start.position - axis * ext0[k]), axis);
            ext1[k] = clampJoint(joint, along);
        }
    } else if (mode == "positioner") {
        for (std::size_t k = 0; k < external.size(); ++k) {
            const KinJoint& joint = resource.joints[static_cast<std::size_t>(external[k])];
            if (joint.externalKind != "positioner" || k >= point.externalValues.size()) {
                continue;
            }
            ext1[k] = clampJoint(joint, point.externalValues[k]);
        }
    }

    bool workpiece = false;
    CartesianPose startLocal = start;
    CartesianPose goalLocal;
    goalLocal.position = point.position;
    goalLocal.rotation = point.orientation;
    if (mode == "positioner" && !path.workpiece.empty() && namedFrameWorld(resource, path.workpiece, startLocal)) {
        workpiece = true;
        const CartesianPose frameNow = startLocal;
        startLocal = poseFromMatrix(mul(inverseAffine(matrixFromPose(frameNow)), matrixFromPose(start)));
    }

    double distance = std::max(length(goal.position - start.position), 1e-6);
    if (circular) {
        distance = std::max(std::abs(length(start.position - center) * sweep), 1e-6);
    } else if (workpiece) {
        std::vector<double> saved = ext0;
        writeExternal(resource, external, ext1);
        CartesianPose frameEnd;
        if (namedFrameWorld(resource, path.workpiece, frameEnd)) {
            const CartesianPose endPose = poseFromMatrix(mul(matrixFromPose(frameEnd), matrixFromPose(goalLocal)));
            distance = std::max(length(endPose.position - start.position), 1e-6);
        }
        writeExternal(resource, external, saved);
    }

    const double speed = std::max(point.speed, 1e-3);
    const double accel = std::max(speed * 4.0, 1.0);
    double duration = profileDuration(ProfileKind::Trapezoid, 0, distance, speed, accel);
    double driver0 = 0;
    double driver1 = distance;
    double driverV = speed;
    double driverA = accel;
    for (std::size_t k = 0; k < external.size(); ++k) {
        if (std::abs(ext1[k] - ext0[k]) < 1e-9) {
            continue;
        }
        const KinJoint& joint = resource.joints[static_cast<std::size_t>(external[k])];
        const double jointDuration =
            profileDuration(ProfileKind::Trapezoid, ext0[k], ext1[k], std::max(joint.maxSpeed, 1e-6), std::max(joint.maxAccel, 1e-6));
        if (jointDuration > duration) {
            duration = jointDuration;
            driver0 = ext0[k];
            driver1 = ext1[k];
            driverV = std::max(joint.maxSpeed, 1e-6);
            driverA = std::max(joint.maxAccel, 1e-6);
        }
    }

    const RedundancyPreference saved = resource.redundancy;
    chunk.frames.push_back(jointVector(resource));
    if (duration <= 1e-6) {
        resource.redundancy = saved;
        return;
    }
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int step = 1; step <= steps; ++step) {
        bool done = false;
        const double along = profileSample(ProfileKind::Trapezoid, driver0, driver1, driverV, driverA, step * dt, &done);
        const double span = driver1 - driver0;
        const double u = std::abs(span) < 1e-12 ? 1.0 : std::clamp((along - driver0) / span, 0.0, 1.0);
        std::vector<double> ext(external.size());
        for (std::size_t k = 0; k < external.size(); ++k) {
            ext[k] = ext0[k] + (ext1[k] - ext0[k]) * u;
        }
        writeExternal(resource, external, ext);
        CartesianPose pose;
        if (workpiece) {
            CartesianPose frame;
            if (namedFrameWorld(resource, path.workpiece, frame)) {
                CartesianPose local;
                local.position = startLocal.position + (goalLocal.position - startLocal.position) * u;
                local.rotation = slerpPublic(startLocal.rotation, goalLocal.rotation, u);
                pose = poseFromMatrix(mul(matrixFromPose(frame), matrixFromPose(local)));
            } else {
                pose.position = start.position + (goal.position - start.position) * u;
                pose.rotation = slerpPublic(start.rotation, goal.rotation, u);
            }
        } else if (circular) {
            pose.position = rotateAbout(start.position, center, circleAxis, sweep * u);
            pose.rotation = slerpPublic(start.rotation, goal.rotation, u);
        } else {
            pose.position = start.position + (goal.position - start.position) * u;
            pose.rotation = slerpPublic(start.rotation, goal.rotation, u);
        }
        resource.redundancy.mode = "fixed";
        resource.redundancy.fixed = ext;
        const std::vector<double> next = ikOrHold(resource, pose, warnings);
        resource.redundancy = saved;
        setJointVector(resource, next);
        chunk.frames.push_back(jointVector(resource));
        if (done) {
            break;
        }
    }
    resource.redundancy = saved;
}

std::string ownerLink(const Scene& scene, const KinematicResource& resource, const std::string& partName) {
    auto node = scene.findByName(partName);
    while (node) {
        for (const KinLink& link : resource.links) {
            if (link.nodeId == node->id) {
                return link.name;
            }
        }
        node = node->parent.lock();
    }
    return {};
}

bool relevantPair(const Scene& scene, const KinematicResource& resource, const std::string& a, const std::string& b) {
    if (!scene.proximity.pairs.empty()) {
        return true;
    }
    const std::string left = ownerLink(scene, resource, a);
    const std::string right = ownerLink(scene, resource, b);
    if (left.empty() && right.empty()) {
        return false;
    }
    if (left == right) {
        return false;
    }
    if (!left.empty() && !right.empty() && adjacentLinks(resource, left, right)) {
        return false;
    }
    return true;
}

void meshCorners(const Scene& scene, const SceneNode& node, std::vector<Vec3>& corners) {
    if (!node.meshId.empty()) {
        if (const auto mesh = scene.mesh(node.meshId); mesh && mesh->bounds.valid) {
            const Mat4 world = node.worldMatrix();
            const Vec3 lo = mesh->bounds.min;
            const Vec3 hi = mesh->bounds.max;
            const Vec3 local[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                                   {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
            for (const Vec3& point : local) {
                corners.push_back(transformPoint(world, point));
            }
        }
    }
    for (const auto& child : node.children) {
        if (child) {
            meshCorners(scene, *child, corners);
        }
    }
}

double pointAxisDistance(Vec3 point, Vec3 origin, Vec3 axis) {
    const Vec3 delta = point - origin;
    return length(delta - axis * dot(delta, axis));
}

double jointReach(const Scene& scene, const KinematicResource& resource, const KinJoint& joint) {
    const KinLink* child = nullptr;
    const KinLink* parent = nullptr;
    for (const KinLink& link : resource.links) {
        if (link.name == joint.childLink) {
            child = &link;
        }
        if (link.name == joint.parentLink) {
            parent = &link;
        }
    }
    if (child == nullptr) {
        return 0;
    }
    auto childNode = scene.find(child->nodeId);
    if (!childNode) {
        return 0;
    }
    Mat4 parentWorld = Mat4::identity();
    if (parent != nullptr) {
        if (auto parentNode = scene.find(parent->nodeId)) {
            parentWorld = parentNode->worldMatrix();
        }
    } else if (auto parentNode = childNode->parent.lock()) {
        parentWorld = parentNode->worldMatrix();
    }
    const Vec3 origin = transformPoint(parentWorld, joint.anchor);
    const Vec3 axis = normalize(transformVector(parentWorld, joint.axis));
    if (length(axis) < 1e-12) {
        return 0;
    }
    std::vector<Vec3> corners;
    meshCorners(scene, *childNode, corners);
    double reach = 0;
    for (const Vec3& corner : corners) {
        reach = std::max(reach, pointAxisDistance(corner, origin, axis));
    }
    return reach;
}

double motionBound(const Scene& scene, const KinematicResource& resource, const std::vector<double>& q0,
                   const std::vector<double>& q1) {
    std::unordered_map<std::string, double> memo;
    std::function<double(const std::string&)> slack = [&](const std::string& link) {
        const auto existing = memo.find(link);
        if (existing != memo.end()) {
            return existing->second;
        }
        memo[link] = 0;
        double bound = 0;
        for (std::size_t i = 0; i < resource.joints.size(); ++i) {
            const KinJoint& joint = resource.joints[i];
            if (joint.parentLink != link) {
                continue;
            }
            const double before = i < q0.size() ? q0[i] : 0;
            const double after = i < q1.size() ? q1[i] : 0;
            const double delta = std::abs(after - before);
            const double child = slack(joint.childLink);
            if (joint.motion == JointMotion::Prismatic) {
                bound = std::max(bound, delta + child);
            } else {
                const double theta = deg2rad(delta);
                const double reach = jointReach(scene, resource, joint);
                bound = std::max(bound, theta * (reach + child) + child);
            }
        }
        memo[link] = bound;
        return bound;
    };
    double bound = 0;
    for (const KinLink& link : resource.links) {
        bool root = true;
        for (const KinJoint& joint : resource.joints) {
            if (joint.childLink == link.name) {
                root = false;
            }
        }
        if (root) {
            bound = std::max(bound, slack(link.name));
        }
    }
    return bound;
}

std::vector<double> mixSamples(const std::vector<double>& a, const std::vector<double>& b, double u) {
    std::vector<double> out(std::max(a.size(), b.size()));
    for (std::size_t i = 0; i < out.size(); ++i) {
        const double left = i < a.size() ? a[i] : 0;
        const double right = i < b.size() ? b[i] : 0;
        out[i] = (1.0 - u) * left + u * right;
    }
    return out;
}

}  // namespace

// slerp is file-local in Solve.cpp. Duplicate a small one here.
Quat slerpPublic(Quat a, Quat b, double t) {
    auto dotQ = [](Quat p, Quat q) { return p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z; };
    a = a.normalized();
    b = b.normalized();
    double d = dotQ(a, b);
    if (d < 0) {
        b = Quat{-b.w, -b.x, -b.y, -b.z};
        d = -d;
    }
    if (d > 0.9995) {
        return Quat{a.w + t * (b.w - a.w), a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)}.normalized();
    }
    const double theta = std::acos(std::clamp(d, -1.0, 1.0));
    const double s = std::sin(theta);
    const double w0 = std::sin((1.0 - t) * theta) / s;
    const double w1 = std::sin(t * theta) / s;
    return Quat{w0 * a.w + w1 * b.w, w0 * a.x + w1 * b.x, w0 * a.y + w1 * b.y, w0 * a.z + w1 * b.z}.normalized();
}

std::vector<std::pair<std::string, std::string>> collidingPairs(const Scene& scene, const KinematicResource& resource) {
    auto query = createCollisionQuery();
    query->rebuild(scene);
    std::set<std::pair<std::string, std::string>> hits;
    for (const ContactHit& hit : query->query(scene, 0)) {
        if (hit.distance > 1e-4 || !relevantPair(scene, resource, hit.a, hit.b)) {
            continue;
        }
        if (hit.a < hit.b) {
            hits.emplace(hit.a, hit.b);
        } else {
            hits.emplace(hit.b, hit.a);
        }
    }
    return {hits.begin(), hits.end()};
}

PathSweepHit sweepPlannedSamples(Scene& scene, KinematicResource& resource, const std::vector<std::vector<double>>& samples) {
    PathSweepHit sweep;
    if (samples.empty()) {
        return sweep;
    }
    setJointVector(resource, samples.front());
    applyResource(scene, resource);
    auto query = createCollisionQuery();
    query->rebuild(scene);
    if (samples.size() == 1) {
        for (const ContactHit& hit : query->query(scene, 0)) {
            if (hit.distance <= 1e-4 && relevantPair(scene, resource, hit.a, hit.b)) {
                sweep.hit = true;
                sweep.a = hit.a;
                sweep.b = hit.b;
                sweep.point = hit.pointA;
                return sweep;
            }
        }
        return sweep;
    }
    for (std::size_t segment = 0; segment + 1 < samples.size(); ++segment) {
        double u = 0;
        for (int iter = 0; iter < 24; ++iter) {
            const std::vector<double> pose = mixSamples(samples[segment], samples[segment + 1], u);
            setJointVector(resource, pose);
            applyResource(scene, resource);
            const double bound = std::max(motionBound(scene, resource, pose, samples[segment + 1]), 0.0);
            const double gapLimit = std::max(bound, 0.05);
            double distance = gapLimit + 1;
            ContactHit chosen;
            bool found = false;
            for (const ContactHit& hit : query->query(scene, gapLimit)) {
                if (!relevantPair(scene, resource, hit.a, hit.b)) {
                    continue;
                }
                if (!found || hit.distance < distance) {
                    found = true;
                    distance = hit.distance;
                    chosen = hit;
                }
            }
            const double time = (static_cast<double>(segment) + u) * (1.0 / 60.0);
            if (found && distance <= 0.05) {
                sweep.hit = true;
                sweep.time = time;
                sweep.a = chosen.a;
                sweep.b = chosen.b;
                sweep.point = chosen.pointA;
                return sweep;
            }
            if (!found || distance > bound) {
                break;
            }
            const double remain = 1.0 - u;
            const double alpha = std::clamp(0.9 * distance / std::max(bound, 1e-9), 0.05, 1.0);
            if (alpha >= 0.999) {
                break;
            }
            u += alpha * remain;
            if (u >= 1.0 - 1e-4) {
                break;
            }
        }
    }
    return sweep;
}

void highlightCollisions(Scene& scene, const KinematicResource& resource,
                         const std::vector<std::pair<std::string, std::string>>& pairs) {
    const Vec3 alert{0.86, 0.28, 0.22};
    for (const auto& [a, b] : pairs) {
        for (const std::string& name : {a, b}) {
            const KinLink* link = nullptr;
            for (const KinLink& candidate : resource.links) {
                if (candidate.name == name) {
                    link = &candidate;
                }
            }
            if (link != nullptr) {
                if (auto node = scene.find(link->nodeId)) {
                    node->color = alert;
                }
            }
            if (auto node = scene.findByName(name)) {
                node->color = alert;
            }
        }
    }
}

MotionPlan planRobotPath(Scene& scene, KinematicResource& resource, const RobotPath& path) {
    MotionPlan plan;
    if (path.points.empty()) {
        plan.warnings.push_back("path is empty");
        return plan;
    }
    std::vector<double> savedPosition;
    savedPosition.reserve(resource.joints.size());
    for (const KinJoint& joint : resource.joints) {
        savedPosition.push_back(joint.position);
    }
    std::vector<Chunk> chunks(path.points.size());
    for (std::size_t i = 0; i < path.points.size(); ++i) {
        const PathPoint& point = path.points[i];
        if (point.move == PathMove::Ptp && !point.cartesian) {
            std::vector<double> target = jointVector(resource);
            for (std::size_t j = 0; j < point.joints.size() && j < target.size(); ++j) {
                target[j] = point.joints[j];
            }
            for (std::size_t j = 0; j < target.size() && j < resource.joints.size(); ++j) {
                const double clamped = clampJoint(resource.joints[j], target[j]);
                if (std::abs(clamped - target[j]) > 1e-4) {
                    plan.warnings.push_back(resource.joints[j].name + " limit");
                }
                target[j] = clamped;
            }
            pushPtp(resource, target, plan.dt, chunks[i]);
            continue;
        }
        CartesianPose goal;
        goal.position = point.position;
        goal.rotation = point.orientation;
        if (!point.cartesian && !point.joints.empty()) {
            const std::vector<double> current = jointVector(resource);
            setJointVector(resource, point.joints);
            goal = forwardModel(resource);
            setJointVector(resource, current);
        }
        const std::string mode = externalModeOf(path, point);
        if (mode == "sync" || mode == "positioner") {
            pushCoupled(resource, path, point, goal, plan.dt, chunks[i], plan.warnings);
            continue;
        }
        const RedundancyPreference savedRedundancy = resource.redundancy;
        if (hasExternalAxis(resource)) {
            resource.redundancy.mode = "fixed";
            resource.redundancy.fixed.clear();
            for (const KinJoint& joint : resource.joints) {
                if (!joint.externalName.empty()) {
                    resource.redundancy.fixed.push_back(jointCommand(resource, joint));
                }
            }
        }
        if (point.move == PathMove::Circ) {
            const CartesianPose start = forwardModel(resource);
            Vec3 center;
            Vec3 axis;
            double sweep = 0;
            if (!circleThrough(start.position, point.via, goal.position, center, axis, sweep)) {
                plan.warnings.push_back(point.name.empty() ? "circ fell back to lin" : point.name + " fell back to lin");
                pushCartesian(resource, goal, point.speed, plan.dt, chunks[i], plan.warnings);
            } else {
                const double radius = length(start.position - center);
                const double distance = std::abs(radius * sweep);
                const double speed = std::max(point.speed, 1e-3);
                const double duration = profileDuration(ProfileKind::Trapezoid, 0, std::max(distance, 1e-6), speed, speed * 4.0);
                chunks[i].frames.push_back(jointVector(resource));
                const int steps = std::max(1, static_cast<int>(std::ceil(duration / plan.dt)));
                for (int step = 1; step <= steps; ++step) {
                    bool done = false;
                    const double along =
                        profileSample(ProfileKind::Trapezoid, 0, std::max(distance, 1e-6), speed, speed * 4.0, step * plan.dt, &done);
                    const double u = distance <= 1e-6 ? 1.0 : std::clamp(along / distance, 0.0, 1.0);
                    CartesianPose pose;
                    pose.position = rotateAbout(start.position, center, axis, sweep * u);
                    pose.rotation = slerpPublic(start.rotation, goal.rotation, u);
                    setJointVector(resource, ikOrHold(resource, pose, plan.warnings));
                    chunks[i].frames.push_back(jointVector(resource));
                    if (done) {
                        break;
                    }
                }
            }
            resource.redundancy = savedRedundancy;
            continue;
        }
        pushCartesian(resource, goal, point.speed, plan.dt, chunks[i], plan.warnings);
        resource.redundancy = savedRedundancy;
    }

    plan.segmentSeconds.reserve(chunks.size());
    for (const Chunk& chunk : chunks) {
        const double span = chunk.frames.size() <= 1 ? 0.0 : (static_cast<double>(chunk.frames.size()) - 1) * plan.dt;
        plan.segmentSeconds.push_back(span);
    }

    for (std::size_t i = 0; i < chunks.size(); ++i) {
        int begin = plan.samples.empty() ? 0 : 1;
        int end = static_cast<int>(chunks[i].frames.size());
        int drop = 0;
        if (i + 1 < chunks.size() && path.points[i].zone > 0.0) {
            const double blendTime = std::min(0.3 * static_cast<double>(chunks[i].frames.size()) * plan.dt,
                                              path.points[i].zone / std::max(path.points[i].speed, 1.0));
            drop = static_cast<int>(std::round(blendTime / plan.dt));
            drop = std::min(drop, std::max(0, end - begin - 1));
            drop = std::min(drop, std::max(0, static_cast<int>(chunks[i + 1].frames.size()) - 1));
        }
        if (drop > 0) {
            end -= drop;
        }
        for (int frame = begin; frame < end; ++frame) {
            plan.samples.push_back(chunks[i].frames[static_cast<std::size_t>(frame)]);
        }
        for (int k = 0; k < drop; ++k) {
            const auto& incoming = chunks[i].frames[chunks[i].frames.size() - static_cast<std::size_t>(drop) + static_cast<std::size_t>(k)];
            const auto& outgoing = chunks[i + 1].frames[static_cast<std::size_t>(k)];
            const double u = static_cast<double>(k + 1) / static_cast<double>(drop + 1);
            std::vector<double> mixed(std::max(incoming.size(), outgoing.size()));
            for (std::size_t n = 0; n < mixed.size(); ++n) {
                const double a = n < incoming.size() ? incoming[n] : 0;
                const double b = n < outgoing.size() ? outgoing[n] : 0;
                mixed[n] = (1.0 - u) * a + u * b;
            }
            plan.samples.push_back(std::move(mixed));
        }
        if (drop > 0 && i + 1 < chunks.size()) {
            chunks[i + 1].frames.erase(chunks[i + 1].frames.begin(),
                                       chunks[i + 1].frames.begin() + std::min(drop, static_cast<int>(chunks[i + 1].frames.size())));
        }
    }

    {
        auto warm = createCollisionQuery();
        warm->rebuild(scene);
        warm->commitCache(scene);
    }
    std::set<std::pair<std::string, std::string>> collisions;
    for (const std::vector<double>& sample : plan.samples) {
        setJointVector(resource, sample);
        applyResource(scene, resource);
        for (const auto& hit : collidingPairs(scene, resource)) {
            collisions.insert(hit);
        }
    }
    const PathSweepHit swept = sweepPlannedSamples(scene, resource, plan.samples);
    plan.sweepHit = swept.hit;
    plan.sweepTime = swept.time;
    plan.sweepA = swept.a;
    plan.sweepB = swept.b;
    if (swept.hit) {
        if (swept.a < swept.b) {
            collisions.emplace(swept.a, swept.b);
        } else {
            collisions.emplace(swept.b, swept.a);
        }
    }
    plan.collisions.assign(collisions.begin(), collisions.end());
    for (std::size_t i = 0; i < savedPosition.size() && i < resource.joints.size(); ++i) {
        resource.joints[i].position = savedPosition[i];
    }
    applyResource(scene, resource);
    plan.seconds = plan.samples.size() <= 1 ? 0 : (static_cast<double>(plan.samples.size()) - 1) * plan.dt;
    plan.ok = true;
    for (const std::string& warning : plan.warnings) {
        if (warning.find("unreachable") != std::string::npos || warning.find("limit") != std::string::npos) {
            plan.ok = false;
        }
    }
    return plan;
}

bool BuiltinRobotSolver::build(Scene& scene, std::string& error) {
    scene_ = &scene;
    error.clear();
    return true;
}

void BuiltinRobotSolver::shutdown() {
    playing_ = false;
    scene_ = nullptr;
}

bool BuiltinRobotSolver::arm(const std::string& pathName, std::string& error) {
    if (scene_ == nullptr) {
        error = "solver has no scene";
        return false;
    }
    const RobotPath* path = nullptr;
    for (const RobotPath& candidate : scene_->robotPaths) {
        if (candidate.name == pathName) {
            path = &candidate;
            break;
        }
    }
    if (path == nullptr) {
        error = "no path named " + pathName;
        return false;
    }
    KinematicResource* resource = findResource(scene_->resources, path->resource);
    if (resource == nullptr) {
        error = "no resource named " + path->resource;
        return false;
    }
    plan_ = planRobotPath(*scene_, *resource, *path);
    resourceName_ = resource->name;
    cursor_ = 0;
    playing_ = !plan_.samples.empty();
    if (playing_) {
        setJointVector(*resource, plan_.samples.front());
        applyResource(*scene_, *resource);
    }
    error.clear();
    return true;
}

void BuiltinRobotSolver::step(double dtSeconds) {
    if (!playing_ || scene_ == nullptr || plan_.samples.empty()) {
        return;
    }
    cursor_ += std::max(dtSeconds, 0.0);
    int index = static_cast<int>(std::floor(cursor_ / plan_.dt));
    if (index >= static_cast<int>(plan_.samples.size())) {
        index = static_cast<int>(plan_.samples.size()) - 1;
        playing_ = false;
    }
    index = std::max(index, 0);
    KinematicResource* resource = findResource(scene_->resources, resourceName_);
    if (resource == nullptr) {
        playing_ = false;
        return;
    }
    setJointVector(*resource, plan_.samples[static_cast<std::size_t>(index)]);
    applyResource(*scene_, *resource);
    // Collision sets already name the parts that matter. Hiding the rest keeps a
    // 100-solid robot from being voxelized on every tick; visibility is restored
    // before the viewport draws.
    const auto hidden = hideAllDuringQuery_ ? hideAllMeshedParts(*scene_) : hidePartsOutsideProximity(*scene_);
    const auto hits = collidingPairs(*scene_, *resource);
    for (const auto& node : hidden) {
        if (node) {
            node->visible = true;
        }
    }
    if (!hits.empty()) {
        highlightCollisions(*scene_, *resource, hits);
    }
}

std::unique_ptr<IRobotSolver> createRobotSolver() { return std::make_unique<BuiltinRobotSolver>(); }

}  // namespace sim
