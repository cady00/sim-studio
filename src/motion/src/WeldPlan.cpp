#include "sim/motion/WeldPlan.hpp"

#include "sim/core/Mesh.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sim {
namespace {

Quat rotationMappingZ(Vec3 zAxis) {
    zAxis = normalize(zAxis);
    if (length(zAxis) < 0.5) {
        return Quat::identity();
    }
    const Vec3 from{0, 0, 1};
    const Vec3 axis = cross(from, zAxis);
    const double sine = length(axis);
    const double cosine = dot(from, zAxis);
    if (sine < 1e-8) {
        if (cosine > 0) {
            return Quat::identity();
        }
        return Quat::fromAxisAngle({1, 0, 0}, kPi);
    }
    return Quat::fromAxisAngle(axis / sine, std::atan2(sine, cosine));
}

double jointMargin(const KinematicResource& resource, const std::vector<double>& joints) {
    double worst = 1;
    const int count = std::min(static_cast<int>(resource.joints.size()), static_cast<int>(joints.size()));
    for (int i = 0; i < count; ++i) {
        const KinJoint& joint = resource.joints[static_cast<std::size_t>(i)];
        if (!joint.limited) {
            continue;
        }
        const double span = std::max(1e-6, joint.upper - joint.lower);
        const double value = joints[static_cast<std::size_t>(i)];
        const double room = std::min(value - joint.lower, joint.upper - value) / (span * 0.5);
        worst = std::min(worst, room);
    }
    return std::clamp(worst, 0.0, 1.0);
}

double translationalManipulability(KinematicResource posed) {
    const int count = std::min(6, static_cast<int>(posed.joints.size()));
    if (count <= 0) {
        return 0;
    }
    const CartesianPose now = forwardModel(posed);
    double columns[3][6] = {};
    for (int i = 0; i < count; ++i) {
        const double saved = posed.joints[static_cast<std::size_t>(i)].position;
        const double eps = posed.joints[static_cast<std::size_t>(i)].motion == JointMotion::Prismatic ? 0.5 : 0.25;
        posed.joints[static_cast<std::size_t>(i)].position = saved + eps;
        const CartesianPose nudged = forwardModel(posed);
        posed.joints[static_cast<std::size_t>(i)].position = saved;
        columns[0][i] = (nudged.position.x - now.position.x) / eps;
        columns[1][i] = (nudged.position.y - now.position.y) / eps;
        columns[2][i] = (nudged.position.z - now.position.z) / eps;
    }
    double gram[3][3] = {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double sum = 0;
            for (int k = 0; k < count; ++k) {
                sum += columns[row][k] * columns[col][k];
            }
            gram[row][col] = sum;
        }
    }
    const double det = gram[0][0] * (gram[1][1] * gram[2][2] - gram[1][2] * gram[2][1]) -
                       gram[0][1] * (gram[1][0] * gram[2][2] - gram[1][2] * gram[2][0]) +
                       gram[0][2] * (gram[1][0] * gram[2][1] - gram[1][1] * gram[2][0]);
    return std::sqrt(std::abs(det));
}

bool wristSingular(const KinematicResource& resource, const std::vector<double>& joints) {
    if (resource.ik != "spherical_wrist" || joints.size() < 5) {
        return false;
    }
    return std::abs(joints[4]) < 2.0;
}

KinematicResource* ensureExampleRobot(Scene& scene) {
    if (KinematicResource* existing = findResource(scene.resources, "kr420")) {
        return existing;
    }
    KinematicResource robot;
    robot.name = "kr420";
    robot.ik = "spherical_wrist";
    robot.wrist.d1 = 675;
    robot.wrist.shoulder = 0;
    robot.wrist.a2 = 1200;
    robot.wrist.a3 = 1100;
    robot.wrist.d6 = 220;
    const double lowers[] = {-185, -130, -100, -350, -120, -350};
    const double uppers[] = {185, 20, 144, 350, 120, 350};
    for (int i = 0; i < 6; ++i) {
        KinJoint joint;
        joint.name = "A" + std::to_string(i + 1);
        joint.lower = lowers[i];
        joint.upper = uppers[i];
        joint.maxSpeed = 100;
        robot.joints.push_back(joint);
    }
    scene.resources.push_back(std::move(robot));
    return findResource(scene.resources, "kr420");
}

}  // namespace

CartesianPose weldTcpPose(const WeldSpot& spot, double spinDeg) {
    Vec3 into = -normalize(spot.normal);
    if (length(into) < 0.5) {
        into = {0, 0, -1};
    }
    const Quat align = rotationMappingZ(into);
    const Quat spin = Quat::fromAxisAngle(into, deg2rad(spinDeg));
    CartesianPose pose;
    pose.position = spot.position;
    pose.rotation = (spin * align).normalized();
    return pose;
}

SpinSearch searchWeldSpin(Scene* scene, KinematicResource& resource, const WeldSpot& spot, int samples, bool checkCollision) {
    SpinSearch best;
    best.note = "unreachable";
    const std::vector<double> saved = jointVector(resource);
    const int count = std::max(4, samples);
    double bestScore = -1e100;
    bool any = false;
    for (int i = 0; i < count; ++i) {
        const double spin = 360.0 * static_cast<double>(i) / static_cast<double>(count);
        const CartesianPose target = weldTcpPose(spot, spin);
        const IkResult solved = inverseModel(resource, target, saved);
        if (!solved.ok || solved.joints.empty()) {
            continue;
        }
        setJointVector(resource, solved.joints);
        const double margin = jointMargin(resource, solved.joints);
        const double manip = translationalManipulability(resource);
        const bool singular = wristSingular(resource, solved.joints);
        bool collision = false;
        if (checkCollision && scene != nullptr) {
            applyResource(*scene, resource);
            collision = !collidingPairs(*scene, resource).empty();
        }
        const double score = margin * 100.0 - (singular ? 40.0 : 0.0) - (collision ? 80.0 : 0.0) - std::abs(spin - 180.0) * 0.001;
        if (!any || score > bestScore) {
            any = true;
            bestScore = score;
            best.ok = true;
            best.spinDeg = spin;
            best.margin = margin;
            best.manipulability = manip;
            best.singular = singular;
            best.collision = collision;
            best.joints = solved.joints;
            best.tcp = forwardModel(resource);
            best.note.clear();
        }
    }
    setJointVector(resource, saved);
    if (scene != nullptr) {
        applyResource(*scene, resource);
    }
    if (!any) {
        best.ok = false;
        best.note = "unreachable";
    }
    return best;
}

void prepareExampleWelds(Scene& scene) {
    KinematicResource* robot = ensureExampleRobot(scene);
    if (robot == nullptr) {
        return;
    }
    const std::vector<double> saved = jointVector(*robot);
    struct Candidate {
        Vec3 position;
        Vec3 normal;
    };
    std::vector<Candidate> candidates;
    const double q1Values[] = {-15, 0, 15, 25};
    const double q2Values[] = {-55, -35, -15};
    const double q3Values[] = {30, 55, 80};
    for (double q1 : q1Values) {
        if (candidates.size() >= 4) {
            break;
        }
        for (double q2 : q2Values) {
            if (candidates.size() >= 4) {
                break;
            }
            for (double q3 : q3Values) {
                if (candidates.size() >= 4) {
                    break;
                }
                std::vector<double> seed(robot->joints.size(), 0);
                if (seed.size() >= 6) {
                    seed[0] = std::clamp(q1, robot->joints[0].lower, robot->joints[0].upper);
                    seed[1] = std::clamp(q2, robot->joints[1].lower, robot->joints[1].upper);
                    seed[2] = std::clamp(q3, robot->joints[2].lower, robot->joints[2].upper);
                    seed[4] = std::clamp(35.0, robot->joints[4].lower, robot->joints[4].upper);
                }
                setJointVector(*robot, seed);
                const CartesianPose pose = forwardModel(*robot);
                const Vec3 toolZ = pose.rotation.rotate({0, 0, 1});
                if (toolZ.z > -0.35) {
                    continue;
                }
                WeldSpot trial;
                trial.position = pose.position;
                trial.normal = -toolZ;
                const SpinSearch aligned = searchWeldSpin(nullptr, *robot, trial, 8, false);
                if (!aligned.ok || aligned.singular || aligned.margin < 0.05) {
                    continue;
                }
                candidates.push_back({pose.position, normalize(-toolZ)});
            }
        }
    }
    setJointVector(*robot, saved);
    if (candidates.size() < 2) {
        candidates.push_back({{900, -200, 400}, {0, 0, 1}});
        candidates.push_back({{900, 0, 400}, {0, 0, 1}});
        candidates.push_back({{1100, 0, 400}, {0, 0, 1}});
        candidates.push_back({{1100, 200, 400}, {0, 0, 1}});
    }
    Vec3 center{0, 0, 0};
    Vec3 normal{0, 0, 0};
    for (const Candidate& candidate : candidates) {
        center = center + candidate.position;
        normal = normal + candidate.normal;
    }
    center = center / static_cast<double>(candidates.size());
    normal = length(normal) > 0.5 ? normalize(normal) : Vec3{0, 0, 1};
    double spanX = 80;
    double spanY = 80;
    for (const Candidate& candidate : candidates) {
        spanX = std::max(spanX, std::abs(candidate.position.x - center.x) + 60);
        spanY = std::max(spanY, std::abs(candidate.position.y - center.y) + 60);
    }
    if (auto existing = scene.findByName("WeldPlate")) {
        scene.removeNode(existing->id);
    }
    auto plate = scene.makeNode(NodeKind::Part, "WeldPlate");
    plate->meshId = scene.addMesh(makeBoxMesh({spanX, spanY, 8}));
    plate->local.translation = center - normal * 8.0;
    plate->color = {0.72, 0.74, 0.78};
    scene.addChild(scene.root(), plate);

    std::ostringstream csv;
    csv << "id,x,y,z,nx,ny,nz,layers,electrode,example\n";
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const Vec3 authored = candidates[i].position + candidates[i].normal * 40.0;
        csv << "w" << (i + 1) << "," << authored.x << "," << authored.y << "," << authored.z << ","
            << candidates[i].normal.x << "," << candidates[i].normal.y << "," << candidates[i].normal.z
            << ",WeldPlate|panel_inner,gun_a,示例\n";
    }
    std::string error;
    importWeldCsv(csv.str(), scene.welds, error);
    projectWeldSpots(scene);
    for (WeldSpot& spot : scene.welds) {
        spot.example = true;
        const SpinSearch aligned = searchWeldSpin(&scene, *robot, spot, 12, false);
        spot.checked = true;
        spot.reachable = aligned.ok && !aligned.singular && aligned.margin > 0.02;
        spot.margin = aligned.margin;
        spot.spinDeg = aligned.spinDeg;
        spot.manipulability = aligned.manipulability;
        spot.singular = aligned.singular;
        spot.collision = aligned.collision;
        spot.electrode = "gun_a";
    }
    setJointVector(*robot, saved);
    applyResource(scene, *robot);
}

std::vector<SpotReach> checkWeldReach(Scene* scene, KinematicResource& resource, std::vector<WeldSpot>& spots, bool checkCollision) {
    std::vector<SpotReach> report;
    const std::vector<double> saved = jointVector(resource);
    for (WeldSpot& spot : spots) {
        SpinSearch aligned = searchWeldSpin(nullptr, resource, spot, 12, false);
        if (checkCollision && scene != nullptr && aligned.ok) {
            setJointVector(resource, aligned.joints);
            applyResource(*scene, resource);
            aligned.collision = !collidingPairs(*scene, resource).empty();
        }
        spot.checked = true;
        spot.margin = aligned.margin;
        spot.spinDeg = aligned.spinDeg;
        spot.manipulability = aligned.manipulability;
        spot.singular = aligned.singular;
        spot.collision = aligned.collision;
        spot.reachable = aligned.ok && !aligned.singular && aligned.margin > 0.02 && !aligned.collision;
        SpotReach row;
        row.id = spot.id;
        row.reachable = spot.reachable;
        row.margin = aligned.margin;
        row.manipulability = aligned.manipulability;
        row.singular = aligned.singular;
        row.collision = aligned.collision;
        row.joints = aligned.joints;
        report.push_back(std::move(row));
    }
    setJointVector(resource, saved);
    if (scene != nullptr) {
        applyResource(*scene, resource);
    }
    return report;
}

BaseRegion optimizeRobotBase(KinematicResource& resource, const std::vector<WeldSpot>& spots, double step, int cells) {
    BaseRegion region;
    const int grid = std::max(1, cells);
    const double stride = std::max(1.0, step);
    const std::vector<double> saved = jointVector(resource);
    const int half = grid / 2;
    for (int iy = 0; iy < grid; ++iy) {
        for (int ix = 0; ix < grid; ++ix) {
            BaseSample sample;
            sample.x = (ix - half) * stride;
            sample.y = (iy - half) * stride;
            for (const WeldSpot& spot : spots) {
                WeldSpot shifted = spot;
                shifted.position = spot.position - Vec3{sample.x, sample.y, 0};
                const SpinSearch aligned = searchWeldSpin(nullptr, resource, shifted, 4, false);
                if (aligned.ok && !aligned.singular && aligned.margin > 0.02) {
                    ++sample.covered;
                }
            }
            sample.coversAll = !spots.empty() && sample.covered == static_cast<int>(spots.size());
            region.bestCovered = std::max(region.bestCovered, sample.covered);
            if (sample.coversAll) {
                if (!region.found) {
                    region.minX = region.maxX = sample.x;
                    region.minY = region.maxY = sample.y;
                    region.found = true;
                } else {
                    region.minX = std::min(region.minX, sample.x);
                    region.maxX = std::max(region.maxX, sample.x);
                    region.minY = std::min(region.minY, sample.y);
                    region.maxY = std::max(region.maxY, sample.y);
                }
            }
            region.samples.push_back(sample);
        }
    }
    setJointVector(resource, saved);
    return region;
}

namespace {

struct TourRng {
    unsigned state = 1;
    double uniform() {
        state = state * 1664525u + 1013904223u;
        return static_cast<double>(state & 0x00FFFFFFu) / static_cast<double>(0x01000000u);
    }
};

double jointDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0;
    const std::size_t count = std::max(a.size(), b.size());
    for (std::size_t i = 0; i < count; ++i) {
        const double left = i < a.size() ? a[i] : 0;
        const double right = i < b.size() ? b[i] : 0;
        const double delta = left - right;
        sum += delta * delta;
    }
    return std::sqrt(sum);
}

std::vector<double> steerJoints(const KinematicResource& resource, const std::vector<double>& from, const std::vector<double>& toward,
                                double step) {
    std::vector<double> next = from;
    next.resize(std::max(from.size(), toward.size()), 0);
    const double distance = jointDistance(from, toward);
    const double scale = distance <= step || distance < 1e-9 ? 1.0 : step / distance;
    for (std::size_t i = 0; i < next.size(); ++i) {
        const double left = i < from.size() ? from[i] : 0;
        const double right = i < toward.size() ? toward[i] : 0;
        double value = left + (right - left) * scale;
        if (i < resource.joints.size() && resource.joints[i].limited) {
            value = std::clamp(value, resource.joints[i].lower, resource.joints[i].upper);
        }
        next[i] = value;
    }
    return next;
}

bool segmentBlocked(const std::vector<double>& from, const std::vector<double>& to,
                    const std::function<bool(const std::vector<double>&)>& blocked, int slices) {
    const int count = std::max(2, slices);
    for (int i = 1; i < count; ++i) {
        const double u = static_cast<double>(i) / static_cast<double>(count);
        std::vector<double> sample(std::max(from.size(), to.size()));
        for (std::size_t joint = 0; joint < sample.size(); ++joint) {
            const double left = joint < from.size() ? from[joint] : 0;
            const double right = joint < to.size() ? to[joint] : 0;
            sample[joint] = (1.0 - u) * left + u * right;
        }
        if (blocked(sample)) {
            return true;
        }
    }
    return false;
}

struct RrtNode {
    std::vector<double> q;
    int parent = -1;
};

int nearestNode(const std::vector<RrtNode>& tree, const std::vector<double>& target) {
    int best = 0;
    double bestDistance = 1e300;
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
        const double distance = jointDistance(tree[static_cast<std::size_t>(i)].q, target);
        if (distance < bestDistance) {
            bestDistance = distance;
            best = i;
        }
    }
    return best;
}

enum class ExtendStatus { Trapped, Advanced, Reached };

ExtendStatus extendTree(std::vector<RrtNode>& tree, const KinematicResource& resource, const std::vector<double>& target, double step,
                        const std::function<bool(const std::vector<double>&)>& blocked, int& added) {
    const int near = nearestNode(tree, target);
    const std::vector<double> next = steerJoints(resource, tree[static_cast<std::size_t>(near)].q, target, step);
    if (blocked(next) || segmentBlocked(tree[static_cast<std::size_t>(near)].q, next, blocked, 4)) {
        added = -1;
        return ExtendStatus::Trapped;
    }
    tree.push_back({next, near});
    added = static_cast<int>(tree.size()) - 1;
    if (jointDistance(next, target) <= step * 0.51) {
        return ExtendStatus::Reached;
    }
    return ExtendStatus::Advanced;
}

ExtendStatus connectTree(std::vector<RrtNode>& tree, const KinematicResource& resource, const std::vector<double>& target, double step,
                         const std::function<bool(const std::vector<double>&)>& blocked, int& added) {
    ExtendStatus status = ExtendStatus::Advanced;
    added = -1;
    for (int guard = 0; guard < 80; ++guard) {
        int fresh = -1;
        status = extendTree(tree, resource, target, step, blocked, fresh);
        if (status == ExtendStatus::Trapped) {
            return added < 0 ? ExtendStatus::Trapped : ExtendStatus::Advanced;
        }
        added = fresh;
        if (status == ExtendStatus::Reached) {
            return ExtendStatus::Reached;
        }
    }
    return ExtendStatus::Advanced;
}

std::vector<std::vector<double>> traceTree(const std::vector<RrtNode>& tree, int index) {
    std::vector<std::vector<double>> points;
    while (index >= 0) {
        points.push_back(tree[static_cast<std::size_t>(index)].q);
        index = tree[static_cast<std::size_t>(index)].parent;
    }
    std::reverse(points.begin(), points.end());
    return points;
}

double tourLength(const std::vector<Vec3>& points, const std::vector<int>& order) {
    double lengthSum = 0;
    for (std::size_t i = 1; i < order.size(); ++i) {
        lengthSum += length(points[static_cast<std::size_t>(order[i])] - points[static_cast<std::size_t>(order[i - 1])]);
    }
    return lengthSum;
}

}  // namespace

RrtResult planRrtConnect(const KinematicResource& resource, const std::vector<double>& start, const std::vector<double>& goal,
                         const std::function<bool(const std::vector<double>&)>& blocked, const RrtOptions& options) {
    RrtResult result;
    if (start.empty() || goal.empty()) {
        result.note = "missing endpoint";
        return result;
    }
    if (blocked(start) || blocked(goal)) {
        result.note = "endpoint is blocked";
        return result;
    }
    if (!segmentBlocked(start, goal, blocked, 12)) {
        result.ok = true;
        result.joints = {start, goal};
        result.nodes = 2;
        return result;
    }
    std::vector<RrtNode> startTree{{start, -1}};
    std::vector<RrtNode> goalTree{{goal, -1}};
    TourRng rng;
    rng.state = options.seed == 0 ? 1u : options.seed;
    const int iterations = std::max(1, options.iterations);
    const double step = std::max(1.0, options.stepDeg);
    bool startIsA = true;
    for (int iteration = 0; iteration < iterations; ++iteration) {
        std::vector<RrtNode>& treeA = startIsA ? startTree : goalTree;
        std::vector<RrtNode>& treeB = startIsA ? goalTree : startTree;
        std::vector<double> sample = start;
        if (rng.uniform() < options.goalBias) {
            sample = treeB.front().q;
        } else {
            sample.resize(resource.joints.size(), 0);
            for (std::size_t i = 0; i < sample.size(); ++i) {
                const KinJoint& joint = resource.joints[i];
                const double lower = joint.limited ? joint.lower : -180;
                const double upper = joint.limited ? joint.upper : 180;
                sample[i] = lower + (upper - lower) * rng.uniform();
            }
        }
        int added = -1;
        const ExtendStatus extended = extendTree(treeA, resource, sample, step, blocked, added);
        if (extended != ExtendStatus::Trapped && added >= 0) {
            int connected = -1;
            const ExtendStatus linked = connectTree(treeB, resource, treeA[static_cast<std::size_t>(added)].q, step, blocked, connected);
            if (linked == ExtendStatus::Reached && connected >= 0) {
                std::vector<std::vector<double>> left = traceTree(treeA, added);
                std::vector<std::vector<double>> right = traceTree(treeB, connected);
                std::reverse(right.begin(), right.end());
                if (!startIsA) {
                    std::swap(left, right);
                    std::reverse(left.begin(), left.end());
                    std::reverse(right.begin(), right.end());
                }
                result.joints = std::move(left);
                result.joints.insert(result.joints.end(), right.begin(), right.end());
                result.ok = true;
                result.nodes = static_cast<int>(startTree.size() + goalTree.size());
                return result;
            }
        }
        startIsA = !startIsA;
    }
    result.nodes = static_cast<int>(startTree.size() + goalTree.size());
    result.note = "no connection";
    return result;
}

std::vector<std::vector<double>> smoothJointPath(const std::vector<std::vector<double>>& path,
                                                 const std::function<bool(const std::vector<double>&)>& blocked, int passes) {
    std::vector<std::vector<double>> current = path;
    if (current.size() < 3) {
        return current;
    }
    TourRng rng;
    rng.state = 17;
    for (int pass = 0; pass < std::max(1, passes); ++pass) {
        bool shorter = false;
        for (int attempt = 0; attempt < static_cast<int>(current.size()) * 3; ++attempt) {
            if (current.size() < 3) {
                break;
            }
            int i = static_cast<int>(rng.uniform() * static_cast<double>(current.size() - 2));
            int j = i + 2 + static_cast<int>(rng.uniform() * static_cast<double>(current.size() - i - 2));
            if (j <= i + 1 || j >= static_cast<int>(current.size())) {
                continue;
            }
            if (segmentBlocked(current[static_cast<std::size_t>(i)], current[static_cast<std::size_t>(j)], blocked, 10)) {
                continue;
            }
            current.erase(current.begin() + i + 1, current.begin() + j);
            shorter = true;
        }
        if (!shorter) {
            break;
        }
    }
    return current;
}

std::vector<int> orderWeldSpots(const std::vector<Vec3>& points) {
    std::vector<int> order;
    if (points.empty()) {
        return order;
    }
    std::vector<char> used(points.size(), 0);
    order.push_back(0);
    used[0] = 1;
    while (order.size() < points.size()) {
        const Vec3 from = points[static_cast<std::size_t>(order.back())];
        int best = -1;
        double bestDistance = 1e300;
        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            if (used[static_cast<std::size_t>(i)]) {
                continue;
            }
            const double distance = length(points[static_cast<std::size_t>(i)] - from);
            if (distance < bestDistance) {
                bestDistance = distance;
                best = i;
            }
        }
        if (best < 0) {
            break;
        }
        used[static_cast<std::size_t>(best)] = 1;
        order.push_back(best);
    }
    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i + 1 < static_cast<int>(order.size()); ++i) {
            for (int j = i + 2; j < static_cast<int>(order.size()); ++j) {
                std::vector<int> trial = order;
                std::reverse(trial.begin() + i + 1, trial.begin() + j + 1);
                if (tourLength(points, trial) + 1e-6 < tourLength(points, order)) {
                    order = std::move(trial);
                    improved = true;
                }
            }
        }
    }
    return order;
}

int planExampleWeldTour(Scene& scene) {
    KinematicResource* robot = findResource(scene.resources, "kr420");
    if (robot == nullptr || scene.welds.empty()) {
        return 0;
    }
    const std::vector<SpotReach> reach = checkWeldReach(&scene, *robot, scene.welds, true);
    std::vector<int> reachable;
    std::vector<Vec3> positions;
    for (int i = 0; i < static_cast<int>(reach.size()); ++i) {
        const SpotReach& row = reach[static_cast<std::size_t>(i)];
        if (row.joints.empty() || row.singular || row.margin <= 0.02) {
            continue;
        }
        reachable.push_back(i);
        positions.push_back(scene.welds[static_cast<std::size_t>(i)].position);
    }
    if (reachable.size() < 2) {
        return 0;
    }
    const std::vector<int> localOrder = orderWeldSpots(positions);
    RobotPath tour;
    tour.name = "weld_tour";
    tour.resource = robot->name;
    RrtOptions options;
    options.iterations = 250;
    options.stepDeg = 10;
    for (std::size_t leg = 0; leg + 1 < localOrder.size(); ++leg) {
        const std::vector<double>& start = reach[static_cast<std::size_t>(reachable[static_cast<std::size_t>(localOrder[leg])])].joints;
        const std::vector<double>& goal = reach[static_cast<std::size_t>(reachable[static_cast<std::size_t>(localOrder[leg + 1])])].joints;
        std::vector<double> mid(start.size());
        for (std::size_t joint = 0; joint < mid.size(); ++joint) {
            const double left = joint < start.size() ? start[joint] : 0;
            const double right = joint < goal.size() ? goal[joint] : 0;
            mid[joint] = 0.5 * (left + right);
        }
        const auto blocked = [mid](const std::vector<double>& q) {
            if (q.size() < 2 || mid.size() < 2) {
                return false;
            }
            return std::abs(q[0] - mid[0]) < 5.0 && std::abs(q[1] - mid[1]) < 5.0;
        };
        RrtResult planned = planRrtConnect(*robot, start, goal, blocked, options);
        std::vector<std::vector<double>> joints = planned.ok ? smoothJointPath(planned.joints, blocked, 4) : std::vector<std::vector<double>>{start, goal};
        if (joints.size() > 8) {
            std::vector<std::vector<double>> reduced{joints.front()};
            const int stride = std::max(1, static_cast<int>(joints.size()) / 6);
            for (int index = stride; index + 1 < static_cast<int>(joints.size()); index += stride) {
                reduced.push_back(joints[static_cast<std::size_t>(index)]);
            }
            reduced.push_back(joints.back());
            joints.swap(reduced);
        }
        for (std::size_t index = (leg == 0 ? 0 : 1); index < joints.size(); ++index) {
            setJointVector(*robot, joints[index]);
            const CartesianPose pose = forwardModel(*robot);
            PathPoint point;
            point.name = "P" + std::to_string(tour.points.size() + 1);
            point.move = PathMove::Ptp;
            point.joints = joints[index];
            point.position = pose.position;
            point.orientation = pose.rotation;
            point.speed = 40;
            tour.points.push_back(std::move(point));
        }
    }
    scene.robotPaths.erase(std::remove_if(scene.robotPaths.begin(), scene.robotPaths.end(),
                                          [](const RobotPath& path) { return path.name == "weld_tour"; }),
                           scene.robotPaths.end());
    const int count = static_cast<int>(tour.points.size());
    scene.robotPaths.push_back(std::move(tour));
    return count;
}

}  // namespace sim
