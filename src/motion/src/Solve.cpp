#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace sim {
namespace {

constexpr int kIkIterations = 40;

Vec3 rotationVector(Quat from, Quat to) {
    Quat delta = (to.normalized() * from.normalized().conjugate()).normalized();
    if (delta.w < 0) {
        delta = Quat{-delta.w, -delta.x, -delta.y, -delta.z};
    }
    const double xyz = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    const double angle = 2.0 * std::atan2(xyz, delta.w);
    if (xyz < 1e-12 || angle < 1e-12) {
        return {};
    }
    return Vec3{delta.x, delta.y, delta.z} * (angle / xyz);
}

void readAngles(const KinematicResource& resource, double* out, int count) {
    for (int i = 0; i < count; ++i) {
        out[i] = 0;
        if (i < static_cast<int>(resource.joints.size())) {
            out[i] = jointCommand(resource, resource.joints[static_cast<std::size_t>(i)]);
        }
    }
}

bool withinLimits(const KinematicResource& resource, const std::vector<double>& q, std::string& warning) {
    const int n = std::min(static_cast<int>(q.size()), static_cast<int>(resource.joints.size()));
    for (int i = 0; i < n; ++i) {
        const KinJoint& joint = resource.joints[static_cast<std::size_t>(i)];
        if (!joint.coupledTo.empty() || !joint.limited) {
            continue;
        }
        if (q[static_cast<std::size_t>(i)] < joint.lower - 1e-4 || q[static_cast<std::size_t>(i)] > joint.upper + 1e-4) {
            warning = joint.name.empty() ? "joint limit" : joint.name + " limit";
            return false;
        }
    }
    return true;
}

CartesianPose poseErrorFrame(const CartesianPose& current, const CartesianPose& target, double error[6]) {
    const Vec3 dp = target.position - current.position;
    const Vec3 w = rotationVector(current.rotation, target.rotation);
    error[0] = dp.x;
    error[1] = dp.y;
    error[2] = dp.z;
    error[3] = w.x;
    error[4] = w.y;
    error[5] = w.z;
    return current;
}

bool solve6(double a[6][7], double x[6]) {
    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 6; ++row) {
            if (std::abs(a[row][col]) > std::abs(a[pivot][col])) {
                pivot = row;
            }
        }
        if (std::abs(a[pivot][col]) < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (int k = col; k < 7; ++k) {
                std::swap(a[pivot][k], a[col][k]);
            }
        }
        const double div = a[col][col];
        for (int k = col; k < 7; ++k) {
            a[col][k] /= div;
        }
        for (int row = 0; row < 6; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (int k = col; k < 7; ++k) {
                a[row][k] -= factor * a[col][k];
            }
        }
    }
    for (int i = 0; i < 6; ++i) {
        x[i] = a[i][6];
    }
    return true;
}

IkResult numericalSolve(const KinematicResource& resource, const CartesianPose& target, std::vector<double> seed, int count) {
    IkResult result;
    if (count <= 0) {
        result.warning = "no joints";
        return result;
    }
    if (static_cast<int>(seed.size()) < count) {
        seed.assign(static_cast<std::size_t>(count), 0);
        readAngles(resource, seed.data(), count);
    }
    std::vector<double> q = seed;
    q.resize(static_cast<std::size_t>(count));
    KinematicResource trial = resource;
    while (static_cast<int>(trial.joints.size()) < count) {
        KinJoint extra;
        extra.name = "j" + std::to_string(trial.joints.size() + 1);
        extra.limited = false;
        trial.joints.push_back(extra);
    }
    for (int iter = 0; iter < kIkIterations; ++iter) {
        for (int i = 0; i < count; ++i) {
            trial.joints[static_cast<std::size_t>(i)].position = q[static_cast<std::size_t>(i)];
            trial.joints[static_cast<std::size_t>(i)].coupledTo.clear();
        }
        const CartesianPose now = forwardModel(trial);
        double error[6];
        poseErrorFrame(now, target, error);
        const double pos = std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
        const double ang = std::sqrt(error[3] * error[3] + error[4] * error[4] + error[5] * error[5]);
        if (pos < 0.05 && ang < deg2rad(0.05)) {
            result.ok = true;
            result.approximated = iter > 0;
            result.joints = q;
            withinLimits(resource, q, result.warning);
            if (!result.warning.empty()) {
                result.ok = false;
            }
            return result;
        }
        double columns[6][6];
        for (int i = 0; i < count; ++i) {
            const double eps = trial.joints[static_cast<std::size_t>(i)].motion == JointMotion::Prismatic ? 0.05 : 0.05;
            trial.joints[static_cast<std::size_t>(i)].position = q[static_cast<std::size_t>(i)] + eps;
            const CartesianPose nudged = forwardModel(trial);
            trial.joints[static_cast<std::size_t>(i)].position = q[static_cast<std::size_t>(i)];
            double nudgedError[6];
            poseErrorFrame(now, nudged, nudgedError);
            for (int row = 0; row < 6; ++row) {
                columns[row][i] = nudgedError[row] / eps;
            }
        }
        for (int i = count; i < 6; ++i) {
            for (int row = 0; row < 6; ++row) {
                columns[row][i] = 0;
            }
        }
        constexpr double lambda = 1e-2;
        double normal[6][7] = {};
        for (int r = 0; r < 6; ++r) {
            for (int c = 0; c < 6; ++c) {
                double dot = (r == c) ? lambda * lambda : 0;
                for (int k = 0; k < count; ++k) {
                    dot += columns[r][k] * columns[c][k];
                }
                normal[r][c] = dot;
            }
            normal[r][6] = error[r];
        }
        double task[6];
        if (!solve6(normal, task)) {
            break;
        }
        for (int i = 0; i < count; ++i) {
            double step = 0;
            for (int row = 0; row < 6; ++row) {
                step += columns[row][i] * task[row];
            }
            step = std::clamp(step, -8.0, 8.0);
            q[static_cast<std::size_t>(i)] += step;
            if (i < static_cast<int>(resource.joints.size())) {
                q[static_cast<std::size_t>(i)] = clampJoint(resource.joints[static_cast<std::size_t>(i)], q[static_cast<std::size_t>(i)]);
            }
        }
    }
    result.warning = "unreachable";
    result.joints = q;
    result.approximated = true;
    return result;
}

void wristAngles(const SphericalWrist& arm, const double qDeg[6], double qRad[6], CartesianPose& pose) {
    for (int i = 0; i < 6; ++i) {
        qRad[i] = deg2rad(qDeg[i]);
    }
    const double q1 = qRad[0];
    const double q2 = qRad[1];
    const double q3 = qRad[2];
    const double c1 = std::cos(q1);
    const double s1 = std::sin(q1);
    const double c2 = std::cos(q2);
    const double s2 = std::sin(q2);
    const double c23 = std::cos(q2 + q3);
    const double s23 = std::sin(q2 + q3);
    const double xArm = arm.a2 * c2 + arm.a3 * c23;
    const double zArm = -(arm.a2 * s2 + arm.a3 * s23);
    const double radial = arm.shoulder + xArm;
    const Vec3 wrist{c1 * radial, s1 * radial, arm.d1 + zArm};
    const Quat rotation = Quat::fromAxisAngle({0, 0, 1}, q1) * Quat::fromAxisAngle({0, 1, 0}, q2 + q3) *
                          Quat::fromAxisAngle({1, 0, 0}, qRad[3]) * Quat::fromAxisAngle({0, 1, 0}, qRad[4]) *
                          Quat::fromAxisAngle({0, 0, 1}, qRad[5]);
    pose.rotation = (rotation * arm.tcpRotation).normalized();
    pose.position = wrist + rotation.rotate({0, 0, arm.d6}) + pose.rotation.rotate(arm.tcp);
}

bool decomposeWrist(Quat wrist, double& q4, double& q5, double& q6) {
    const Mat4 m = Transform{{0, 0, 0}, wrist.normalized(), {1, 1, 1}}.matrix();
    const double r02 = m(2, 0);
    const double r00 = m(0, 0);
    const double r01 = m(1, 0);
    const double r12 = m(2, 1);
    const double r22 = m(2, 2);
    const double r10 = m(0, 1);
    const double r11 = m(1, 1);
    if (std::abs(r02) > 0.999999) {
        q5 = std::asin(std::clamp(r02, -1.0, 1.0));
        q4 = 0;
        q6 = std::atan2(r10, r11);
        return true;
    }
    q5 = std::asin(std::clamp(r02, -1.0, 1.0));
    const double c5 = std::cos(q5);
    q4 = std::atan2(-r12 / c5, r22 / c5);
    q6 = std::atan2(-r01 / c5, r00 / c5);
    return true;
}

IkResult inverseWrist(const KinematicResource& resource, const CartesianPose& target, const std::vector<double>& seed) {
    const SphericalWrist& arm = resource.wrist;
    double seedDeg[6] = {};
    if (seed.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            seedDeg[i] = seed[static_cast<std::size_t>(i)];
        }
    } else {
        readAngles(resource, seedDeg, 6);
    }
    const Quat tool = (target.rotation * arm.tcpRotation.conjugate()).normalized();
    const Vec3 flange = target.position - target.rotation.rotate(arm.tcp);
    const Vec3 approach = tool.rotate({0, 0, 1});
    const Vec3 wrist = flange - approach * arm.d6;
    const double r = std::hypot(wrist.x, wrist.y);
    const double shoulders[2] = {std::atan2(wrist.y, wrist.x), std::atan2(wrist.y, wrist.x) + kPi};
    const double radials[2] = {r, -r};
    IkResult best;
    best.warning = "unreachable";
    double bestScore = std::numeric_limits<double>::infinity();
    for (int shoulder = 0; shoulder < 2; ++shoulder) {
        const double xArm = radials[shoulder] - arm.shoulder;
        const double zArm = arm.d1 - wrist.z;
        const double reach2 = xArm * xArm + zArm * zArm;
        double cosine = (reach2 - arm.a2 * arm.a2 - arm.a3 * arm.a3) / (2.0 * arm.a2 * arm.a3);
        if (cosine > 1.0 + 1e-6 || cosine < -1.0 - 1e-6 || arm.a2 <= 1e-6 || arm.a3 <= 1e-6) {
            continue;
        }
        cosine = std::clamp(cosine, -1.0, 1.0);
        const double sine = std::sqrt(std::max(0.0, 1.0 - cosine * cosine));
        const double elbows[2] = {std::atan2(sine, cosine), std::atan2(-sine, cosine)};
        for (double q3 : elbows) {
            const double q2 = std::atan2(zArm, xArm) - std::atan2(arm.a3 * std::sin(q3), arm.a2 + arm.a3 * std::cos(q3));
            const Quat armRotation = Quat::fromAxisAngle({0, 0, 1}, shoulders[shoulder]) * Quat::fromAxisAngle({0, 1, 0}, q2 + q3);
            const Quat wristQ = (armRotation.conjugate() * tool).normalized();
            double q4 = 0;
            double q5 = 0;
            double q6 = 0;
            decomposeWrist(wristQ, q4, q5, q6);
            const double candidate[6] = {rad2deg(shoulders[shoulder]), rad2deg(q2), rad2deg(q3),
                                         rad2deg(q4), rad2deg(q5), rad2deg(q6)};
            double wrapped[6];
            for (int i = 0; i < 6; ++i) {
                wrapped[i] = candidate[i];
                if (i != 2) {
                    while (wrapped[i] - seedDeg[i] > 180.0) {
                        wrapped[i] -= 360.0;
                    }
                    while (wrapped[i] - seedDeg[i] < -180.0) {
                        wrapped[i] += 360.0;
                    }
                }
            }
            KinematicResource trial = resource;
            trial.ik = "spherical_wrist";
            while (trial.joints.size() < 6) {
                KinJoint extra;
                extra.limited = false;
                trial.joints.push_back(extra);
            }
            for (int i = 0; i < 6; ++i) {
                trial.joints[static_cast<std::size_t>(i)].position = wrapped[i];
                trial.joints[static_cast<std::size_t>(i)].coupledTo.clear();
                trial.joints[static_cast<std::size_t>(i)].limited = false;
            }
            const CartesianPose check = forwardModel(trial);
            const double pos = length(check.position - target.position);
            const double ang = length(rotationVector(check.rotation, target.rotation));
            if (pos > 0.05 || ang > deg2rad(0.05)) {
                continue;
            }
            std::string limitWarning;
            const bool inside = withinLimits(resource, {wrapped, wrapped + 6}, limitWarning);
            double score = 0;
            for (int i = 0; i < 6; ++i) {
                const double d = wrapped[i] - seedDeg[i];
                score += d * d;
            }
            if (!inside) {
                score += 1e6;
            }
            if (score < bestScore) {
                bestScore = score;
                best.ok = inside;
                best.approximated = false;
                best.warning = inside ? std::string{} : limitWarning;
                best.joints.assign(wrapped, wrapped + 6);
            }
        }
    }
    if (!best.ok) {
        IkResult numeric = numericalSolve(resource, target, seed.size() >= 6 ? seed : std::vector<double>(seedDeg, seedDeg + 6), 6);
        numeric.approximated = true;
        if (numeric.ok) {
            numeric.warning = best.warning.empty() ? "numerical fallback" : best.warning;
            return numeric;
        }
        if (best.joints.empty()) {
            return numeric;
        }
    }
    return best;
}

IkResult inverseScara(const KinematicResource& resource, const CartesianPose& target, const std::vector<double>& seed) {
    const ScaraArm& arm = resource.scara;
    double seedQ[4] = {};
    if (seed.size() >= 4) {
        for (int i = 0; i < 4; ++i) {
            seedQ[i] = seed[static_cast<std::size_t>(i)];
        }
    } else {
        readAngles(resource, seedQ, 4);
    }
    const Mat4 orientation = Transform{{0, 0, 0}, target.rotation.normalized(), {1, 1, 1}}.matrix();
    const double yaw = std::atan2(orientation(0, 1), orientation(0, 0));
    const Vec3 tool = target.position - target.rotation.rotate(arm.tcp);
    const double reach2 = tool.x * tool.x + tool.y * tool.y;
    double cosine = (reach2 - arm.l1 * arm.l1 - arm.l2 * arm.l2) / (2.0 * arm.l1 * arm.l2);
    IkResult result;
    if (cosine > 1.0 + 1e-6 || cosine < -1.0 - 1e-6 || arm.l1 <= 1e-6 || arm.l2 <= 1e-6) {
        result.warning = "unreachable";
        result.approximated = true;
        return numericalSolve(resource, target, {seedQ, seedQ + 4}, 4);
    }
    cosine = std::clamp(cosine, -1.0, 1.0);
    const double sine = std::sqrt(std::max(0.0, 1.0 - cosine * cosine));
    const double elbows[2] = {std::atan2(sine, cosine), std::atan2(-sine, cosine)};
    double best = std::numeric_limits<double>::infinity();
    for (double q2 : elbows) {
        const double q1 = std::atan2(tool.y, tool.x) - std::atan2(arm.l2 * std::sin(q2), arm.l1 + arm.l2 * std::cos(q2));
        const double q3 = tool.z - arm.baseZ;
        double q4 = yaw - q1 - q2;
        double deg[4] = {rad2deg(q1), rad2deg(q2), q3, rad2deg(q4)};
        for (int i : {0, 1, 3}) {
            while (deg[i] - seedQ[i] > 180.0) {
                deg[i] -= 360.0;
            }
            while (deg[i] - seedQ[i] < -180.0) {
                deg[i] += 360.0;
            }
        }
        std::string warning;
        const bool inside = withinLimits(resource, {deg, deg + 4}, warning);
        double score = 0;
        for (int i = 0; i < 4; ++i) {
            const double d = deg[i] - seedQ[i];
            score += d * d;
        }
        if (!inside) {
            score += 1e6;
        }
        if (score < best) {
            best = score;
            result.ok = inside;
            result.warning = inside ? std::string{} : warning;
            result.joints.assign(deg, deg + 4);
            result.approximated = false;
        }
    }
    if (!result.ok) {
        IkResult numeric = numericalSolve(resource, target, {seedQ, seedQ + 4}, 4);
        if (numeric.ok) {
            numeric.approximated = true;
            numeric.warning = result.warning.empty() ? "numerical fallback" : result.warning;
            return numeric;
        }
    }
    return result;
}

Quat rotationOf(const Mat4& m) {
    const double trace = m(0, 0) + m(1, 1) + m(2, 2);
    Quat q;
    if (trace > 0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q.w = 0.25 * s;
        q.x = (m(1, 2) - m(2, 1)) / s;
        q.y = (m(2, 0) - m(0, 2)) / s;
        q.z = (m(0, 1) - m(1, 0)) / s;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;
        q.w = (m(1, 2) - m(2, 1)) / s;
        q.x = 0.25 * s;
        q.y = (m(1, 0) + m(0, 1)) / s;
        q.z = (m(2, 0) + m(0, 2)) / s;
    } else if (m(1, 1) > m(2, 2)) {
        const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;
        q.w = (m(2, 0) - m(0, 2)) / s;
        q.x = (m(1, 0) + m(0, 1)) / s;
        q.y = 0.25 * s;
        q.z = (m(2, 1) + m(1, 2)) / s;
    } else {
        const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;
        q.w = (m(0, 1) - m(1, 0)) / s;
        q.x = (m(2, 0) + m(0, 2)) / s;
        q.y = (m(2, 1) + m(1, 2)) / s;
        q.z = 0.25 * s;
    }
    return q.normalized();
}

CartesianPose forwardChain(const KinematicResource& resource) {
    std::unordered_map<std::string, Mat4> world;
    std::unordered_map<std::string, int> childOf;
    for (int i = 0; i < static_cast<int>(resource.joints.size()); ++i) {
        childOf[resource.joints[static_cast<std::size_t>(i)].childLink] = i;
    }
    for (const KinLink& link : resource.links) {
        if (!childOf.count(link.name)) {
            world[link.name] = Mat4::identity();
        }
    }
    bool moved = true;
    for (int guard = 0; guard < static_cast<int>(resource.joints.size()) + 2 && moved; ++guard) {
        moved = false;
        for (const KinJoint& joint : resource.joints) {
            if (world.count(joint.childLink) != 0 || world.count(joint.parentLink) == 0) {
                continue;
            }
            const double position = jointCommand(resource, joint);
            Transform local = joint.restLocal;
            const Vec3 axis = normalize(joint.axis);
            if (joint.motion == JointMotion::Prismatic) {
                local.translation = joint.restLocal.translation + axis * position;
            } else {
                const Quat delta = Quat::fromAxisAngle(axis, deg2rad(position));
                local.rotation = (delta * joint.restLocal.rotation).normalized();
                const Vec3 offset = joint.restLocal.translation - joint.anchor;
                local.translation = joint.anchor + delta.rotate(offset);
            }
            world[joint.childLink] = mul(world.at(joint.parentLink), local.matrix());
            moved = true;
        }
    }
    std::string tip;
    int depth = -1;
    for (const KinLink& link : resource.links) {
        int hops = 0;
        std::string cursor = link.name;
        for (int i = 0; i < 16 && childOf.count(cursor) != 0; ++i) {
            cursor = resource.joints[static_cast<std::size_t>(childOf[cursor])].parentLink;
            ++hops;
        }
        if (hops > depth) {
            depth = hops;
            tip = link.name;
        }
    }
    CartesianPose pose;
    if (tip.empty() || world.count(tip) == 0) {
        return pose;
    }
    const NamedFrame* frame = nullptr;
    for (const NamedFrame& candidate : resource.frames) {
        if (candidate.name == "tcp") {
            frame = &candidate;
            break;
        }
    }
    const Vec3 localPoint = frame != nullptr ? frame->origin : Vec3{};
    const Quat localRotation = frame != nullptr ? frame->rotation : Quat::identity();
    pose.position = transformPoint(world.at(tip), localPoint);
    pose.rotation = (rotationOf(world.at(tip)) * localRotation).normalized();
    return pose;
}

}  // namespace

double jointCommand(const KinematicResource& resource, const KinJoint& joint, int depth);

double jointCommand(const KinematicResource& resource, const KinJoint& joint) {
    return jointCommand(resource, joint, 0);
}

double jointCommand(const KinematicResource& resource, const KinJoint& joint, int depth) {
    if (!joint.coupledTo.empty() && depth < 8) {
        if (const KinJoint* master = findJoint(resource, joint.coupledTo)) {
            return clampJoint(joint, jointCommand(resource, *master, depth + 1) * joint.coupleRatio);
        }
    }
    return clampJoint(joint, joint.position);
}

CartesianPose forwardModel(const KinematicResource& resource) {
    if (resource.ik == "spherical_wrist") {
        double q[6];
        readAngles(resource, q, 6);
        CartesianPose pose;
        double unused[6];
        wristAngles(resource.wrist, q, unused, pose);
        return pose;
    }
    if (resource.ik == "scara") {
        double q[4];
        readAngles(resource, q, 4);
        const double q1 = deg2rad(q[0]);
        const double q2 = deg2rad(q[1]);
        const double q4 = deg2rad(q[3]);
        const double yaw = q1 + q2 + q4;
        const Quat rotation = Quat::fromAxisAngle({0, 0, 1}, yaw);
        CartesianPose pose;
        pose.rotation = rotation;
        pose.position = {resource.scara.l1 * std::cos(q1) + resource.scara.l2 * std::cos(q1 + q2),
                         resource.scara.l1 * std::sin(q1) + resource.scara.l2 * std::sin(q1 + q2),
                         resource.scara.baseZ + q[2]};
        pose.position = pose.position + rotation.rotate(resource.scara.tcp);
        return pose;
    }
    if (resource.ik == "numerical") {
        return forwardChain(resource);
    }
    CartesianPose pose;
    return pose;
}

IkResult inverseModel(const KinematicResource& resource, const CartesianPose& target, const std::vector<double>& seed) {
    if (resource.ik == "spherical_wrist") {
        return inverseWrist(resource, target, seed);
    }
    if (resource.ik == "numerical") {
        IkResult numeric = numericalSolve(resource, target, seed, static_cast<int>(resource.joints.size()));
        numeric.approximated = true;
        return numeric;
    }
    if (resource.ik == "scara") {
        return inverseScara(resource, target, seed);
    }
    const int count = std::max(1, static_cast<int>(resource.joints.size()));
    return numericalSolve(resource, target, seed, count);
}

}  // namespace sim
