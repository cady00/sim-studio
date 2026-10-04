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

}  // namespace sim
