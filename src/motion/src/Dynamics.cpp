#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {
namespace {

constexpr double kVelocityAccel = 1.0e9;

Vec3 column(const Mat4& matrix, int index) { return {matrix(index, 0), matrix(index, 1), matrix(index, 2)}; }

double pointMassAboutAxis(double mass, Vec3 point, Vec3 axisPoint, Vec3 axisDir) {
    if (mass <= 0) {
        return 0;
    }
    const Vec3 rel = point - axisPoint;
    const Vec3 along = axisDir * dot(rel, axisDir);
    const double distanceM = length(rel - along) / 1000.0;
    return mass * distanceM * distanceM;
}

void collectDriven(const KinematicResource& resource, const std::string& link, std::vector<std::string>& out) {
    if (link.empty() || std::find(out.begin(), out.end(), link) != out.end()) {
        return;
    }
    out.push_back(link);
    for (const KinJoint& joint : resource.joints) {
        if (joint.parentLink == link) {
            collectDriven(resource, joint.childLink, out);
        }
    }
    for (const KinLink& item : resource.links) {
        if (item.mountedOn == link) {
            collectDriven(resource, item.name, out);
        }
    }
}

const KinLink* linkNamed(const KinematicResource& resource, const std::string& name) {
    for (const KinLink& link : resource.links) {
        if (link.name == name) {
            return &link;
        }
    }
    return nullptr;
}

// linkInertia is kg·m² for a revolute joint and kg for a prismatic joint.
// payloadInertia uses the same unit for the flange load. Returns false when the driven links have no mass data.
bool measureInertia(const KinematicResource& resource, const KinJoint& joint, double& linkInertia, double& payloadInertia) {
    linkInertia = 0;
    payloadInertia = 0;
    std::vector<std::string> driven;
    collectDriven(resource, joint.childLink, driven);
    if (driven.empty()) {
        return false;
    }
    Mat4 parent = Mat4::identity();
    if (!joint.parentLink.empty() && !linkWorld(resource, joint.parentLink, parent)) {
        return false;
    }
    const Vec3 axisPoint = transformPoint(parent, joint.anchor);
    Vec3 axisDir = normalize(transformVector(parent, joint.axis));
    if (length(axisDir) < 1e-12) {
        axisDir = {0, 0, 1};
    }
    bool known = false;
    std::string tip = joint.childLink;
    int tipHops = 0;
    for (const std::string& name : driven) {
        int hops = 0;
        std::string cursor = name;
        for (int guard = 0; guard < 16; ++guard) {
            const KinJoint* owner = nullptr;
            for (const KinJoint& item : resource.joints) {
                if (item.childLink == cursor) {
                    owner = &item;
                    break;
                }
            }
            if (owner == nullptr) {
                break;
            }
            cursor = owner->parentLink;
            ++hops;
        }
        if (hops >= tipHops) {
            tipHops = hops;
            tip = name;
        }
        const KinLink* link = linkNamed(resource, name);
        if (link == nullptr || (!link->hasInertia && link->mass <= 0)) {
            continue;
        }
        Mat4 world = Mat4::identity();
        if (!linkWorld(resource, name, world)) {
            continue;
        }
        known = true;
        if (joint.motion == JointMotion::Prismatic) {
            linkInertia += std::max(0.0, link->mass);
            continue;
        }
        const Vec3 com = transformPoint(world, link->com * 1000.0);
        const Vec3 nLink{dot(column(world, 0), axisDir), dot(column(world, 1), axisDir), dot(column(world, 2), axisDir)};
        const double ix = link->ixx * nLink.x + link->ixy * nLink.y + link->ixz * nLink.z;
        const double iy = link->ixy * nLink.x + link->iyy * nLink.y + link->iyz * nLink.z;
        const double iz = link->ixz * nLink.x + link->iyz * nLink.y + link->izz * nLink.z;
        linkInertia += nLink.x * ix + nLink.y * iy + nLink.z * iz;
        linkInertia += pointMassAboutAxis(link->mass, com, axisPoint, axisDir);
    }
    if (resource.payload > 0) {
        Mat4 tipWorld = Mat4::identity();
        if (linkWorld(resource, tip, tipWorld)) {
            const Vec3 flange = {tipWorld(3, 0), tipWorld(3, 1), tipWorld(3, 2)};
            if (joint.motion == JointMotion::Prismatic) {
                payloadInertia = resource.payload;
            } else {
                payloadInertia = pointMassAboutAxis(resource.payload, flange, axisPoint, axisDir);
            }
            known = true;
        }
    }
    return known && (linkInertia + payloadInertia) > 1e-12;
}

JointAccelLimit fromUser(const KinematicResource& resource, const KinJoint& joint, double linkInertia, double payloadInertia,
                         bool known) {
    JointAccelLimit limit;
    limit.source = "user";
    limit.value = std::max(joint.maxAccel, 1e-6);
    if (known && linkInertia > 1e-9 && payloadInertia > 1e-12 && resource.payload > 0) {
        limit.value = joint.maxAccel * linkInertia / (linkInertia + payloadInertia);
        limit.payloadScaled = true;
    }
    return limit;
}

}  // namespace

JointAccelLimit jointAccelLimit(const KinematicResource& resource, const KinJoint& joint) {
    double linkInertia = 0;
    double payloadInertia = 0;
    const bool known = measureInertia(resource, joint, linkInertia, payloadInertia);
    const double total = linkInertia + payloadInertia;
    const bool wantDynamics = joint.accelSource == "dynamics" || joint.accelSource.empty();
    if (wantDynamics && joint.effort > 1e-9 && known && total > 1e-12) {
        JointAccelLimit limit;
        limit.source = "dynamics";
        double accel = joint.effort / total;
        if (joint.motion == JointMotion::Revolute) {
            accel *= 180.0 / kPi;
        } else {
            accel *= 1000.0;
        }
        limit.value = std::max(accel, 1e-6);
        limit.payloadScaled = resource.payload > 0 && payloadInertia > 1e-12;
        return limit;
    }
    if (joint.accelSource == "user" || joint.accelSource == "motionparameters" ||
        (joint.accelSource.empty() && joint.maxAccel > 0)) {
        JointAccelLimit limit = fromUser(resource, joint, linkInertia, payloadInertia, known);
        if (joint.accelSource == "motionparameters") {
            limit.source = "motionparameters";
        }
        return limit;
    }
    JointAccelLimit limit;
    limit.source = "velocity";
    limit.value = kVelocityAccel;
    return limit;
}

double cartesianAccelCap(const KinematicResource& resource, const std::vector<double>& start, const std::vector<double>& goal,
                         double distance, double requested) {
    double accel = std::max(requested, 1.0);
    const std::size_t count = std::min(start.size(), goal.size());
    for (std::size_t i = 0; i < count && i < resource.joints.size(); ++i) {
        const KinJoint& joint = resource.joints[i];
        if (!joint.coupledTo.empty()) {
            continue;
        }
        const double travel = std::abs(goal[i] - start[i]);
        if (travel < 1e-6) {
            continue;
        }
        const JointAccelLimit limit = jointAccelLimit(resource, joint);
        if (limit.source == "velocity") {
            continue;
        }
        accel = std::min(accel, std::max(limit.value * std::max(distance, 1e-6) / travel, 1.0));
    }
    return accel;
}

}  // namespace sim
