#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kScurveVel = 1.875;
constexpr double kScurveAcc = 5.773502691896258;

}  // namespace

double profileDuration(ProfileKind kind, double q0, double q1, double vmax, double amax) {
    const double distance = std::abs(q1 - q0);
    if (distance < 1e-12) {
        return 0;
    }
    vmax = std::max(vmax, 1e-6);
    amax = std::max(amax, 1e-6);
    if (kind == ProfileKind::SCurve) {
        return std::max(kScurveVel * distance / vmax, std::sqrt(kScurveAcc * distance / amax));
    }
    const double tAcc = vmax / amax;
    const double dAcc = 0.5 * amax * tAcc * tAcc;
    if (2.0 * dAcc >= distance) {
        return 2.0 * std::sqrt(distance / amax);
    }
    const double tCruise = (distance - 2.0 * dAcc) / vmax;
    return 2.0 * tAcc + tCruise;
}

double profileSample(ProfileKind kind, double q0, double q1, double vmax, double amax, double time, bool* done) {
    const double duration = profileDuration(kind, q0, q1, vmax, amax);
    if (done != nullptr) {
        *done = time >= duration - 1e-9;
    }
    if (duration <= 1e-12 || time >= duration) {
        return q1;
    }
    if (time <= 0) {
        return q0;
    }
    const double delta = q1 - q0;
    const double sign = delta >= 0 ? 1.0 : -1.0;
    const double distance = std::abs(delta);
    vmax = std::max(vmax, 1e-6);
    amax = std::max(amax, 1e-6);
    if (kind == ProfileKind::SCurve) {
        const double s = time / duration;
        const double blend = ((6.0 * s - 15.0) * s + 10.0) * s * s * s;
        return q0 + delta * blend;
    }
    double tAcc = vmax / amax;
    double dAcc = 0.5 * amax * tAcc * tAcc;
    double peak = vmax;
    if (2.0 * dAcc >= distance) {
        tAcc = std::sqrt(distance / amax);
        peak = amax * tAcc;
        dAcc = 0.5 * amax * tAcc * tAcc;
    }
    const double tCruise = 2.0 * dAcc >= distance - 1e-9 ? 0.0 : (distance - 2.0 * dAcc) / peak;
    double along = distance;
    if (time < tAcc) {
        along = 0.5 * amax * time * time;
    } else if (time < tAcc + tCruise) {
        along = dAcc + peak * (time - tAcc);
    } else {
        const double decel = time - tAcc - tCruise;
        along = dAcc + peak * tCruise + peak * decel - 0.5 * amax * decel * decel;
    }
    along = std::clamp(along, 0.0, distance);
    return q0 + sign * along;
}

}  // namespace sim
