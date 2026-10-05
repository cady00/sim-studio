#pragma once

#include "sim/core/Math.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace sim {

// Positive when the AABBs are apart, zero when they touch, negative when they overlap.
// The magnitude of a negative value is the smallest axis overlap, so a deeper overlap
// sorts first. This is the default collision-likelihood feature.
inline double aabbSeparation(Vec3 aMin, Vec3 aMax, Vec3 bMin, Vec3 bMax) {
    double outside = 0;
    double overlap = 1e300;
    const double a0[3] = {aMin.x, aMin.y, aMin.z};
    const double a1[3] = {aMax.x, aMax.y, aMax.z};
    const double b0[3] = {bMin.x, bMin.y, bMin.z};
    const double b1[3] = {bMax.x, bMax.y, bMax.z};
    for (int axis = 0; axis < 3; ++axis) {
        if (a1[axis] < b0[axis]) {
            const double gap = b0[axis] - a1[axis];
            outside += gap * gap;
        } else if (b1[axis] < a0[axis]) {
            const double gap = a0[axis] - b1[axis];
            outside += gap * gap;
        } else {
            overlap = std::min(overlap, std::min(a1[axis], b1[axis]) - std::max(a0[axis], b0[axis]));
        }
    }
    if (outside > 0) {
        return std::sqrt(outside);
    }
    return -overlap;
}

inline double aabbSeparation(const Aabb& a, const Aabb& b) {
    if (!a.valid || !b.valid) {
        return 0;
    }
    return aabbSeparation(a.min, a.max, b.min, b.max);
}

// Ranks narrow-phase candidates. A lower score is tested sooner.
// `rankLearned` is the reserved model hook. It returns false when no model is loaded,
// and the distance-plus-previous-frame score is used instead. No weights ship with the product.
class ICollisionLikelihood {
public:
    virtual ~ICollisionLikelihood() = default;
    virtual const char* backendName() const = 0;
    virtual bool rankLearned(double gapMm, bool previousHit, double& scoreOut, std::string& error) const {
        (void)gapMm;
        (void)previousHit;
        (void)scoreOut;
        error.clear();
        return false;
    }
    virtual double score(double gapMm, bool previousHit) const {
        double learned = 0;
        std::string error;
        if (rankLearned(gapMm, previousHit, learned, error)) {
            return learned;
        }
        return gapMm + (previousHit ? -1.0e12 : 0.0);
    }
};

class DistancePreviousLikelihood : public ICollisionLikelihood {
public:
    const char* backendName() const override { return "distance-and-previous"; }
};

}  // namespace sim
