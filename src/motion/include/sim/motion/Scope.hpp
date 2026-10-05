#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace sim {

class Scene;

// Samples named signals and joint commands against simulation time.
// A channel id is "sig:<name>" or "jnt:<resource>/<joint>".
// Velocity and acceleration are backward differences of a joint channel.
// The trace keeps the newest kMaxSamples rows.
struct ScopeTrace {
    static constexpr std::size_t kMaxSamples = 240000;

    std::vector<double> time;
    struct Series {
        std::string id;
        std::vector<double> value;
    };
    std::vector<std::string> channels;
    std::vector<Series> series;

    void clear();
    void watch(const std::vector<std::string>& ids);
    void sample(const Scene& scene, double timeSeconds);
    // Last sample at or before timeSeconds. Empty trace returns 0.
    double valueAt(const std::string& id, double timeSeconds) const;
    std::vector<double> velocity(const std::string& jointId) const;
    std::vector<double> acceleration(const std::string& jointId) const;
    // Joint columns gain ".v" and ".a" when those flags are set.
    std::string toCsv(bool velocityColumns, bool accelerationColumns) const;

    const Series* findSeries(const std::string& id) const;
};

}  // namespace sim
