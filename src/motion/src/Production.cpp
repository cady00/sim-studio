#include "sim/motion/Motion.hpp"

#include "sim/core/LineModel.hpp"
#include "sim/core/Scene.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

double mergedOccupancy(const ScheduleReport& report, const std::string& station) {
    std::vector<std::pair<double, double>> spans;
    for (const ScheduleBar& bar : report.bars) {
        if (bar.station == station && bar.finish > bar.start) {
            spans.emplace_back(bar.start, bar.finish);
        }
    }
    if (spans.empty()) {
        return 0;
    }
    std::sort(spans.begin(), spans.end());
    double total = 0;
    double left = spans.front().first;
    double right = spans.front().second;
    for (std::size_t i = 1; i < spans.size(); ++i) {
        if (spans[i].first <= right) {
            right = std::max(right, spans[i].second);
        } else {
            total += right - left;
            left = spans[i].first;
            right = spans[i].second;
        }
    }
    return total + (right - left);
}

double fallbackOccupancy(const std::vector<Operation>& operations, const std::string& station) {
    double longest = 0;
    for (const Operation& operation : operations) {
        if (operation.kind == OpKind::Task && operation.station == station) {
            longest = std::max(longest, std::max(0.0, operation.seconds) + std::max(0.0, operation.wait));
        }
    }
    return longest;
}

}  // namespace

bool LineProduction::start(const Scene& scene, std::string& error) {
    stages_.clear();
    input_ = false;
    sourceClock_ = 0;
    time_ = 0;
    output_ = 0;
    sourceBlocked_ = 0;
    running_ = false;
    if (scene.layout.stations.empty()) {
        error = "line has no stations";
        return false;
    }
    std::vector<Station> stations = scene.layout.stations;
    std::sort(stations.begin(), stations.end(), [](const Station& a, const Station& b) { return a.x < b.x; });
    const ScheduleReport report = scheduleOperations(scene.operations, scene.layout);
    for (const Station& station : stations) {
        double process = report.error.empty() ? mergedOccupancy(report, station.id) : 0;
        if (process <= 1e-9) {
            process = fallbackOccupancy(scene.operations, station.id);
        }
        if (process <= 1e-9) {
            continue;
        }
        Stage stage;
        stage.id = station.id;
        stage.process = process;
        stages_.push_back(std::move(stage));
    }
    if (stages_.empty()) {
        error = "line has no timed station";
        return false;
    }
    interval_ = stages_.front().process;
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
        if (node->mech.kind == MechKind::Source && node->mech.interval > 1e-6) {
            interval_ = node->mech.interval;
            break;
        }
    }
    running_ = true;
    error.clear();
    return true;
}

void LineProduction::advance(double dt) {
    time_ += dt;
    if (input_) {
        sourceBlocked_ += dt;
    } else {
        sourceClock_ += dt;
    }
    for (Stage& stage : stages_) {
        if (stage.occupied && stage.processing) {
            const double room = std::max(0.0, stage.process - stage.worked);
            const double used = std::min(dt, room);
            stage.worked += used;
            stage.busy += used;
            if (stage.worked >= stage.process - 1e-9) {
                stage.processing = false;
                stage.blocked += dt - used;
            }
        } else if (stage.occupied) {
            stage.blocked += dt;
        } else {
            stage.starved += dt;
        }
    }
}

void LineProduction::transfer() {
    bool moved = true;
    int guard = 0;
    while (moved && guard < 8) {
        moved = false;
        ++guard;
        for (Stage& stage : stages_) {
            if (stage.occupied && stage.processing && stage.worked >= stage.process - 1e-9) {
                stage.processing = false;
            }
        }
        if (!stages_.empty() && stages_.back().occupied && !stages_.back().processing) {
            stages_.back().occupied = false;
            ++stages_.back().completions;
            ++output_;
            moved = true;
        }
        for (int index = static_cast<int>(stages_.size()) - 2; index >= 0; --index) {
            Stage& current = stages_[static_cast<std::size_t>(index)];
            Stage& next = stages_[static_cast<std::size_t>(index + 1)];
            if (current.occupied && !current.processing && !next.occupied) {
                current.occupied = false;
                ++current.completions;
                next.occupied = true;
                next.processing = true;
                next.worked = 0;
                moved = true;
            }
        }
        if (input_ && !stages_.empty() && !stages_.front().occupied) {
            input_ = false;
            stages_.front().occupied = true;
            stages_.front().processing = true;
            stages_.front().worked = 0;
            moved = true;
        }
        if (!input_ && sourceClock_ >= interval_ - 1e-12) {
            input_ = true;
            sourceClock_ = 0;
            moved = true;
        }
    }
}

void LineProduction::step(double dt) {
    if (!running_ || dt <= 0) {
        return;
    }
    double left = dt;
    int guard = 0;
    while (left > 1e-9 && guard < 100000) {
        ++guard;
        transfer();
        double slice = left;
        if (!input_) {
            slice = std::min(slice, std::max(0.0, interval_ - sourceClock_));
        }
        for (const Stage& stage : stages_) {
            if (stage.occupied && stage.processing) {
                slice = std::min(slice, std::max(0.0, stage.process - stage.worked));
            }
        }
        if (slice <= 1e-12) {
            break;
        }
        advance(slice);
        left -= slice;
    }
    transfer();
}

ProductionStats LineProduction::stats() const {
    ProductionStats stats;
    stats.time = time_;
    stats.output = output_;
    stats.jobsPerHour = time_ > 1e-9 ? output_ * 3600.0 / time_ : 0;
    stats.sourceBlockedSeconds = sourceBlocked_;
    stats.stations.reserve(stages_.size());
    for (const Stage& stage : stages_) {
        StationAccount account;
        account.id = stage.id;
        account.busySeconds = stage.busy;
        account.starvedSeconds = stage.starved;
        account.blockedSeconds = stage.blocked;
        account.completions = stage.completions;
        account.processSeconds = stage.process;
        stats.stations.push_back(std::move(account));
    }
    return stats;
}

}  // namespace sim
