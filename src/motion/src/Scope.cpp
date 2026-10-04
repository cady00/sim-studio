#include "sim/motion/Scope.hpp"

#include "sim/core/Kinematics.hpp"
#include "sim/core/Scene.hpp"
#include "sim/motion/Motion.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace sim {
namespace {

double signalNumber(const Signal& signal) {
    switch (signal.type) {
        case SignalType::Bool:
            return signal.flag ? 1.0 : 0.0;
        case SignalType::Int:
            return static_cast<double>(signal.integer);
        case SignalType::Real:
            return signal.real;
    }
    return 0;
}

double channelValue(const Scene& scene, const std::string& id) {
    if (id.rfind("sig:", 0) == 0) {
        const Signal* signal = scene.findSignal(id.substr(4));
        return signal == nullptr ? 0 : signalNumber(*signal);
    }
    if (id.rfind("jnt:", 0) == 0) {
        const std::string body = id.substr(4);
        const std::size_t slash = body.find('/');
        if (slash == std::string::npos) {
            return 0;
        }
        const KinematicResource* resource = findResource(scene.resources, body.substr(0, slash));
        if (resource == nullptr) {
            return 0;
        }
        const KinJoint* joint = findJoint(*resource, body.substr(slash + 1));
        return joint == nullptr ? 0 : jointCommand(*resource, *joint);
    }
    return 0;
}

std::vector<double> differentiate(const std::vector<double>& time, const std::vector<double>& value) {
    std::vector<double> out(value.size(), 0);
    const std::size_t count = std::min(time.size(), value.size());
    for (std::size_t i = 1; i < count; ++i) {
        const double dt = time[i] - time[i - 1];
        if (std::abs(dt) > 1e-12) {
            out[i] = (value[i] - value[i - 1]) / dt;
        }
    }
    return out;
}

std::string csvCell(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (char letter : text) {
        if (letter == '"') {
            escaped += "\"\"";
        } else {
            escaped += letter;
        }
    }
    escaped += '"';
    return escaped;
}

}  // namespace

void ScopeTrace::clear() {
    time.clear();
    series.clear();
    for (const std::string& id : channels) {
        series.push_back(Series{id, {}});
    }
}

void ScopeTrace::watch(const std::vector<std::string>& ids) {
    channels = ids;
    for (const std::string& id : channels) {
        if (findSeries(id) == nullptr) {
            Series created;
            created.id = id;
            created.value.assign(time.size(), 0);
            series.push_back(std::move(created));
        }
    }
}

const ScopeTrace::Series* ScopeTrace::findSeries(const std::string& id) const {
    for (const Series& item : series) {
        if (item.id == id) {
            return &item;
        }
    }
    return nullptr;
}

void ScopeTrace::sample(const Scene& scene, double timeSeconds) {
    if (!time.empty() && std::abs(time.back() - timeSeconds) < 1e-9) {
        for (const std::string& id : channels) {
            for (Series& item : series) {
                if (item.id == id && !item.value.empty()) {
                    item.value.back() = channelValue(scene, id);
                }
            }
        }
        return;
    }
    time.push_back(timeSeconds);
    for (Series& item : series) {
        const bool watched = std::find(channels.begin(), channels.end(), item.id) != channels.end();
        item.value.push_back(watched ? channelValue(scene, item.id) : (item.value.empty() ? 0 : item.value.back()));
    }
    if (time.size() <= kMaxSamples) {
        return;
    }
    const std::size_t extra = time.size() - kMaxSamples;
    time.erase(time.begin(), time.begin() + static_cast<std::ptrdiff_t>(extra));
    for (Series& item : series) {
        if (item.value.size() >= extra) {
            item.value.erase(item.value.begin(), item.value.begin() + static_cast<std::ptrdiff_t>(extra));
        }
    }
}

double ScopeTrace::valueAt(const std::string& id, double timeSeconds) const {
    const Series* item = findSeries(id);
    if (item == nullptr || item->value.empty() || time.empty()) {
        return 0;
    }
    const std::size_t count = std::min(time.size(), item->value.size());
    std::size_t index = 0;
    for (std::size_t i = 0; i < count; ++i) {
        if (time[i] <= timeSeconds + 1e-12) {
            index = i;
        } else {
            break;
        }
    }
    if (time[index] > timeSeconds + 1e-12) {
        return 0;
    }
    return item->value[index];
}

std::vector<double> ScopeTrace::velocity(const std::string& jointId) const {
    const Series* item = findSeries(jointId);
    if (item == nullptr) {
        return {};
    }
    return differentiate(time, item->value);
}

std::vector<double> ScopeTrace::acceleration(const std::string& jointId) const {
    return differentiate(time, velocity(jointId));
}

std::string ScopeTrace::toCsv(bool velocityColumns, bool accelerationColumns) const {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(6);
    out << "time";
    std::vector<const Series*> columns;
    std::vector<std::string> headers;
    for (const std::string& id : channels) {
        const Series* item = findSeries(id);
        if (item == nullptr) {
            continue;
        }
        columns.push_back(item);
        headers.push_back(id);
        out << ',' << csvCell(id);
        if (velocityColumns && id.rfind("jnt:", 0) == 0) {
            headers.push_back(id + ".v");
            out << ',' << csvCell(id + ".v");
        }
        if (accelerationColumns && id.rfind("jnt:", 0) == 0) {
            headers.push_back(id + ".a");
            out << ',' << csvCell(id + ".a");
        }
    }
    std::vector<std::vector<double>> derived;
    derived.reserve(headers.size());
    std::size_t column = 0;
    for (const std::string& id : channels) {
        const Series* item = findSeries(id);
        if (item == nullptr) {
            continue;
        }
        derived.push_back(item->value);
        ++column;
        if (velocityColumns && id.rfind("jnt:", 0) == 0) {
            derived.push_back(velocity(id));
            ++column;
        }
        if (accelerationColumns && id.rfind("jnt:", 0) == 0) {
            derived.push_back(acceleration(id));
            ++column;
        }
    }
    (void)column;
    (void)headers;
    for (std::size_t row = 0; row < time.size(); ++row) {
        out << '\n' << time[row];
        for (const std::vector<double>& values : derived) {
            const double number = row < values.size() ? values[row] : 0;
            out << ',' << number;
        }
    }
    out << '\n';
    return out.str();
}

}  // namespace sim
