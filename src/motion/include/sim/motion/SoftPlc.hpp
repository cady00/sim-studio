#pragma once

#include "sim/core/Plc.hpp"
#include "sim/core/Scene.hpp"

#include <functional>
#include <string>
#include <vector>

namespace sim {

// Optional OPC UA exchange. write is false to read into value, true to publish value.
// Returns false when that node is not connected. The runtime logs the miss once per node.
using PlcOpcExchange = std::function<bool(const std::string& nodeId, bool write, bool& value, std::string& error)>;

// One PLC execution backend. The built-in soft PLC is the default.
// PLCSIM Advanced is a later adapter and is not implemented here.
class IPlcBackend {
public:
    virtual ~IPlcBackend() = default;
    virtual const char* name() const = 0;
    virtual bool available() const = 0;
    virtual bool start(const SoftPlcDocument& doc, std::string& error) = 0;
    virtual void stop() = 0;
    virtual void scan(Scene& scene, double simTime, double dt) = 0;
    virtual const std::vector<std::string>& log() const = 0;
};

// Clears timer elapsed time. The program, forces, and mappings stay.
void resetSoftPlcTimers(SoftPlcDocument& doc);

// Scan log as CSV. Columns are time, scan, signal, value. Lines that are not
// scan records are skipped. One row per signal named on that scan.
std::string softPlcScanCsv(const std::vector<std::string>& log);

// Unwired boolean drafts for an imported mechanism. An arm gets
// Rn_AtHome, Rn_Busy, Rn_Fault (in) and Rn_Start, Rn_Grip (out).
// Each external axis gets Name_InPos (in). Device strings stay empty
// and nothing is forced. A track does not take a robot number.
void appendRobotPlcDraft(SoftPlcDocument& doc, const KinematicResource& resource);

// Scans the ladder on the simulation clock. Sensors written during the physics
// step are visible on the next scan. At most eight scans run in one step.
class SoftPlcRuntime final : public IPlcBackend {
public:
    void setOpcExchange(PlcOpcExchange exchange);

    const char* name() const override { return "softplc"; }
    bool available() const override { return true; }
    bool start(const SoftPlcDocument& doc, std::string& error) override;
    void stop() override;
    void scan(Scene& scene, double simTime, double dt) override;
    const std::vector<std::string>& log() const override { return log_; }
    int scans() const { return scans_; }

private:
    struct Image {
        std::string address;
        bool flag = false;
        std::int64_t integer = 0;
        double real = 0;
        bool done = false;
    };

    void oneScan(Scene& scene, double simTime, double period);
    void note(const std::string& line);
    bool readOpc(const std::string& node, bool& value);
    bool writeOpc(const std::string& node, bool value);
    const Image* previous(const std::string& address) const;

    PlcOpcExchange exchange_;
    double acc_ = 0;
    int scans_ = 0;
    bool running_ = false;
    std::vector<std::string> log_;
    std::vector<std::string> opcWarned_;
    std::vector<Image> previous_;
    // Rung power from the previous scan, so CTU sees one rising edge per pulse.
    std::vector<unsigned char> rungPower_;
    // Mesh-collision latch for this physics step. Zones are tested every scan.
    std::vector<unsigned char> collisionLatch_;
};

// Reserved adapter. available() is false and start() fails. scan() does nothing
// and this type does not open a socket.
class PlcSimAdvancedBackend final : public IPlcBackend {
public:
    const char* name() const override { return "plcsim"; }
    bool available() const override { return false; }
    bool start(const SoftPlcDocument& doc, std::string& error) override;
    void stop() override;
    void scan(Scene& scene, double simTime, double dt) override;
    const std::vector<std::string>& log() const override { return log_; }

private:
    std::vector<std::string> log_;
};

}  // namespace sim
