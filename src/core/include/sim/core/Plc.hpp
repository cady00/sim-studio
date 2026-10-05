#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

// Process image of the built-in soft PLC. Addresses are names such as I0.0, Q0.0, M0.0, T0, D0.
enum class PlcValueKind { Bool, Int, Real, Timer, Counter };

struct PlcTag {
    std::string address;
    PlcValueKind kind = PlcValueKind::Bool;
    bool flag = false;
    std::int64_t integer = 0;
    double real = 0;
    // Timer preset and elapsed time, seconds. done is the timer coil.
    double preset = 0;
    double elapsed = 0;
    bool done = false;
    bool forced = false;
    bool forceFlag = false;
    std::int64_t forceInteger = 0;
    double forceReal = 0;
};

// Device signal or OPC UA numeric node, bound to one PLC address.
struct PlcBinding {
    std::string device;
    std::string address;
    // in: device or OPC into the PLC. out: PLC into the device or OPC.
    std::string direction = "in";
    std::string opcNode;
};

struct PlcContact {
    std::string address;
    bool normallyOpen = true;
    // empty is a level contact. rising is false-to-true since the previous scan.
    // falling is true-to-false. An edge ignores normallyOpen.
    std::string edge;
    // Contacts ORed with this one inside the AND branch. One level only.
    std::vector<PlcContact> alternatives;
};

// One rung. Contacts in one branch are AND. A contact may OR its alternatives.
// series is the first branch. parallel holds the other branches.
// With no parallel branches, an empty series is always true.
// action: out, set, reset, ton, tof, move, ctu, ctd, ctr.
// move writes literal into an int or real tag.
// ctu counts up on the rising edge of the rung and is done at or above the preset.
// ctd counts down on the rising edge and is done at or below zero.
// ctr clears the count while the rung is true and then applies the ctu done test.
// ton is an on-delay. tof is an off-delay: the coil stays done after power drops
// until the preset elapses. A timer that has not yet seen power stays off.
struct PlcRung {
    std::vector<PlcContact> series;
    std::vector<std::vector<PlcContact>> parallel;
    std::string coil;
    std::string action = "out";
    double literal = 0;
};

// Sphere watched by the soft PLC. A listed node origin or robot TCP inside the
// radius writes a boolean input. An empty address becomes Name_Occupied, or
// Z1_Occupied when the name is empty too. An empty watch list uses every robot TCP.
struct PlcZoneWatch {
    std::string name;
    std::string address;
    Vec3 center{0, 0, 0};
    double radius = 150;
    std::vector<std::string> watch;
};

// Mesh collision of one kinematic resource, written as a boolean input.
// An empty address becomes R1_Collision, R2_Collision, ... in list order.
// An empty resource name tests every resource.
struct PlcCollisionWatch {
    std::string address;
    std::string resource;
};

struct SoftPlcDocument {
    bool enabled = false;
    double scanSeconds = 0.01;
    // softplc | plcsim. plcsim is reserved and does not scan.
    std::string backend = "softplc";
    std::vector<PlcTag> tags;
    std::vector<PlcBinding> bindings;
    std::vector<PlcRung> rungs;
    std::vector<PlcZoneWatch> zones;
    std::vector<PlcCollisionWatch> collisions;
};

const char* plcValueKindName(PlcValueKind kind);
bool plcValueKindFromName(const std::string& name, PlcValueKind& out);

PlcTag* findPlcTag(SoftPlcDocument& doc, const std::string& address);
const PlcTag* findPlcTag(const SoftPlcDocument& doc, const std::string& address);

nlohmann::json softPlcToJson(const SoftPlcDocument& doc);
bool softPlcFromJson(const nlohmann::json& json, SoftPlcDocument& doc, std::string& error);

}  // namespace sim
