#pragma once

#include "sim/core/Math.hpp"

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <vector>

namespace sim {

class Scene;

// One weld. Position and normal are millimeters in the world frame.
// `normal` points out of the sheet, toward the gun. The gun TCP +Z points
// the opposite way, into the sheet. `layers` are the sheet part names.
// `electrode` is the gun or electrode-group name. `example` marks authored sample data.
struct WeldSpot {
    std::string id;
    std::string name;
    Vec3 position{0, 0, 0};
    Vec3 normal{0, 0, 1};
    std::vector<std::string> layers;
    std::string electrode;
    std::string part;
    bool onSurface = false;
    bool example = false;
    bool checked = false;
    bool reachable = false;
    double margin = 0;
    double spinDeg = 0;
    double manipulability = 0;
    bool singular = false;
    bool collision = false;
};

// CSV columns: id,x,y,z,nx,ny,nz,layers,electrode,example.
// A header row is optional. Layers are separated by '|'.
bool importWeldCsv(const std::string& text, std::vector<WeldSpot>& spots, std::string& error);
// <welds><spot id="" electrode="" example="1"><point x y z/><normal x y z/><layer>name</layer></spot></welds>
bool importWeldXml(const std::string& text, std::vector<WeldSpot>& spots, std::string& error);

nlohmann::json weldsToJson(const std::vector<WeldSpot>& spots);
void weldsFromJson(const nlohmann::json& json, std::vector<WeldSpot>& spots);

// Ray from outside the sheet along -normal. The hit replaces position and normal.
// A spot with an empty part uses the first layer that owns a mesh.
int projectWeldSpots(Scene& scene);

}  // namespace sim
