#pragma once

#include "sim/core/Kinematics.hpp"

#include <memory>
#include <string>

namespace sim {

// Vendor post. KUKA writes .src/.dat. ABB writes a RAPID module plus a data module.
// FANUC writes an ASCII LS program (the controller's text upload). Binary TP is not produced.
class IOlpProgram {
public:
    virtual ~IOlpProgram() = default;
    virtual const char* vendor() const = 0;
    virtual bool exportProgram(const KinematicResource& resource, const RobotPath& path, std::string& src,
                               std::string& dat, std::string& error) const = 0;
    virtual bool importProgram(const KinematicResource* resource, const std::string& src, const std::string& dat,
                               RobotPath& path, std::string& error) const = 0;
};

// "kuka", "abb", or "fanuc". Any other name returns null.
std::unique_ptr<IOlpProgram> createOlpProgram(const std::string& vendor);

}  // namespace sim
