#pragma once

#include <string>
#include <string_view>

namespace sim {

// Scene coordinates are millimeters. Conversions are explicit.
enum class LengthUnit { Millimeter, Centimeter, Meter, Inch, Foot };

double toMillimeters(double value, LengthUnit from);
double fromMillimeters(double millimeters, LengthUnit to);
bool parseLengthUnit(std::string_view text, LengthUnit& out);
const char* lengthUnitName(LengthUnit unit);

}  // namespace sim
