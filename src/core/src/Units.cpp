#include "sim/core/Units.hpp"

namespace sim {

double toMillimeters(double value, LengthUnit from) {
    switch (from) {
        case LengthUnit::Millimeter:
            return value;
        case LengthUnit::Centimeter:
            return value * 10.0;
        case LengthUnit::Meter:
            return value * 1000.0;
        case LengthUnit::Inch:
            return value * 25.4;
        case LengthUnit::Foot:
            return value * 304.8;
    }
    return value;
}

double fromMillimeters(double millimeters, LengthUnit to) {
    switch (to) {
        case LengthUnit::Millimeter:
            return millimeters;
        case LengthUnit::Centimeter:
            return millimeters / 10.0;
        case LengthUnit::Meter:
            return millimeters / 1000.0;
        case LengthUnit::Inch:
            return millimeters / 25.4;
        case LengthUnit::Foot:
            return millimeters / 304.8;
    }
    return millimeters;
}

bool parseLengthUnit(std::string_view text, LengthUnit& out) {
    if (text == "mm" || text == "millimeter" || text == "millimeters") {
        out = LengthUnit::Millimeter;
        return true;
    }
    if (text == "cm") {
        out = LengthUnit::Centimeter;
        return true;
    }
    if (text == "m" || text == "meter" || text == "meters") {
        out = LengthUnit::Meter;
        return true;
    }
    if (text == "in" || text == "inch" || text == "inches") {
        out = LengthUnit::Inch;
        return true;
    }
    if (text == "ft" || text == "foot" || text == "feet") {
        out = LengthUnit::Foot;
        return true;
    }
    return false;
}

const char* lengthUnitName(LengthUnit unit) {
    switch (unit) {
        case LengthUnit::Millimeter:
            return "mm";
        case LengthUnit::Centimeter:
            return "cm";
        case LengthUnit::Meter:
            return "m";
        case LengthUnit::Inch:
            return "in";
        case LengthUnit::Foot:
            return "ft";
    }
    return "mm";
}

}  // namespace sim
