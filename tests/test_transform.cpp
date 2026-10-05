#include "sim/core/Math.hpp"
#include "sim/core/Transform.hpp"
#include "sim/core/Units.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;

TEST_CASE("quaternion rotates X toward Y about Z") {
    const sim::Quat rotation = sim::Quat::fromAxisAngle({0, 0, 1}, sim::kPi / 2.0);
    const sim::Vec3 point = rotation.rotate({1, 0, 0});
    REQUIRE(point.x == Approx(0).margin(1e-9));
    REQUIRE(point.y == Approx(1).margin(1e-9));
    REQUIRE(point.z == Approx(0).margin(1e-9));
}

TEST_CASE("transform translation is applied in millimeters") {
    sim::Transform transform;
    transform.translation = {10, 0, 0};
    const sim::Vec3 point = transform.transformPoint({1, 2, 3});
    REQUIRE(point.x == Approx(11));
    REQUIRE(point.y == Approx(2));
    REQUIRE(point.z == Approx(3));
}

TEST_CASE("parent world matrix composes with the local matrix") {
    sim::Transform parent;
    parent.translation = {0, 0, 5};
    sim::Transform local;
    local.translation = {2, 0, 0};
    const sim::Vec3 point = sim::transformPoint(sim::mul(parent.matrix(), local.matrix()), {0, 0, 0});
    REQUIRE(point.x == Approx(2));
    REQUIRE(point.z == Approx(5));
}

TEST_CASE("affine inverse undoes a rigid transform") {
    sim::Transform transform;
    transform.translation = {4, -3, 8};
    transform.rotation = sim::Quat::fromEulerXYZ(sim::deg2rad(20), sim::deg2rad(-15), sim::deg2rad(40));
    const sim::Mat4 matrix = transform.matrix();
    const sim::Mat4 identity = sim::mul(matrix, sim::inverseAffine(matrix));
    REQUIRE(identity(0, 0) == Approx(1).margin(1e-9));
    REQUIRE(identity(1, 1) == Approx(1).margin(1e-9));
    REQUIRE(identity(2, 2) == Approx(1).margin(1e-9));
    REQUIRE(identity(3, 0) == Approx(0).margin(1e-8));
    REQUIRE(identity(3, 1) == Approx(0).margin(1e-8));
    REQUIRE(identity(3, 2) == Approx(0).margin(1e-8));
}

TEST_CASE("euler XYZ round-trips") {
    const double rx = sim::deg2rad(30);
    const double ry = sim::deg2rad(-20);
    const double rz = sim::deg2rad(45);
    const sim::Quat quaternion = sim::Quat::fromEulerXYZ(rx, ry, rz);
    double ox = 0, oy = 0, oz = 0;
    quaternion.toEulerXYZ(ox, oy, oz);
    REQUIRE(ox == Approx(rx).margin(1e-8));
    REQUIRE(oy == Approx(ry).margin(1e-8));
    REQUIRE(oz == Approx(rz).margin(1e-8));
}

TEST_CASE("length units convert to millimeters") {
    REQUIRE(sim::toMillimeters(1.0, sim::LengthUnit::Meter) == Approx(1000));
    REQUIRE(sim::toMillimeters(1.0, sim::LengthUnit::Inch) == Approx(25.4));
    sim::LengthUnit parsed{};
    REQUIRE(sim::parseLengthUnit("mm", parsed));
    REQUIRE(parsed == sim::LengthUnit::Millimeter);
}

TEST_CASE("point to segment distance") {
    REQUIRE(sim::distancePointToSegment2D(0, 1, 0, 0, 10, 0) == Approx(1));
    REQUIRE(sim::distancePointToSegment2D(5, 0, 0, 0, 10, 0) == Approx(0).margin(1e-9));
}
