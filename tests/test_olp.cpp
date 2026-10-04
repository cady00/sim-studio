#include "sim/motion/Motion.hpp"
#include "sim/motion/Olp.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

sim::KinematicResource armWithExternals() {
    sim::KinematicResource resource;
    resource.name = "kr420";
    resource.ik = "spherical_wrist";
    for (int i = 0; i < 6; ++i) {
        sim::KinJoint joint;
        joint.name = "A" + std::to_string(i + 1);
        resource.joints.push_back(joint);
    }
    const char* names[] = {"E1", "E2", "E6"};
    const char* joints[] = {"track", "tilt", "spin"};
    for (int i = 0; i < 3; ++i) {
        sim::KinJoint joint;
        joint.name = joints[i];
        joint.externalName = names[i];
        resource.joints.push_back(joint);
    }
    return resource;
}

sim::RobotPath samplePath() {
    sim::RobotPath path;
    path.name = "weld_tour";
    path.resource = "kr420";
    sim::PathPoint ptp;
    ptp.name = "p1";
    ptp.move = sim::PathMove::Ptp;
    ptp.joints = {10, -20, 30, 40, -50, 60, 12.5, -3, 7.25};
    ptp.speed = 40;
    sim::PathPoint lin;
    lin.name = "p2";
    lin.move = sim::PathMove::Lin;
    lin.cartesian = true;
    lin.position = {100, -20, 350};
    lin.orientation = sim::Quat::fromEulerXYZ(sim::deg2rad(10), sim::deg2rad(-20), sim::deg2rad(30));
    lin.joints = ptp.joints;
    lin.externalValues = {12.5, -3, 7.25};
    lin.speed = 80;
    path.points = {ptp, lin};
    return path;
}

void expectPoint(const sim::PathPoint& point, bool linear) {
    if (!linear) {
        REQUIRE(point.move == sim::PathMove::Ptp);
        REQUIRE(point.joints.size() == 9);
        REQUIRE(point.joints[0] == Catch::Approx(10));
        REQUIRE(point.joints[1] == Catch::Approx(-20));
        REQUIRE(point.joints[5] == Catch::Approx(60));
        REQUIRE(point.speed == Catch::Approx(40));
    } else {
        REQUIRE(point.move == sim::PathMove::Lin);
        REQUIRE(point.cartesian);
        REQUIRE(point.position.x == Catch::Approx(100));
        REQUIRE(point.position.y == Catch::Approx(-20));
        REQUIRE(point.position.z == Catch::Approx(350));
        double rx = 0;
        double ry = 0;
        double rz = 0;
        point.orientation.toEulerXYZ(rx, ry, rz);
        REQUIRE(sim::rad2deg(rx) == Catch::Approx(10).margin(0.2));
        REQUIRE(sim::rad2deg(ry) == Catch::Approx(-20).margin(0.2));
        REQUIRE(sim::rad2deg(rz) == Catch::Approx(30).margin(0.2));
        REQUIRE(point.speed == Catch::Approx(80));
    }
    REQUIRE(point.externalValues.size() == 3);
    REQUIRE(point.externalValues[0] == Catch::Approx(12.5));
    REQUIRE(point.externalValues[1] == Catch::Approx(-3));
    REQUIRE(point.externalValues[2] == Catch::Approx(7.25));
    REQUIRE(point.joints.size() == 9);
    REQUIRE(point.joints[6] == Catch::Approx(12.5));
    REQUIRE(point.joints[7] == Catch::Approx(-3));
    REQUIRE(point.joints[8] == Catch::Approx(7.25));
}

}  // namespace

TEST_CASE("kuka abb and fanuc posts round trip a path with external axes") {
    const sim::KinematicResource robot = armWithExternals();
    const sim::RobotPath path = samplePath();
    REQUIRE(sim::createOlpProgram("other") == nullptr);
    for (const char* vendor : {"kuka", "abb", "fanuc"}) {
        const std::unique_ptr<sim::IOlpProgram> post = sim::createOlpProgram(vendor);
        REQUIRE(post != nullptr);
        REQUIRE(std::string(post->vendor()) == vendor);
        std::string src;
        std::string dat;
        std::string error;
        REQUIRE(post->exportProgram(robot, path, src, dat, error));
        sim::RobotPath back;
        REQUIRE(post->importProgram(&robot, src, dat, back, error));
        REQUIRE(back.points.size() == 2);
        expectPoint(back.points[0], false);
        expectPoint(back.points[1], true);
        if (std::string(vendor) == "kuka") {
            REQUIRE(src.find("PTP") != std::string::npos);
            REQUIRE(dat.find("E6AXIS") != std::string::npos);
            REQUIRE(dat.find("E6 ") != std::string::npos);
            REQUIRE(dat.find("E1 ") != std::string::npos);
            std::string edited = dat;
            const std::string needle = "E1 12.500";
            const auto at = edited.find(needle);
            REQUIRE(at != std::string::npos);
            edited.replace(at, needle.size(), "E1 99.000");
            sim::RobotPath fromDat;
            REQUIRE(post->importProgram(&robot, src, edited, fromDat, error));
            REQUIRE(fromDat.points[0].joints[6] == Catch::Approx(99));
        }
        if (std::string(vendor) == "abb") {
            REQUIRE(src.find("MoveJ") != std::string::npos);
            REQUIRE(dat.find("robtarget") != std::string::npos);
            REQUIRE(dat.find("9E9") != std::string::npos);
        }
        if (std::string(vendor) == "fanuc") {
            REQUIRE(src.find("/PROG") != std::string::npos);
            REQUIRE(src.find("GP2") != std::string::npos);
            REQUIRE(src.find("ASCII LS") != std::string::npos);
        }
    }
}

TEST_CASE("an inline kuka aggregate imports without a dat file") {
    const sim::KinematicResource robot = armWithExternals();
    const char* src = "DEF inline()\n"
                      "PTP {A1 1.000,A2 2.000,A3 3.000,A4 4.000,A5 5.000,A6 6.000,E1 7.000,E2 0.000,E3 0.000,E4 0.000,E5 0.000,E6 8.000}\n"
                      "END\n";
    const std::unique_ptr<sim::IOlpProgram> post = sim::createOlpProgram("kuka");
    sim::RobotPath path;
    std::string error;
    REQUIRE(post->importProgram(&robot, src, "", path, error));
    REQUIRE(path.points.size() == 1);
    REQUIRE(path.points[0].joints[0] == Catch::Approx(1));
    REQUIRE(path.points[0].joints[5] == Catch::Approx(6));
    REQUIRE(path.points[0].joints[6] == Catch::Approx(7));
    REQUIRE(path.points[0].joints[8] == Catch::Approx(8));
}

TEST_CASE("kuka src keeps tool base spline interrupt and outputs") {
    const sim::KinematicResource robot = armWithExternals();
    sim::RobotPath path = samplePath();
    path.frames.push_back({"tool", {0, 0, 150}, 0, 90, 0});
    path.frames.push_back({"base", {10, 0, 0}, 0, 0, 0});
    path.directives.push_back({"interrupt", "INTERRUPT DECL 1 WHEN $IN[1]==TRUE DO BRAKE"});
    path.directives.push_back({"interrupt", "INTERRUPT ON 1"});
    path.directives.push_back({"io", "$OUT[2]=TRUE"});
    path.points[0].shape = "spl";
    path.points[1].shape = "slin";
    const std::unique_ptr<sim::IOlpProgram> post = sim::createOlpProgram("kuka");
    std::string src;
    std::string dat;
    std::string error;
    REQUIRE(post->exportProgram(robot, path, src, dat, error));
    REQUIRE(src.find("SPLINE") != std::string::npos);
    REQUIRE(src.find("SLIN") != std::string::npos);
    REQUIRE(src.find("ENDSPLINE") != std::string::npos);
    REQUIRE(src.find("$TOOL=") != std::string::npos);
    REQUIRE(src.find("$OUT[2]=TRUE") != std::string::npos);
    REQUIRE(src.find("INTERRUPT ON 1") != std::string::npos);
    sim::RobotPath back;
    REQUIRE(post->importProgram(&robot, src, dat, back, error));
    REQUIRE(back.points.size() == 2);
    REQUIRE(back.points[0].shape == "spl");
    REQUIRE(back.points[1].shape == "slin");
    REQUIRE(back.frames.size() == 2);
    REQUIRE(back.frames[0].kind == "tool");
    REQUIRE(back.frames[0].origin.z == Catch::Approx(150));
    REQUIRE(back.frames[0].b == Catch::Approx(90));
    REQUIRE(back.frames[1].kind == "base");
    REQUIRE(back.frames[1].origin.x == Catch::Approx(10));
    REQUIRE(back.directives.size() == 3);
    REQUIRE(back.directives[0].kind == "interrupt");
    REQUIRE(back.directives[2].text == "$OUT[2]=TRUE");
    expectPoint(back.points[0], false);
    expectPoint(back.points[1], true);
}
