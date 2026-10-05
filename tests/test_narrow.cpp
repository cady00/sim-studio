#include "sim/collision/Likelihood.hpp"
#include "sim/collision/Query.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/motion/WeldPlan.hpp"
#include "sim/physics/ContactNarrow.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

namespace {

sim::MeshData twoTriangles() {
    sim::MeshData mesh;
    auto add = [&](sim::Vec3 a, sim::Vec3 b, sim::Vec3 c) {
        const auto base = static_cast<std::uint32_t>(mesh.vertexCount());
        const sim::Vec3 normal = sim::normalize(sim::cross(b - a, c - a));
        for (sim::Vec3 point : {a, b, c}) {
            mesh.positions.push_back(static_cast<float>(point.x));
            mesh.positions.push_back(static_cast<float>(point.y));
            mesh.positions.push_back(static_cast<float>(point.z));
            mesh.normals.push_back(static_cast<float>(normal.x));
            mesh.normals.push_back(static_cast<float>(normal.y));
            mesh.normals.push_back(static_cast<float>(normal.z));
        }
        mesh.indices.insert(mesh.indices.end(), {base, base + 1, base + 2});
    };
    add({0, 0, 0}, {10, 0, 0}, {0, 10, 0});
    add({100, 0, 0}, {110, 0, 0}, {100, 10, 0});
    mesh.recomputeBounds();
    return mesh;
}

struct CountingModel : sim::ICollisionLikelihood {
    const char* backendName() const override { return "counting"; }
    bool rankLearned(double gapMm, bool previousHit, double& scoreOut, std::string& error) const override {
        ++calls;
        error.clear();
        scoreOut = -gapMm + (previousHit ? 1.0e12 : 0.0);
        return true;
    }
    mutable int calls = 0;
};

bool sameHits(const std::vector<sim::ContactHit>& left, const std::vector<sim::ContactHit>& right) {
    if (left.size() != right.size()) {
        return false;
    }
    for (std::size_t index = 0; index < left.size(); ++index) {
        if (left[index].a != right[index].a || left[index].b != right[index].b || left[index].kind != right[index].kind) {
            return false;
        }
        if (std::abs(left[index].distance - right[index].distance) > 1e-6) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("likelihood score prefers the previous hit and a learned ranker can replace it") {
    REQUIRE(sim::aabbSeparation({0, 0, 0}, {10, 10, 10}, {15, 0, 0}, {25, 10, 10}) == Catch::Approx(5));
    REQUIRE(sim::aabbSeparation({0, 0, 0}, {10, 10, 10}, {8, 0, 0}, {12, 10, 10}) == Catch::Approx(-2));
    sim::DistancePreviousLikelihood heuristic;
    REQUIRE(heuristic.score(5, true) < heuristic.score(0, false));
    REQUIRE(heuristic.backendName() == std::string("distance-and-previous"));
    double learned = 0;
    std::string error = "kept";
    REQUIRE_FALSE(heuristic.rankLearned(5, true, learned, error));
    REQUIRE(error.empty());
}

TEST_CASE("ordered box detection matches the exhaustive manifold and stops on the previous hit") {
    const sim::MeshData mesh = twoTriangles();
    const sim::TriangleBvh bvh = sim::buildTriangleBvh(mesh);
    const sim::Mat4 world = sim::Mat4::identity();
    const sim::Vec3 center{50, 5, 0};
    const sim::Vec3 half{80, 20, 20};
    int exhaustiveTests = 0;
    const std::vector<sim::NarrowContact> full = sim::queryBoxMesh(bvh, world, center, half, 8, &exhaustiveTests);
    REQUIRE_FALSE(full.empty());
    const std::vector<sim::NarrowContact> again = sim::queryBoxMesh(bvh, world, center, half, 8, nullptr);
    REQUIRE(again.size() == full.size());
    REQUIRE(again[0].depth == Catch::Approx(full[0].depth));

    sim::NarrowMemory memory;
    memory.triangle = 1;
    sim::NarrowDetectStats stats;
    REQUIRE(sim::detectBoxMesh(bvh, world, center, half, nullptr, memory, &stats));
    REQUIRE(stats.hit);
    REQUIRE(stats.triangle == 1);
    REQUIRE(stats.tested == 1);
    REQUIRE(exhaustiveTests >= 2);

    CountingModel model;
    sim::NarrowMemory fresh;
    sim::NarrowDetectStats flipped;
    REQUIRE(sim::detectBoxMesh(bvh, world, center, half, &model, fresh, &flipped));
    REQUIRE(model.calls >= 2);
    REQUIRE(flipped.hit);
    REQUIRE(sim::detectBoxMesh(bvh, world, center, half, nullptr, fresh, nullptr) == !full.empty());

    const sim::Vec3 missed{50, 5, 400};
    sim::NarrowMemory missMemory;
    sim::NarrowDetectStats miss;
    REQUIRE_FALSE(sim::detectBoxMesh(bvh, world, missed, half, nullptr, missMemory, &miss));
    REQUIRE(sim::queryBoxMesh(bvh, world, missed, {10, 10, 10}, 4).empty());
    REQUIRE(miss.tested == 0);
}

TEST_CASE("overlap early exit matches the full contact query") {
    sim::Scene scene;
    sim::buildCollisionDemo(scene);
    auto query = sim::createCollisionQuery();
    const std::vector<sim::ContactHit> first = query->query(scene, 15);
    REQUIRE(first.size() == 2);
    REQUIRE(query->overlaps(scene, 15));
    REQUIRE(query->overlaps(scene, 0));
    const std::vector<sim::ContactHit> second = query->query(scene, 15);
    REQUIRE(sameHits(first, second));
    REQUIRE_FALSE(query->overlaps(scene, -100));

    auto ranked = sim::createCollisionQuery();
    auto model = std::make_shared<CountingModel>();
    ranked->setLikelihood(model);
    ranked->rebuild(scene);
    REQUIRE(ranked->overlaps(scene, 15));
    REQUIRE(model->calls > 0);
    const std::vector<sim::ContactHit> untouched = ranked->query(scene, 15);
    REQUIRE(sameHits(first, untouched));

    sim::Scene apart;
    const std::string meshId = apart.addMesh(sim::makeBoxMesh({10, 10, 10}));
    auto left = apart.makeNode(sim::NodeKind::Part, "Left");
    auto right = apart.makeNode(sim::NodeKind::Part, "Right");
    left->meshId = meshId;
    right->meshId = meshId;
    right->local.translation = {100, 0, 0};
    apart.addChild(apart.root(), left);
    apart.addChild(apart.root(), right);
    auto separated = sim::createCollisionQuery();
    REQUIRE(separated->query(apart, 0).empty());
    REQUIRE_FALSE(separated->overlaps(apart, 0));
    REQUIRE(separated->overlaps(apart, 90));
    REQUIRE(separated->query(apart, 90).size() == 1);
}

TEST_CASE("bvh detection and rrt connect report a measured speedup") {
    constexpr int triangles = 1600;
    const auto mesh = sim::makeTriangleGrid(triangles, 400);
    const sim::TriangleBvh bvh = sim::buildTriangleBvh(*mesh);
    const sim::Mat4 world = sim::Mat4::identity();
    const sim::Vec3 center{200, 200, 0};
    const sim::Vec3 half{220, 220, 30};

    int warmTests = 0;
    REQUIRE_FALSE(sim::queryBoxMesh(bvh, world, center, half, 4, &warmTests).empty());
    sim::NarrowMemory warmMemory;
    REQUIRE(sim::detectBoxMesh(bvh, world, center, half, nullptr, warmMemory, nullptr));

    constexpr int repeats = 40;
    const auto timeQuery = [&]() {
        const auto start = std::chrono::steady_clock::now();
        int tests = 0;
        for (int i = 0; i < repeats; ++i) {
            REQUIRE_FALSE(sim::queryBoxMesh(bvh, world, center, half, 4, &tests).empty());
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return std::pair<double, int>{ms, tests};
    };
    const auto timeDetect = [&]() {
        const auto start = std::chrono::steady_clock::now();
        int tests = 0;
        sim::NarrowMemory memory;
        for (int i = 0; i < repeats; ++i) {
            sim::NarrowDetectStats stats;
            REQUIRE(sim::detectBoxMesh(bvh, world, center, half, nullptr, memory, &stats));
            tests += stats.tested;
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return std::pair<double, int>{ms, tests};
    };
    const auto full = timeQuery();
    const auto early = timeDetect();
    const double bvhRatio = full.first / std::max(early.first, 1e-6);
    REQUIRE(full.second > early.second * 10);
    REQUIRE(early.second == repeats);

    const sim::Vec3 missed{200, 200, 800};
    const auto timeMiss = [&](bool detect) {
        const auto start = std::chrono::steady_clock::now();
        sim::NarrowMemory memory;
        for (int i = 0; i < repeats; ++i) {
            if (detect) {
                REQUIRE_FALSE(sim::detectBoxMesh(bvh, world, missed, {20, 20, 20}, nullptr, memory, nullptr));
            } else {
                REQUIRE(sim::queryBoxMesh(bvh, world, missed, {20, 20, 20}, 4).empty());
            }
        }
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    };
    const double missFull = timeMiss(false);
    const double missEarly = timeMiss(true);

    sim::KinematicResource robot;
    robot.name = "arm";
    robot.joints.resize(2);
    robot.joints[0].limited = true;
    robot.joints[0].lower = -90;
    robot.joints[0].upper = 90;
    robot.joints[1].limited = true;
    robot.joints[1].lower = -90;
    robot.joints[1].upper = 90;
    const std::vector<double> start{-80, 0};
    const std::vector<double> goal{80, 0};
    sim::RrtOptions options;
    options.iterations = 250;
    options.stepDeg = 12;
    options.seed = 3;

    int earlyCalls = 0;
    int earlySat = 0;
    int fullCalls = 0;
    int fullSat = 0;
    sim::NarrowMemory earlyMemory;
    auto earlyPredicate = [&](const std::vector<double>& q) {
        ++earlyCalls;
        sim::NarrowDetectStats stats;
        const bool hit = sim::detectBoxMesh(bvh, world, center, half, nullptr, earlyMemory, &stats);
        earlySat += stats.tested;
        return hit && q.size() > 1 && q[0] > 20 && q[0] < 55 && std::abs(q[1]) < 15;
    };
    auto fullPredicate = [&](const std::vector<double>& q) {
        ++fullCalls;
        int sat = 0;
        const bool hit = !sim::queryBoxMesh(bvh, world, center, half, 1, &sat).empty();
        fullSat += sat;
        return hit && q.size() > 1 && q[0] > 20 && q[0] < 55 && std::abs(q[1]) < 15;
    };

    const auto timePlan = [&](const auto& predicate) {
        const auto startTime = std::chrono::steady_clock::now();
        const sim::RrtResult planned = sim::planRrtConnect(robot, start, goal, predicate, options);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - startTime).count();
        return std::pair<sim::RrtResult, double>{planned, ms};
    };
    const auto earlyPlan = timePlan(earlyPredicate);
    const auto fullPlan = timePlan(fullPredicate);
    REQUIRE(earlyPlan.first.ok);
    REQUIRE(fullPlan.first.ok);
    REQUIRE(earlyPlan.first.joints.size() == fullPlan.first.joints.size());
    for (std::size_t index = 0; index < earlyPlan.first.joints.size(); ++index) {
        REQUIRE(earlyPlan.first.joints[index].size() == fullPlan.first.joints[index].size());
        for (std::size_t joint = 0; joint < earlyPlan.first.joints[index].size(); ++joint) {
            REQUIRE(earlyPlan.first.joints[index][joint] == Catch::Approx(fullPlan.first.joints[index][joint]).margin(1e-9));
        }
    }
    const double rrtRatio = fullPlan.second / std::max(earlyPlan.second, 1e-6);
    std::cout << "NARROW_SPEEDUP bvh_hit=" << bvhRatio << " bvh_hit_ms=" << full.first << "/" << early.first
              << " bvh_sat=" << full.second << "/" << early.second << " bvh_miss=" << (missFull / std::max(missEarly, 1e-6))
              << " rrt=" << rrtRatio << " rrt_ms=" << fullPlan.second << "/" << earlyPlan.second << " rrt_sat=" << fullSat << "/"
              << earlySat << " rrt_calls=" << fullCalls << "/" << earlyCalls << "\n";
    REQUIRE(rrtRatio > 1.5);
    REQUIRE(bvhRatio > 1.5);
}
