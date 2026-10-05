#include "sim/ai/Commands.hpp"
#include "sim/collision/Query.hpp"
#include "sim/core/Mesh.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>

namespace {

sim::ConvexShape sphere(sim::Vec3 center, double radius) {
    sim::ConvexShape shape;
    shape.kind = sim::ConvexShape::Kind::Sphere;
    shape.center = center;
    shape.radius = radius;
    return shape;
}

sim::ConvexShape box(sim::Vec3 center, sim::Vec3 half, sim::Quat rotation = sim::Quat::identity()) {
    sim::ConvexShape shape;
    shape.kind = sim::ConvexShape::Kind::Box;
    shape.center = center;
    shape.half = half;
    shape.rotation = rotation;
    return shape;
}

sim::MeshData makeLMesh() {
    sim::MeshData mesh;
    const sim::Vec3 footprint[] = {{0, 0, 0}, {40, 0, 0}, {40, 20, 0}, {20, 20, 0}, {20, 40, 0}, {0, 40, 0}};
    auto add = [&](sim::Vec3 a, sim::Vec3 b, sim::Vec3 c) {
        const sim::Vec3 normal = sim::normalize(sim::cross(b - a, c - a));
        const auto base = static_cast<std::uint32_t>(mesh.vertexCount());
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
    constexpr int n = 6;
    for (int i = 1; i < n - 1; ++i) {
        add(footprint[0] + sim::Vec3{0, 0, 20}, footprint[i] + sim::Vec3{0, 0, 20}, footprint[i + 1] + sim::Vec3{0, 0, 20});
        add(footprint[0], footprint[i + 1], footprint[i]);
    }
    for (int i = 0; i < n; ++i) {
        const sim::Vec3 a = footprint[i];
        const sim::Vec3 b = footprint[(i + 1) % n];
        add(a, b, b + sim::Vec3{0, 0, 20});
        add(a, b + sim::Vec3{0, 0, 20}, a + sim::Vec3{0, 0, 20});
    }
    mesh.recomputeBounds();
    return mesh;
}

}  // namespace

TEST_CASE("gjk matches analytic sphere and box distances") {
    const sim::ContactHit separated = sim::queryConvex(sphere({0, 0, 0}, 10), sphere({30, 0, 0}, 10));
    REQUIRE(separated.distance == Catch::Approx(10).margin(1e-3));
    REQUIRE(separated.penetration == 0);
    REQUIRE(std::abs(separated.normal.x) > 0.9);

    const sim::ContactHit sphereBox = sim::queryConvex(box({0, 0, 0}, {10, 10, 10}), sphere({20, 0, 0}, 5));
    REQUIRE(sphereBox.distance == Catch::Approx(5).margin(1e-2));

    const sim::ContactHit boxes = sim::queryConvex(box({0, 0, 0}, {10, 10, 10}), box({30, 0, 0}, {10, 10, 10}));
    REQUIRE(boxes.distance == Catch::Approx(10).margin(1e-2));

    const sim::Quat turn = sim::Quat::fromAxisAngle({0, 0, 1}, sim::kPi / 2);
    const sim::ContactHit turned = sim::queryConvex(box({0, 0, 0}, {10, 2, 2}, turn), sphere({0, 20, 0}, 5));
    REQUIRE(turned.distance == Catch::Approx(5).margin(0.05));
}

TEST_CASE("epa reports a known sphere and box penetration") {
    const sim::ContactHit spheres = sim::queryConvex(sphere({0, 0, 0}, 10), sphere({6, 0, 0}, 10));
    REQUIRE(spheres.penetration == Catch::Approx(14).margin(0.05));
    REQUIRE(spheres.distance < 0);
    REQUIRE_FALSE(spheres.volumeEstimated);
    // Centers 6 mm apart, radii 10. Each cap has height 7, so the intersection is 2254 π / 3.
    REQUIRE(spheres.volume == Catch::Approx(2254.0 * sim::kPi / 3.0).margin(1.0));

    const sim::ContactHit shallow = sim::queryConvex(sphere({0, 0, 0}, 10), sphere({19, 0, 0}, 10));
    REQUIRE(shallow.penetration == Catch::Approx(1).margin(0.05));

    const sim::ContactHit boxes = sim::queryConvex(box({0, 0, 0}, {10, 10, 10}), box({15, 0, 0}, {10, 10, 10}));
    REQUIRE(boxes.penetration == Catch::Approx(5).margin(0.15));
    REQUIRE(boxes.volumeEstimated);
    REQUIRE(boxes.volume == Catch::Approx(2000.0).margin(1e-6));
}

TEST_CASE("sweep catches a thin wall that endpoint samples miss") {
    sim::ConvexShape moving = sphere({-40, 0, 0}, 10);
    const sim::ConvexShape wall = box({0, 0, 0}, {0.2, 30, 30});
    bool sampled = false;
    for (double t : {0.0, 0.25, 0.75, 1.0}) {
        moving.center = {-40 + 80 * t, 0, 0};
        if (sim::queryConvex(moving, wall).distance <= 0) {
            sampled = true;
        }
    }
    REQUIRE_FALSE(sampled);
    moving.center = {-40, 0, 0};
    const sim::SweepHit hit = sim::sweepConvex(moving, {40, 0, 0}, wall);
    REQUIRE(hit.hit);
    REQUIRE(hit.time == Catch::Approx(29.8 / 80.0).margin(0.02));
}

TEST_CASE("concave mesh decomposes into more than one hull") {
    const sim::MeshData mesh = makeLMesh();
    const auto pieces = sim::decomposeMesh(mesh);
    REQUIRE(pieces.size() >= 2);
}

TEST_CASE("narrow band sdf gradient points out of a sphere") {
    std::vector<sim::ConvexShape> pieces{sphere({0, 0, 0}, 10)};
    const sim::SdfGrid grid = sim::buildSdf(pieces, 16);
    REQUIRE(grid.nx == 16);
    REQUIRE(grid.ny == 16);
    REQUIRE(grid.nz == 16);
    REQUIRE(grid.values.size() == 4096);
    REQUIRE(grid.cell == Catch::Approx(1.625).margin(1e-12));
    REQUIRE(grid.band == Catch::Approx(4.875).margin(1e-12));
    const int ix = std::clamp(static_cast<int>(std::lround((12.0 - grid.origin.x) / grid.cell - 0.5)), 0, grid.nx - 1);
    const int iy = std::clamp(static_cast<int>(std::lround((0.0 - grid.origin.y) / grid.cell - 0.5)), 0, grid.ny - 1);
    const int iz = std::clamp(static_cast<int>(std::lround((0.0 - grid.origin.z) / grid.cell - 0.5)), 0, grid.nz - 1);
    const sim::Vec3 point = grid.origin + sim::Vec3{(ix + 0.5) * grid.cell, (iy + 0.5) * grid.cell, (iz + 0.5) * grid.cell};
    const double truth = sim::length(point) - 10.0;
    REQUIRE(truth == Catch::Approx(2.241546).margin(1e-4));
    REQUIRE(truth < grid.band);
    double distance = 0;
    sim::Vec3 gradient;
    REQUIRE(sim::sampleSdf(grid, point, distance, gradient));
    REQUIRE(distance == Catch::Approx(truth).margin(0.05));
    REQUIRE(gradient.x > 0.5);
    REQUIRE(sim::dot(gradient, sim::normalize(point)) > 0.7);
}

TEST_CASE("robot sweep reports a thin obstacle between two samples") {
    sim::Scene scene;
    auto& resource = sim::ensureResource(scene, "slide");
    sim::addLink(scene, resource, "base", "", {});
    const std::string moverId = sim::addLink(scene, resource, "mover", "base", {}).nodeId;
    auto mover = scene.find(moverId);
    auto body = scene.makeNode(sim::NodeKind::Part, "Slider");
    body->meshId = scene.addMesh(sim::makeBoxMesh({10, 10, 10}));
    scene.addChild(mover, body);
    auto wall = scene.makeNode(sim::NodeKind::Part, "Wall");
    wall->meshId = scene.addMesh(sim::makeBoxMesh({0.2, 30, 30}));
    scene.addChild(scene.root(), wall);
    sim::KinJoint& joint = sim::addJoint(resource, scene, "travel", "base", "mover", sim::JointMotion::Prismatic, {1, 0, 0}, {0, 0, 0});
    joint.lower = -80;
    joint.upper = 80;
    sim::setJointVector(resource, {-40});
    sim::applyResource(scene, resource);
    REQUIRE(sim::collidingPairs(scene, resource).empty());
    sim::setJointVector(resource, {40});
    sim::applyResource(scene, resource);
    REQUIRE(sim::collidingPairs(scene, resource).empty());
    const sim::PathSweepHit hit = sim::sweepPlannedSamples(scene, resource, {{-40}, {40}});
    REQUIRE(hit.hit);
    REQUIRE(hit.time == Catch::Approx((29.8 / 80.0) / 60.0).margin(0.001));
}

TEST_CASE("collision commands and python see distance and clearance") {
    sim::Scene scene;
    sim::buildCollisionDemo(scene);
    sim::CommandExecutor commands(scene, nullptr, nullptr);
    const auto query = commands.execute({{"op", "collision_query"}});
    REQUIRE(query.ok);
    REQUIRE(query.value.is_array());
    REQUIRE(query.value.size() == 2);
    const auto distance = commands.execute({{"op", "min_distance"}, {"name", "ClearA"}, {"other", "ClearB"}});
    REQUIRE(distance.ok);
    const double gap = distance.value.at("distance").get<double>();
    REQUIRE(gap == Catch::Approx(8).margin(0.05));

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string error;
    REQUIRE(host->execute("d = simstudio.min_distance('OverlapA', 'OverlapB')\nprint(d['distance'])\n", output, error));
    REQUIRE(error.empty());
    REQUIRE(std::stod(output) < 0);
}

TEST_CASE("proximity setup round trips in a project") {
    sim::Scene scene;
    sim::buildCollisionDemo(scene);
    auto query = sim::createCollisionQuery();
    query->rebuild(scene);
    query->commitCache(scene);
    sim::ProjectStore store;
    sim::ProjectResult saved = store.save(scene, "/tmp/collision-roundtrip.simproj");
    REQUIRE(saved.ok);
    sim::Scene loaded;
    sim::ProjectResult opened = store.load("/tmp/collision-roundtrip.simproj", loaded);
    REQUIRE(opened.ok);
    REQUIRE(loaded.proximity.clearanceMm == Catch::Approx(15));
    REQUIRE(loaded.proximity.sets.size() == 2);
    REQUIRE(loaded.proximity.pairs.size() == 1);
    std::remove("/tmp/collision-roundtrip.simproj");
}

TEST_CASE("broadphase benchmark on a few thousand boxes") {
    constexpr int count = 3200;
    sim::Scene scene;
    scene.beginEdit();
    const int nx = 40;
    const int ny = 20;
    const std::string meshId = scene.addMesh(sim::makeBoxMesh({10, 10, 10}));
    for (int i = 0; i < count; ++i) {
        const int x = i % nx;
        const int y = (i / nx) % ny;
        const int z = i / (nx * ny);
        auto node = scene.makeNode(sim::NodeKind::Part, "P" + std::to_string(i));
        node->meshId = meshId;
        node->local.translation = {x * 40.0, y * 40.0, z * 40.0};
        scene.addChild(scene.root(), node);
    }
    scene.endEdit();
    auto query = sim::createCollisionQuery();
    const auto buildStart = std::chrono::steady_clock::now();
    query->rebuild(scene);
    const double buildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - buildStart).count();
    const auto queryStart = std::chrono::steady_clock::now();
    const auto hits = query->query(scene, 0);
    const double queryMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - queryStart).count();
    const auto distanceStart = std::chrono::steady_clock::now();
    double neighbor = 0;
    for (int i = 0; i < 200; ++i) {
        const auto distance = query->minDistance(scene, "P" + std::to_string(i), "P" + std::to_string(i + 1));
        neighbor += distance.distance;
    }
    const double distanceMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - distanceStart).count();
    std::cout << "BENCH parts=" << count << " build_ms=" << buildMs << " overlap_ms=" << queryMs
              << " overlaps=" << hits.size() << " neighbor200_ms=" << distanceMs << " neighbor_sum=" << neighbor << "\n";
    REQUIRE(hits.empty());
    REQUIRE(buildMs < 100);
    REQUIRE(queryMs < 50);
    // 195 face-adjacent gaps of 20 mm, plus 5 row wraps at sqrt(1540² + 20²).
    const double wrap = std::sqrt(1540.0 * 1540.0 + 20.0 * 20.0);
    REQUIRE(neighbor == Catch::Approx(195.0 * 20.0 + 5.0 * wrap).margin(0.05));
}
