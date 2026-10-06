#include "sim/core/Scene.hpp"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

double rssKb() {
    std::ifstream in("/proc/self/status");
    std::string key;
    while (in >> key) {
        if (key == "VmRSS:") {
            double value = 0;
            in >> value;
            return value;
        }
        std::string rest;
        std::getline(in, rest);
    }
    return 0;
}

struct Sample {
    double createMs = 0;
    double walkMs = 0;
    double rss = 0;
};

double percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0;
    }
    std::sort(values.begin(), values.end());
    const double rank = p * static_cast<double>(values.size() - 1);
    const auto low = static_cast<std::size_t>(rank);
    const auto high = std::min(low + 1, values.size() - 1);
    const double frac = rank - static_cast<double>(low);
    return values[low] * (1.0 - frac) + values[high] * frac;
}

void printRow(const char* bench, int run, const char* metric, double value, const char* unit) {
    std::cout << "G0_BENCH," << bench << "," << run << "," << metric << "," << value << "," << unit
              << ",synthetic,linux,cpu,no-window,llvmpipe-not-used\n";
}

}  // namespace

TEST_CASE("G0 synthetic B01 flat instances and B05 nested walk", "[g0][bench]") {
    constexpr int kAssets = 100;
    constexpr int kInstances = 100000;
    constexpr int kDepth = 128;
    constexpr int kRuns = 3;
    std::vector<Sample> flat(kRuns);
    std::vector<Sample> nested(kRuns);
    for (int run = 0; run < kRuns; ++run) {
        sim::Scene scene;
        std::vector<std::string> meshes;
        meshes.reserve(kAssets);
        for (int asset = 0; asset < kAssets; ++asset) {
            auto mesh = std::make_shared<sim::MeshData>();
            const float x = static_cast<float>(asset);
            mesh->positions = {x, 0, 0, x + 1, 0, 0, x, 1, 0};
            mesh->normals = {0, 0, 1, 0, 0, 1, 0, 0, 1};
            mesh->indices = {0, 1, 2};
            mesh->recomputeBounds();
            meshes.push_back(scene.addMesh(mesh));
        }
        scene.beginEdit();
        const auto started = std::chrono::steady_clock::now();
        auto root = scene.root();
        for (int i = 0; i < kInstances; ++i) {
            auto node = scene.makeNode(sim::NodeKind::Part, "inst");
            node->meshId = meshes[static_cast<std::size_t>(i % kAssets)];
            node->local.translation = {static_cast<double>(i % 1000), static_cast<double>(i / 1000), 0};
            scene.addChild(root, node);
        }
        scene.endEdit();
        flat[run].createMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        const auto walkStart = std::chrono::steady_clock::now();
        double checksum = 0;
        for (const auto& child : root->children) {
            checksum += child->worldMatrix()(3, 0);
        }
        flat[run].walkMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - walkStart).count();
        flat[run].rss = rssKb();
        printRow("B01", run, "create_ms", flat[run].createMs, "ms");
        printRow("B01", run, "world_walk_ms", flat[run].walkMs, "ms");
        printRow("B01", run, "rss_kb", flat[run].rss, "kb");
        printRow("B01", run, "instances", kInstances, "count");
        printRow("B01", run, "assets", kAssets, "count");
        REQUIRE(scene.nodeCount() == kInstances + 1);
        const bool walked = checksum != 0.0;
        REQUIRE(walked);
    }
    for (int run = 0; run < kRuns; ++run) {
        sim::Scene scene;
        scene.beginEdit();
        auto cursor = scene.root();
        const auto started = std::chrono::steady_clock::now();
        for (int level = 0; level < kDepth; ++level) {
            auto node = scene.makeNode(sim::NodeKind::Assembly, "L" + std::to_string(level));
            scene.addChild(cursor, node);
            cursor = node;
        }
        scene.endEdit();
        nested[run].createMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        const auto walkStart = std::chrono::steady_clock::now();
        const sim::Mat4 world = cursor->worldMatrix();
        nested[run].walkMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - walkStart).count();
        nested[run].rss = rssKb();
        printRow("B05", run, "create_ms", nested[run].createMs, "ms");
        printRow("B05", run, "leaf_world_ms", nested[run].walkMs, "ms");
        printRow("B05", run, "rss_kb", nested[run].rss, "kb");
        printRow("B05", run, "depth", kDepth, "count");
        REQUIRE(world.m[15] == 1.0);
        (void)world;
    }
    auto column = [](const std::vector<Sample>& rows, double Sample::*field) {
        std::vector<double> values;
        for (const Sample& row : rows) {
            values.push_back(row.*field);
        }
        return values;
    };
    const auto create = column(flat, &Sample::createMs);
    std::cout << "G0_BENCH_SUMMARY,B01,create_ms,p50," << percentile(create, 0.50) << ",p95," << percentile(create, 0.95)
              << ",p99," << percentile(create, 0.99) << ",n,3,synthetic,targets_not_claimed\n";
}
