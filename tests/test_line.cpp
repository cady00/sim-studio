#include "sim/ai/Commands.hpp"
#include "sim/core/DemoScene.hpp"
#include "sim/core/LineModel.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/io/ProjectStore.hpp"
#include "sim/io/ZipArchive.hpp"
#include "sim/scripting/ScriptHost.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

using Catch::Approx;

namespace {

sim::Operation task(const std::string& id, const std::string& parent, int index, const std::string& station, double seconds) {
    sim::Operation operation;
    operation.id = id;
    operation.name = id;
    operation.parent = parent;
    operation.index = index;
    operation.station = station;
    operation.kind = sim::OpKind::Task;
    operation.seconds = seconds;
    return operation;
}

}  // namespace

TEST_CASE("stations snap, align, and distribute on the plan grid", "[line]") {
    sim::LineLayout layout;
    layout.gridMm = 500;
    sim::Station left;
    left.id = "left";
    left.x = 1230;
    left.y = 260;
    left.width = 2000;
    sim::Station mid;
    mid.id = "mid";
    mid.x = 1800;
    mid.y = 40;
    mid.width = 1000;
    sim::Station right;
    right.id = "right";
    right.x = 4100;
    right.y = -80;
    right.width = 2000;
    layout.stations = {left, mid, right};
    sim::snapStations(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(1000));
    REQUIRE(sim::findStation(layout, "left")->y == Approx(500));

    layout.stations[0].x = 1000;
    layout.stations[0].width = 2000;
    layout.stations[1].x = 4000;
    layout.stations[1].width = 1000;
    layout.stations[2].x = 7000;
    layout.stations[2].width = 2000;
    sim::alignStationsMinX(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(1000));
    REQUIRE(sim::findStation(layout, "mid")->x == Approx(500));
    REQUIRE(sim::findStation(layout, "right")->x == Approx(1000));

    layout.stations[0].x = 0;
    layout.stations[1].x = 100;
    layout.stations[2].x = 1000;
    layout.gridMm = 100;
    sim::distributeStationsX(layout);
    REQUIRE(sim::findStation(layout, "left")->x == Approx(0));
    REQUIRE(sim::findStation(layout, "mid")->x == Approx(500));
    REQUIRE(sim::findStation(layout, "right")->x == Approx(1000));
}

TEST_CASE("ascii dxf lines and polylines become plan segments", "[line]") {
    const char* text = R"(0
SECTION
2
ENTITIES
0
LINE
10
0
20
10
11
30
21
40
0
LWPOLYLINE
70
1
10
0
20
0
10
10
20
0
10
10
20
5
0
CIRCLE
10
0
20
0
40
2
0
ENDSEC
0
EOF
)";
    const sim::DxfResult parsed = sim::parseDxf(text, 2);
    REQUIRE(parsed.ok);
    REQUIRE(parsed.lines.size() == 1 + 3 + 16);
    REQUIRE(parsed.lines.front().second.x == Approx(60));
    REQUIRE(parsed.lines.front().second.y == Approx(80));

    const sim::DxfResult binary = sim::parseDxf(std::string("AutoCAD Binary DXF\r\n\0", 22), 1);
    REQUIRE_FALSE(binary.ok);
    const sim::DxfResult garbage = sim::parseDxf("not a drawing", 1);
    REQUIRE_FALSE(garbage.ok);
}

TEST_CASE("sequence and parallel operations expose the critical path", "[line]") {
    sim::LineLayout layout;
    layout.stations.push_back(sim::Station{"s1", "A", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    layout.stations.push_back(sim::Station{"s2", "B", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    layout.stations.push_back(sim::Station{"s3", "C", 0, 0, 1000, 1000, sim::StationKind::Manual, {}, {}});
    std::vector<sim::Operation> operations;
    sim::Operation seq;
    seq.id = "seq";
    seq.kind = sim::OpKind::Sequence;
    sim::Operation par;
    par.id = "par";
    par.kind = sim::OpKind::Parallel;
    par.after = {"seq"};
    operations.push_back(seq);
    operations.push_back(par);
    operations.push_back(task("a", "seq", 0, "s1", 10));
    operations.push_back(task("b", "seq", 1, "s2", 4));
    operations.push_back(task("c", "par", 0, "s1", 6));
    operations.push_back(task("d", "par", 1, "s3", 3));
    sim::Operation tail = task("e", "", 0, "s2", 2);
    tail.after = {"par"};
    operations.push_back(tail);

    const sim::ScheduleReport report = sim::scheduleOperations(operations, layout);
    REQUIRE(report.error.empty());
    REQUIRE(report.makespan == Approx(22));
    REQUIRE(report.bottleneck == "s1");
    REQUIRE(report.criticalIds == std::vector<std::string>{"a", "b", "c", "e"});

    operations[2].after = {"b"};
    operations[3].after = {"a"};
    const sim::ScheduleReport cycle = sim::scheduleOperations(operations, layout);
    REQUIRE_FALSE(cycle.error.empty());
    REQUIRE(cycle.bars.empty());
}

TEST_CASE("cycle report writes csv and a spreadsheet", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    const sim::ScheduleReport report = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(report.makespan == Approx(32));
    REQUIRE(report.bottleneck == "st_weld");
    const auto csvPath = std::filesystem::temp_directory_path() / "simstudio_cycle.csv";
    const auto xlsxPath = std::filesystem::temp_directory_path() / "simstudio_cycle.xlsx";
    std::string error;
    REQUIRE(sim::exportScheduleCsv(report, csvPath.string(), error));
    std::ifstream csv(csvPath);
    std::string header;
    std::getline(csv, header);
    REQUIRE(header == "station,operation,start_s,finish_s,critical,bottleneck");
    const std::string body((std::istreambuf_iterator<char>(csv)), std::istreambuf_iterator<char>());
    REQUIRE(body.find("焊接") != std::string::npos);
    REQUIRE(body.find("st_weld") != std::string::npos);
    REQUIRE(sim::exportScheduleXlsx(report, xlsxPath.string(), error));
    std::vector<sim::ZipEntry> entries;
    REQUIRE(sim::ZipArchive::read(xlsxPath.string(), entries, error));
    const auto sheet = std::find_if(entries.begin(), entries.end(), [](const sim::ZipEntry& entry) {
        return entry.name == "xl/worksheets/sheet1.xml";
    });
    REQUIRE(sheet != entries.end());
    const std::string xml(sheet->data.begin(), sheet->data.end());
    REQUIRE(xml.find("焊接") != std::string::npos);
}

TEST_CASE("ppr xml import and project version 9 keep the line", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    const std::size_t stations = scene.layout.stations.size();
    const sim::EmsImportResult missing = sim::importEmsXml("missing.ems.xml", scene);
    REQUIRE_FALSE(missing.ok);
    REQUIRE(missing.message.find("cannot open") != std::string::npos);
    REQUIRE(scene.layout.stations.size() == stations);

    const auto xmlPath = std::filesystem::temp_directory_path() / "simstudio_ppr.xml";
    {
        std::ofstream out(xmlPath);
        out << "<Plant name=\"cell\"><Product id=\"p1\" name=\"bracket\"/>"
               "<Process id=\"w1\" name=\"weld\" duration=\"3.5\" station=\"st1\"/>"
               "<Resource id=\"r1\" name=\"robot\" station=\"st1\" resource=\"kr420\"/></Plant>";
    }
    sim::Scene parsed;
    const sim::EmsImportResult imported = sim::importEmsXml(xmlPath.string(), parsed);
    REQUIRE(imported.ok);
    REQUIRE(parsed.ppr.products.size() == 1);
    REQUIRE(parsed.ppr.processes.size() == 1);
    REQUIRE(parsed.ppr.plants.size() == 2);
    REQUIRE(parsed.operations.size() == 1);
    REQUIRE(parsed.operations.front().seconds == Approx(3.5));
    sim::Scene garbage;
    garbage.layout.stations.push_back(sim::Station{});
    const auto badPath = std::filesystem::temp_directory_path() / "simstudio_bad.xml";
    {
        std::ofstream out(badPath);
        out << "<Note>hello</Note>";
    }
    const sim::EmsImportResult rejected = sim::importEmsXml(badPath.string(), garbage);
    REQUIRE_FALSE(rejected.ok);
    REQUIRE(garbage.layout.stations.size() == 1);
    REQUIRE(garbage.ppr.products.empty());

    const auto path = std::filesystem::temp_directory_path() / "simstudio_line.simproj";
    REQUIRE(sim::ProjectStore::save(scene, path.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(path.string(), loaded).ok);
    REQUIRE(loaded.layout.example);
    REQUIRE(loaded.layout.stations.size() == 4);
    REQUIRE(loaded.operations.size() == scene.operations.size());
    REQUIRE(loaded.ppr.plants.size() == 1);
    REQUIRE(loaded.ppr.plants.front().resource == "kr420");

    sim::ZipEntry project;
    project.name = "project.json";
    const std::string json = R"({"format":"simstudio.project","version":7,"name":"old","root":{"id":"root","name":"World","kind":"assembly","children":[]}})";
    project.data.assign(json.begin(), json.end());
    const auto oldPath = std::filesystem::temp_directory_path() / "simstudio_v7.simproj";
    std::string error;
    REQUIRE(sim::ZipArchive::write(oldPath.string(), {project}, error));
    sim::Scene old;
    old.layout.stations.push_back(sim::Station{});
    REQUIRE(sim::ProjectStore::load(oldPath.string(), old).ok);
    REQUIRE(old.layout.stations.empty());
    REQUIRE(old.operations.empty());
    REQUIRE(old.ppr.products.empty());
}

TEST_CASE("example weld line links the kr420 path and logic block", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    REQUIRE(scene.layout.example);
    REQUIRE(scene.layout.title == "示例焊装线");
    REQUIRE(scene.layout.stations.size() == 4);
    const sim::Station* weld = sim::findStation(scene.layout, "st_weld");
    REQUIRE(weld != nullptr);
    REQUIRE(weld->resource == "kr420");
    REQUIRE(weld->nodeName == "KR420");
    REQUIRE(scene.findByName("KR420") != nullptr);
    bool linked = false;
    for (const sim::Operation& operation : scene.operations) {
        if (operation.id == "weld") {
            linked = operation.path == "cycle" && operation.logic == "weld_seq" && operation.seconds == Approx(12);
        }
    }
    REQUIRE(linked);
    REQUIRE(scene.robotPaths.front().name == "cycle");
    REQUIRE(scene.logicBlocks.front().name == "weld_seq");
    const sim::ScheduleReport report = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(report.bottleneckName == "焊接");
    REQUIRE(std::find(report.criticalIds.begin(), report.criticalIds.end(), "prep") == report.criticalIds.end());
}

TEST_CASE("line commands and python read the same schedule", "[line]") {
    sim::Scene scene;
    sim::CommandExecutor commands(scene, nullptr, nullptr);
    const auto catalog = sim::CommandExecutor::catalog();
    bool listed = false;
    for (const auto& entry : catalog) {
        if (entry.value("op", "") == "line_schedule") {
            listed = entry.value("executable", false);
        }
    }
    REQUIRE(listed);
    REQUIRE(commands.execute({{"op", "line_add_station"}, {"id", "s1"}, {"name", "Load"}, {"x", 10}, {"y", 20}, {"kind", "manual"}}).ok);
    REQUIRE(commands.execute({{"op", "line_add_op"},
                              {"id", "job"},
                              {"name", "Load"},
                              {"station", "s1"},
                              {"kind", "task"},
                              {"seconds", 4}})
                .ok);
    const sim::CommandResult schedule = commands.execute({{"op", "line_schedule"}});
    REQUIRE(schedule.ok);
    REQUIRE(schedule.value.at("makespan").get<double>() == Approx(4));
    REQUIRE(schedule.value.at("bottleneck").get<std::string>() == "s1");
    const sim::CommandResult reserved = commands.execute({{"op", "ems_import"}, {"path", "cell.xml"}});
    REQUIRE_FALSE(reserved.ok);
    REQUIRE(scene.layout.stations.size() == 1);

    auto host = sim::createScriptHost();
    host->bind(&scene);
    std::string output;
    std::string error;
    REQUIRE(host->execute("stations = sim.line_stations()\nreport = sim.line_schedule()\ncounts = sim.ppr_counts()\n"
                          "ems = sim.ems_import('cell.xml')\n"
                          "print(len(stations), report['makespan'], report['bottleneck'], ems['ok'], counts['example'])\n",
                          output, error));
    REQUIRE(error.empty());
    REQUIRE(output.find("1 4.0 s1 False") != std::string::npos);
}

TEST_CASE("gantt drag writes wait and a pinned finish", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    REQUIRE(sim::adjustOperationStart(scene.operations, scene.layout, "unload", 30));
    sim::Operation* unload = nullptr;
    for (sim::Operation& operation : scene.operations) {
        if (operation.id == "unload") {
            unload = &operation;
        }
    }
    REQUIRE(unload != nullptr);
    REQUIRE(unload->wait > 1.0);
    const double before = sim::scheduleOperations(scene.operations, scene.layout).makespan;
    REQUIRE(sim::adjustOperationFinish(scene.operations, scene.layout, "unload", 40));
    REQUIRE(unload->timeSource == "pinned");
    REQUIRE(unload->timeNote == "gantt finish");
    const sim::ScheduleReport after = sim::scheduleOperations(scene.operations, scene.layout);
    REQUIRE(after.makespan > before);
}

TEST_CASE("dxf arc spline insert and a binary line", "[line]") {
    const char* text = R"(0
SECTION
2
BLOCKS
0
BLOCK
2
MARK
0
LINE
8
Marks
10
0
20
0
11
10
21
0
0
ENDBLK
0
ENDSEC
0
SECTION
2
ENTITIES
0
ARC
8
Floor
10
0
20
0
40
10
50
0
51
90
0
SPLINE
8
Curve
71
3
10
0
20
0
10
10
20
0
10
10
20
10
10
0
20
10
0
INSERT
8
Placed
2
MARK
10
100
20
50
0
ENDSEC
0
EOF
)";
    const sim::DxfResult parsed = sim::parseDxf(text, 1);
    REQUIRE(parsed.ok);
    REQUIRE(parsed.lines.size() > 4);
    bool sawFloor = false;
    bool sawPlaced = false;
    for (const std::string& layer : parsed.layers) {
        sawFloor = sawFloor || layer == "Floor";
        sawPlaced = sawPlaced || layer == "Placed";
    }
    REQUIRE(sawFloor);
    REQUIRE(sawPlaced);

    std::string binary("AutoCAD Binary DXF\r\n\0", 22);
    auto pushCode = [&](int code) {
        binary.push_back(static_cast<char>(code & 0xff));
        binary.push_back(static_cast<char>((code >> 8) & 0xff));
    };
    auto pushString = [&](int code, const char* value) {
        pushCode(code);
        binary.append(value);
        binary.push_back('\0');
    };
    auto pushDouble = [&](int code, double value) {
        pushCode(code);
        binary.append(reinterpret_cast<const char*>(&value), sizeof(value));
    };
    pushString(0, "SECTION");
    pushString(2, "ENTITIES");
    pushString(0, "LINE");
    pushString(8, "Bin");
    pushDouble(10, 0);
    pushDouble(20, 0);
    pushDouble(11, 25);
    pushDouble(21, 0);
    pushString(0, "ENDSEC");
    pushString(0, "EOF");
    const sim::DxfResult binaryResult = sim::parseDxf(binary, 1);
    REQUIRE(binaryResult.ok);
    REQUIRE(binaryResult.lines.size() == 1);
    REQUIRE(binaryResult.layers.front() == "Bin");
    REQUIRE(binaryResult.lines.front().second.x == Approx(25));

    const char* extra = R"(0
SECTION
2
ENTITIES
0
3DFACE
8
Solid
10
0
20
0
30
0
11
10
21
0
31
5
12
10
22
10
32
5
13
0
23
10
33
0
0
ELLIPSE
8
Solid
10
0
20
0
11
20
21
0
40
0.5
41
0
42
6.283185
0
HATCH
8
Fill
93
3
10
0
20
0
10
30
20
0
10
30
20
15
0
DIMENSION
8
Notes
10
0
20
5
11
15
21
5
13
0
23
0
14
40
24
0
0
ENDSEC
0
EOF
)";
    const sim::DxfResult drawn = sim::parseDxf(extra, 1);
    REQUIRE(drawn.ok);
    bool sawHeight = false;
    bool sawSpan = false;
    int solid = 0;
    int fill = 0;
    int notes = 0;
    for (std::size_t i = 0; i < drawn.lines.size(); ++i) {
        const std::string layer = i < drawn.layers.size() ? drawn.layers[i] : "";
        solid += layer == "Solid" ? 1 : 0;
        fill += layer == "Fill" ? 1 : 0;
        notes += layer == "Notes" ? 1 : 0;
        if (std::abs(drawn.lines[i].first.z) > 1 || std::abs(drawn.lines[i].second.z) > 1) {
            sawHeight = true;
        }
        const double span = std::abs(drawn.lines[i].second.x - drawn.lines[i].first.x);
        if (layer == "Notes" && span > 30) {
            sawSpan = true;
        }
    }
    REQUIRE(solid > 8);
    REQUIRE(fill >= 3);
    REQUIRE(notes >= 1);
    REQUIRE(sawHeight);
    REQUIRE(sawSpan);
}

TEST_CASE("dxf hatch patterns, dimension text, and nested inserts", "[line]") {
    const char* text = R"(0
SECTION
2
BLOCKS
0
BLOCK
2
INNER
0
LINE
8
Inner
10
0
20
0
11
10
21
0
0
ENDBLK
0
BLOCK
2
OUTER
0
INSERT
2
INNER
10
0
20
4
0
ENDBLK
0
ENDSEC
0
SECTION
2
ENTITIES
0
INSERT
8
Placed
2
OUTER
10
100
20
0
0
HATCH
8
Fill
70
1
10
0
20
0
10
20
20
0
10
20
20
10
10
0
20
10
0
DIMENSION
8
Notes
1
40
10
0
20
0
11
20
21
0
0
ENDSEC
0
EOF
)";
    const sim::DxfResult drawn = sim::parseDxf(text, 1);
    REQUIRE(drawn.ok);
    int placed = 0;
    int fill = 0;
    int arrows = 0;
    bool textSpan = false;
    for (std::size_t i = 0; i < drawn.lines.size(); ++i) {
        const std::string layer = i < drawn.layers.size() ? drawn.layers[i] : "";
        const sim::Vec3 a = drawn.lines[i].first;
        const sim::Vec3 b = drawn.lines[i].second;
        if (layer == "Inner" || layer == "Placed") {
            const bool shifted = std::abs(a.y - 4) < 1e-6 && std::abs(b.y - 4) < 1e-6 && std::abs(a.x - 100) < 1e-6 &&
                                 std::abs(b.x - 110) < 1e-6;
            placed += shifted ? 1 : 0;
        }
        fill += layer == "Fill" ? 1 : 0;
        if (layer == "Notes") {
            const double span = sim::length(b - a);
            if (span > 2 && span < 12) {
                ++arrows;
            }
            if (std::abs(a.x - 20) < 8 && std::abs(b.x - 20) < 12) {
                textSpan = true;
            }
        }
    }
    REQUIRE(placed == 1);
    REQUIRE(fill > 4);
    REQUIRE(arrows >= 2);
    REQUIRE(textSpan);
}

TEST_CASE("logic blocks and the operation list can cycle", "[line]") {
    sim::Scene scene;
    sim::LogicBlock block;
    block.name = "spin";
    block.repeat = true;
    sim::LogicAction pulse;
    pulse.kind = "set_output";
    pulse.signal = "tick";
    sim::LogicAction wait;
    wait.kind = "wait";
    wait.seconds = 0.05;
    sim::LogicStep step;
    step.actions = {pulse, wait};
    block.steps.push_back(step);
    scene.logicBlocks.push_back(block);
    sim::LogicMachine machine;
    std::string error;
    REQUIRE(machine.start(scene, "spin", error));
    for (int i = 0; i < 40; ++i) {
        machine.step(scene, 0.01);
    }
    REQUIRE_FALSE(machine.finished("spin"));
    REQUIRE(scene.findSignal("tick") != nullptr);
    REQUIRE(scene.findSignal("tick")->flag);

    sim::Station slow;
    slow.id = "slow";
    slow.x = 0;
    sim::Station fast;
    fast.id = "fast";
    fast.x = 1000;
    scene.layout.stations = {slow, fast};
    scene.layout.cyclic = true;
    sim::Operation line;
    line.id = "line";
    line.kind = sim::OpKind::Sequence;
    sim::Operation first = task("first", "line", 0, "slow", 10);
    sim::Operation second = task("second", "line", 1, "fast", 10);
    second.after = {"first"};
    scene.operations = {line, first, second};
    auto feeder = scene.makeNode(sim::NodeKind::Part, "Feeder");
    feeder->mech.kind = sim::MechKind::Source;
    feeder->mech.interval = 10;
    scene.addChild(scene.root(), feeder);

    sim::LineProduction production;
    REQUIRE(production.start(scene, error));
    production.step(100);
    const sim::ProductionStats stats = production.stats();
    REQUIRE(stats.output == 8);
    REQUIRE(stats.jobsPerHour == Approx(288));
    REQUIRE(stats.stations.size() == 2);
    REQUIRE(stats.stations[0].busySeconds == Approx(90).margin(0.05));
    REQUIRE(stats.stations[0].starvedSeconds == Approx(10).margin(0.05));
    REQUIRE(stats.stations[0].blockedSeconds == Approx(0).margin(0.05));
    REQUIRE(stats.stations[1].starvedSeconds == Approx(20).margin(0.05));
    REQUIRE(stats.stations[1].busySeconds == Approx(80).margin(0.05));

    scene.operations[1].seconds = 5;
    scene.operations[2].seconds = 10;
    feeder->mech.interval = 5;
    sim::LineProduction blocked;
    REQUIRE(blocked.start(scene, error));
    blocked.step(100);
    const sim::ProductionStats tight = blocked.stats();
    REQUIRE(tight.output >= 8);
    REQUIRE(tight.stations[0].blockedSeconds > 1.0);
    REQUIRE(tight.sourceBlockedSeconds > 1.0);
}

TEST_CASE("the example line keeps feeding for an hour", "[line]") {
    sim::Scene scene;
    sim::buildExampleLine(scene);
    REQUIRE(scene.layout.cyclic);
    bool repeating = false;
    bool feeder = false;
    for (const sim::LogicBlock& block : scene.logicBlocks) {
        repeating = repeating || block.repeat;
    }
    std::vector<std::shared_ptr<sim::SceneNode>> stack{scene.root()};
    while (!stack.empty()) {
        auto node = stack.back();
        stack.pop_back();
        if (!node) {
            continue;
        }
        for (const auto& child : node->children) {
            stack.push_back(child);
        }
        if (node->mech.kind == sim::MechKind::Source && node->mech.prototype == "Blank") {
            feeder = true;
        }
    }
    REQUIRE(repeating);
    REQUIRE(feeder);
    sim::LineProduction production;
    std::string error;
    REQUIRE(production.start(scene, error));
    production.step(3600);
    const sim::ProductionStats stats = production.stats();
    REQUIRE(stats.output > 200);
    REQUIRE(stats.jobsPerHour > 200);
    const sim::StationAccount* weld = nullptr;
    for (const sim::StationAccount& station : stats.stations) {
        if (station.id == "st_weld") {
            weld = &station;
        }
    }
    REQUIRE(weld != nullptr);
    REQUIRE(weld->busySeconds / stats.time > 0.8);
    std::cout << "LINE_JPH output=" << stats.output << " jph=" << stats.jobsPerHour << " weld_util="
              << (weld->busySeconds / stats.time) << " weld_block=" << weld->blockedSeconds
              << " weld_starve=" << weld->starvedSeconds << " source_block=" << stats.sourceBlockedSeconds << std::endl;

    const std::string note = production.note(true);
    REQUIRE(note.find("估算") != std::string::npos);
    REQUIRE(note.find("不含网格碰撞") != std::string::npos);
    REQUIRE(note.find("也不是一小时接触计数") != std::string::npos);
    REQUIRE(note.find("st_weld 占用 12.0 s") != std::string::npos);
    REQUIRE(note.find("工序表") != std::string::npos);
    REQUIRE(note.find("路径规划") == std::string::npos);
    REQUIRE(note.find("实测") == std::string::npos);
    REQUIRE(stats.jobsPerHour < 360);

    for (sim::Operation& operation : scene.operations) {
        if (operation.id == "weld") {
            operation.seconds = 6;
            operation.timeSource = "plan";
        }
    }
    sim::LineProduction planned;
    REQUIRE(planned.start(scene, error));
    planned.step(3600);
    const std::string plannedNote = planned.note(true);
    REQUIRE(plannedNote.find("估算") != std::string::npos);
    REQUIRE(plannedNote.find("路径规划") != std::string::npos);
    REQUIRE(plannedNote.find("st_weld 占用 6.0 s") != std::string::npos);
    REQUIRE(plannedNote.find("实测") == std::string::npos);
    REQUIRE(plannedNote.find("不含网格碰撞") != std::string::npos);
    REQUIRE(planned.stats().jobsPerHour != Approx(stats.jobsPerHour));
}

TEST_CASE("sequence transitions and signal waits shift the timeline") {
    sim::Operation load;
    load.id = "load";
    load.seconds = 2;
    sim::Operation weld;
    weld.id = "weld";
    weld.seconds = 4;
    weld.after = {"load"};
    sim::Operation unload;
    unload.id = "unload";
    unload.seconds = 2;
    sim::Operation hold;
    hold.id = "hold";
    hold.seconds = 1;
    sim::SequenceEvent transition;
    transition.id = "tr";
    transition.kind = "transition";
    transition.operation = "weld";
    transition.target = "unload";
    transition.delay = 0.4;
    sim::SequenceEvent setDone;
    setDone.id = "set";
    setDone.kind = "signal";
    setDone.operation = "weld";
    setDone.edge = "finish";
    setDone.signal = "weld_done";
    setDone.action = "set";
    sim::SequenceEvent waitDone;
    waitDone.id = "wait";
    waitDone.kind = "signal";
    waitDone.operation = "hold";
    waitDone.signal = "weld_done";
    waitDone.action = "wait";
    const sim::ScheduleReport report =
        sim::scheduleSequence({load, weld, unload, hold}, {transition, setDone, waitDone}, {});
    REQUIRE(report.error.empty());
    const auto bar = [&](const std::string& id) {
        for (const sim::ScheduleBar& item : report.bars) {
            if (item.id == id) {
                return item;
            }
        }
        return sim::ScheduleBar{};
    };
    REQUIRE(bar("weld").start == Approx(2));
    REQUIRE(bar("weld").finish == Approx(6));
    REQUIRE(bar("unload").start == Approx(6.4));
    REQUIRE(bar("hold").start == Approx(6));
    REQUIRE(report.makespan == Approx(8.4));

    sim::Scene scene;
    scene.operations = {load, weld, unload, hold};
    scene.sequence = {transition, setDone, waitDone};
    const auto path = std::filesystem::temp_directory_path() / "simstudio_sequence.simproj";
    REQUIRE(sim::ProjectStore::save(scene, path.string()).ok);
    sim::Scene loaded;
    REQUIRE(sim::ProjectStore::load(path.string(), loaded).ok);
    REQUIRE(loaded.sequence.size() == 3);
    REQUIRE(loaded.sequence.front().delay == Approx(0.4));
}

TEST_CASE("ems xml round-trips operations resources parts welds paths and placements", "[line]") {
    const std::string unknown = "<CustomNote id=\"keep\" note=\"a&amp;b\">inner <Extra v=\"1\"/> text</CustomNote>";
    const std::string comment = "<!--keep-me-->";
    const auto xmlPath = std::filesystem::temp_directory_path() / "simstudio_ems_round.xml";
    {
        std::ofstream out(xmlPath);
        out << "<eBOP name=\"cell\">" << comment << "<Part id=\"p1\" name=\"bracket\" part=\"bracket\">"
               "<Location x=\"10\" y=\"20\" z=\"30\" rx=\"10\" ry=\"20\" rz=\"30\"/>"
               "</Part>"
               "<Operation id=\"weld\" name=\"焊接\" duration=\"4.5\" station=\"st_weld\" path=\"weld\" after=\"load\"/>"
               "<Resource id=\"r1\" name=\"IRB120\" station=\"st_weld\" resource=\"IRB120\"/>"
               "<WeldPoint id=\"s1\" name=\"spot1\" x=\"100\" y=\"0\" z=\"20\" nx=\"0\" ny=\"0\" nz=\"1\" part=\"bracket\"/>"
               "<RobotPath name=\"weld\" resource=\"IRB120\">"
               "<Location name=\"home\" motion=\"ptp\" joints=\"0 0 0 0 0 0\" speed=\"200\"/>"
               "<Location name=\"approach\" motion=\"lin\" x=\"15\" y=\"-25\" z=\"20\" speed=\"80\"/>"
               "</RobotPath>"
            << unknown << "</eBOP>";
    }
    sim::Scene scene;
    const sim::EmsImportResult imported = sim::importEmsXml(xmlPath.string(), scene);
    REQUIRE(imported.ok);
    REQUIRE(scene.ppr.products.size() == 1);
    REQUIRE(scene.ppr.products.front().partName == "bracket");
    REQUIRE(scene.operations.size() == 1);
    REQUIRE(scene.operations.front().seconds == Approx(4.5));
    REQUIRE(scene.operations.front().path == "weld");
    REQUIRE(scene.operations.front().after.front() == "load");
    REQUIRE(scene.ppr.plants.size() == 1);
    REQUIRE(scene.ppr.plants.front().resource == "IRB120");
    REQUIRE(scene.welds.size() == 1);
    REQUIRE(scene.welds.front().position.x == Approx(100));
    REQUIRE(scene.welds.front().normal.z == Approx(1));
    REQUIRE(scene.robotPaths.size() == 1);
    REQUIRE(scene.robotPaths.front().points.size() == 2);
    REQUIRE(scene.robotPaths.front().points[0].joints.size() == 6);
    REQUIRE(scene.robotPaths.front().points[1].move == sim::PathMove::Lin);
    REQUIRE(scene.robotPaths.front().points[1].position.y == Approx(-25));
    const auto bracket = scene.findByName("bracket");
    REQUIRE(bracket != nullptr);
    REQUIRE(bracket->local.translation.z == Approx(30));
    double rx = 0;
    double ry = 0;
    double rz = 0;
    bracket->local.rotation.toEulerXYZ(rx, ry, rz);
    REQUIRE(sim::rad2deg(ry) == Approx(20).margin(0.05));

    scene.operations.front().seconds = 6.25;
    scene.welds.front().position.y = 8;
    bracket->local.translation.x = 12;
    const auto outPath = std::filesystem::temp_directory_path() / "simstudio_ems_out.xml";
    REQUIRE(sim::exportEmsXml(scene, outPath.string()).ok);
    std::ifstream written(outPath);
    std::stringstream buffer;
    buffer << written.rdbuf();
    const std::string text = buffer.str();
    REQUIRE(text.find(unknown) != std::string::npos);
    REQUIRE(text.find(comment) != std::string::npos);
    REQUIRE(text.find("duration=\"6.25\"") != std::string::npos);

    sim::Scene again;
    REQUIRE(sim::importEmsXml(outPath.string(), again).ok);
    REQUIRE(again.operations.front().seconds == Approx(6.25));
    REQUIRE(again.welds.front().position.y == Approx(8));
    REQUIRE(again.findByName("bracket")->local.translation.x == Approx(12));
    REQUIRE(again.robotPaths.front().points[0].joints[0] == Approx(0));
    REQUIRE(again.ppr.hasEms);

    sim::Scene built;
    built.ppr.products.push_back({"p1", "panel", "", "panel"});
    auto panel = built.makeNode(sim::NodeKind::Part, "panel");
    panel->local.translation = {1, 2, 3};
    panel->local.rotation = sim::Quat::fromEulerXYZ(sim::deg2rad(10), 0, 0);
    built.addChild(built.root(), panel);
    built.ppr.plants.push_back({"r1", "IRB120", "", "st_weld", "IRB120"});
    sim::Operation job;
    job.id = "weld";
    job.name = "weld";
    job.seconds = 3;
    job.station = "st_weld";
    job.path = "weld";
    built.operations.push_back(job);
    sim::WeldSpot spot;
    spot.id = "s1";
    spot.position = {4, 5, 6};
    spot.normal = {0, 1, 0};
    built.welds.push_back(spot);
    sim::RobotPath path;
    path.name = "weld";
    path.resource = "IRB120";
    sim::PathPoint home;
    home.name = "home";
    home.joints = {1, 2, 3};
    home.speed = 50;
    path.points.push_back(home);
    built.robotPaths.push_back(path);
    const auto synthPath = std::filesystem::temp_directory_path() / "simstudio_ems_synth.xml";
    REQUIRE(sim::exportEmsXml(built, synthPath.string()).ok);
    sim::Scene synth;
    REQUIRE(sim::importEmsXml(synthPath.string(), synth).ok);
    REQUIRE(synth.operations.front().seconds == Approx(3));
    REQUIRE(synth.ppr.plants.front().resource == "IRB120");
    REQUIRE(synth.welds.front().normal.y == Approx(1));
    REQUIRE(synth.robotPaths.front().points.front().joints[2] == Approx(3));
    REQUIRE(synth.findByName("panel")->local.translation.z == Approx(3));
    const auto stored = std::filesystem::temp_directory_path() / "simstudio_ems.simproj";
    REQUIRE(sim::ProjectStore::save(again, stored.string()).ok);
    sim::Scene fromProject;
    REQUIRE(sim::ProjectStore::load(stored.string(), fromProject).ok);
    REQUIRE(fromProject.ppr.hasEms);
    const auto kept = std::filesystem::temp_directory_path() / "simstudio_ems_kept.xml";
    REQUIRE(sim::exportEmsXml(fromProject, kept.string()).ok);
    std::ifstream keptFile(kept);
    std::stringstream keptText;
    keptText << keptFile.rdbuf();
    REQUIRE(keptText.str().find(unknown) != std::string::npos);
}
