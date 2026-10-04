#include "sim/app/MainWindow.hpp"

#include "sim/core/Kinematics.hpp"
#include "sim/core/LineModel.hpp"
#include "sim/motion/Motion.hpp"
#include "sim/motion/WeldPlan.hpp"

#include <QApplication>
#include <QDockWidget>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTest>
#include <QTreeWidget>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <sstream>

namespace {

void logStep(const char* text) {
    std::fprintf(stderr, "E2E %s\n", text);
    std::fflush(stderr);
}

int fail(const char* text, QWidget* window) {
    std::fprintf(stderr, "E2E FAIL %s\n", text);
    std::fflush(stderr);
    if (window != nullptr) {
        window->grab().save(QStringLiteral("/opt/cursor/artifacts/ui/e2e-fail.png"));
    }
    return 1;
}

QPushButton* modeButton(QWidget* root, int index) {
    const char* names[] = {"建模", "运动学", "逻辑", "机器人", "调试", "虚拟调试"};
    const QString prefix = QString::fromUtf8(names[index]);
    for (QPushButton* button : root->findChildren<QPushButton*>("modeButton")) {
        if (button->text().startsWith(prefix)) {
            return button;
        }
    }
    return nullptr;
}

bool click(QWidget* widget) {
    auto* button = qobject_cast<QPushButton*>(widget);
    if (button == nullptr) {
        std::fprintf(stderr, "E2E click missing\n");
        return false;
    }
    if (!button->isVisible()) {
        std::fprintf(stderr, "E2E click hidden '%s'\n", button->text().toUtf8().constData());
        button->click();
        return true;
    }
    QTest::mouseClick(button, Qt::LeftButton);
    return true;
}

std::string weldCsvFromRobot(const sim::KinematicResource& robot) {
    sim::KinematicResource copy = robot;
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << "id,x,y,z,nx,ny,nz\n";
    int written = 0;
    const double q1Values[] = {-8.0, 0.0, 10.0};
    const double q2Values[] = {-8.0, -4.0};
    const double q3Values[] = {6.0, 12.0};
    if (copy.joints.size() < 3) {
        return {};
    }
    for (double q1 : q1Values) {
        for (double q2 : q2Values) {
            for (double q3 : q3Values) {
                if (written >= 3) {
                    break;
                }
                std::vector<double> seed(copy.joints.size(), 0.0);
                auto clampJoint = [&](int index, double value) {
                    return std::clamp(value, copy.joints[static_cast<std::size_t>(index)].lower,
                                      copy.joints[static_cast<std::size_t>(index)].upper);
                };
                seed[0] = clampJoint(0, q1);
                seed[1] = clampJoint(1, q2);
                seed[2] = clampJoint(2, q3);
                if (seed.size() >= 5) {
                    seed[4] = clampJoint(4, 35.0);
                }
                sim::setJointVector(copy, seed);
                const sim::CartesianPose pose = sim::forwardModel(copy);
                const sim::Vec3 toolZ = pose.rotation.rotate({0, 0, 1});
                sim::WeldSpot trial;
                trial.position = pose.position;
                trial.normal = length(toolZ) > 0.5 ? -toolZ : sim::Vec3{0, 0, 1};
                const sim::SpinSearch aligned = sim::searchWeldSpin(nullptr, copy, trial, 12, false);
                if (written == 0 && aligned.ok == false) {
                    const sim::IkResult solved = sim::inverseModel(copy, pose, seed);
                    std::fprintf(stderr,
                                 "E2E pose (%.0f %.0f %.0f) toolZ (%.2f %.2f %.2f) spin=%s ik=%s joints=%zu warn=%s\n",
                                 pose.position.x, pose.position.y, pose.position.z, toolZ.x, toolZ.y, toolZ.z,
                                 aligned.note.c_str(), solved.ok ? "ok" : "fail", solved.joints.size(), solved.warning.c_str());
                }
                if (!aligned.ok || aligned.singular || aligned.margin < 0.02) {
                    continue;
                }
                out << "w" << written << ',' << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ','
                    << (-toolZ.x) << ',' << (-toolZ.y) << ',' << (-toolZ.z) << '\n';
                ++written;
            }
        }
    }
    return written >= 2 ? out.str() : std::string{};
}

const sim::RobotPath* tourOf(const sim::Scene& scene) {
    for (const sim::RobotPath& path : scene.robotPaths) {
        if (path.name == "weld_tour") {
            return &path;
        }
    }
    return nullptr;
}

double weldSeconds(const sim::Scene& scene) {
    for (const sim::Operation& operation : scene.operations) {
        if (operation.path == "weld_tour") {
            return operation.seconds;
        }
    }
    return -1;
}

}  // namespace

int main(int argc, char* argv[]) {
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setSamples(4);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication application(argc, argv);
    const std::filesystem::path step =
        std::filesystem::path(SIM_SOURCE_DIR) / "testdata/local/KR420_R3080_KR280_R3080.stp";
    if (!std::filesystem::exists(step)) {
        std::fprintf(stderr, "E2E FAIL missing %s\n", step.string().c_str());
        return 2;
    }
    std::filesystem::create_directories("/opt/cursor/artifacts/ui");
    const QString projectPath = QStringLiteral("/tmp/simstudio-e2e.simproj");
    std::filesystem::remove(projectPath.toStdString());

    sim::MainWindow window(QStringLiteral("zh_CN"));
    window.resize(1440, 900);
    window.show();
    if (!QTest::qWaitForWindowExposed(&window)) {
        return fail("window was not exposed", &window);
    }
    QTest::qWait(200);
    if (window.document().resources.size() != 0 || !window.document().welds.empty()) {
        return fail("window did not start empty", &window);
    }

    logStep("import step");
    window.importStepAsync(QString::fromStdString(step.string()));
    QElapsedTimer timer;
    timer.start();
    while (window.importing() && timer.elapsed() < 180000) {
        QTest::qWait(200);
    }
    if (window.importing()) {
        return fail("step import still running after 180s", &window);
    }
    QTest::qWait(300);
    int parts = 0;
    if (window.document().root()) {
        std::vector<std::shared_ptr<sim::SceneNode>> stack{window.document().root()};
        while (!stack.empty()) {
            auto node = stack.back();
            stack.pop_back();
            if (!node) {
                continue;
            }
            for (const auto& child : node->children) {
                stack.push_back(child);
            }
            if (node->kind == sim::NodeKind::Part && !node->meshId.empty()) {
                ++parts;
            }
        }
    }
    std::fprintf(stderr, "E2E imported parts=%d\n", parts);
    if (parts < 10) {
        return fail("step import produced too few parts", &window);
    }

    logStep("group links");
    if (!click(modeButton(&window, 1))) {
        return fail("kinematics mode button", &window);
    }
    QTest::qWait(150);
    auto* resourceName = window.findChild<QLineEdit*>("motion.resourceName");
    if (resourceName == nullptr) {
        return fail("resource name field missing", &window);
    }
    resourceName->setText(QStringLiteral("kr420"));
    if (!click(window.findChild<QPushButton*>("motion.suggest"))) {
        return fail("suggest button", &window);
    }
    const sim::KinematicResource* robot = sim::findResource(window.document().resources, "kr420");
    if (robot == nullptr) {
        return fail("kr420 resource missing after suggest", &window);
    }
    std::fprintf(stderr, "E2E joints=%zu\n", robot->joints.size());
    for (const sim::KinJoint& joint : robot->joints) {
        std::fprintf(stderr, "E2E joint %s lower=%.1f upper=%.1f speed=%.1f\n", joint.name.c_str(), joint.lower, joint.upper,
                     joint.maxSpeed);
    }
    if (robot->joints.size() != 6) {
        const sim::SuggestResult suggestion = sim::suggestLinkGroups(window.document());
        std::fprintf(stderr, "E2E suggest %s links=%zu\n", suggestion.note.c_str(), suggestion.links.size());
        return fail("expected six revolute joints", &window);
    }
    const std::vector<sim::RevoluteLimit> limits = sim::kr420DatasheetLimits();
    for (std::size_t i = 0; i < limits.size(); ++i) {
        const sim::KinJoint& joint = robot->joints[i];
        if (std::abs(joint.lower - limits[i].lower) > 0.01 || std::abs(joint.upper - limits[i].upper) > 0.01 ||
            std::abs(joint.maxSpeed - limits[i].speed) > 0.01) {
            return fail("datasheet limits were not applied", &window);
        }
    }

    logStep("import welds");
    const std::string csv = weldCsvFromRobot(*robot);
    if (csv.size() < 10) {
        return fail("could not place reachable weld spots on the grouped robot", &window);
    }
    if (!click(modeButton(&window, 3))) {
        return fail("robot mode button", &window);
    }
    QTest::qWait(150);
    auto* weldDock = window.findChild<QDockWidget*>("dock.weld");
    if (weldDock != nullptr) {
        window.resizeDocks({weldDock}, {640}, Qt::Vertical);
        weldDock->raise();
    }
    auto* source = window.findChild<QPlainTextEdit*>("weldSource");
    if (source == nullptr) {
        return fail("weld source missing", &window);
    }
    source->setPlainText(QString::fromStdString(csv));
    if (!click(window.findChild<QPushButton*>("weldImport"))) {
        return fail("weld import button", &window);
    }
    if (window.document().welds.size() < 2) {
        return fail("weld import kept no spots", &window);
    }
    logStep("reach");
    if (!click(window.findChild<QPushButton*>("weldReach"))) {
        return fail("reach button", &window);
    }
    int solved = 0;
    int clear = 0;
    for (const sim::WeldSpot& spot : window.document().welds) {
        std::fprintf(stderr, "E2E spot %s checked=%d reach=%d margin=%.3f singular=%d collision=%d\n", spot.id.c_str(),
                     spot.checked ? 1 : 0, spot.reachable ? 1 : 0, spot.margin, spot.singular ? 1 : 0, spot.collision ? 1 : 0);
        if (spot.checked && !spot.singular && spot.margin > 0.02) {
            ++solved;
        }
        if (spot.reachable) {
            ++clear;
        }
    }
    std::fprintf(stderr, "E2E solved=%d clear=%d of %zu\n", solved, clear, window.document().welds.size());
    if (solved < 2) {
        return fail("fewer than two spots solved inside the joint limits", &window);
    }

    logStep("plan");
    if (!click(window.findChild<QPushButton*>("weldPlan"))) {
        return fail("plan button", &window);
    }
    const sim::RobotPath* tour = tourOf(window.document());
    if (tour == nullptr || tour->points.size() < 2) {
        return fail("weld tour was not stored", &window);
    }
    if (window.document().operations.size() < 2 || window.document().layout.stations.empty()) {
        return fail("operation tree has no weld task", &window);
    }

    logStep("cycle");
    if (!click(modeButton(&window, 0))) {
        return fail("model mode button", &window);
    }
    QTest::qWait(200);
    auto* operationDock = window.findChild<QDockWidget*>("dock.operations");
    auto* ganttDock = window.findChild<QDockWidget*>("dock.gantt");
    if (operationDock == nullptr || !operationDock->isVisible() || ganttDock == nullptr || !ganttDock->isVisible()) {
        return fail("operation tree or gantt stayed hidden", &window);
    }
    window.resizeDocks({operationDock, ganttDock}, {260, 280}, Qt::Vertical);
    auto* tree = window.findChild<QTreeWidget*>("line.operations");
    if (tree == nullptr || tree->topLevelItemCount() < 1) {
        return fail("operation tree is empty", &window);
    }
    if (!click(window.findChild<QPushButton*>("line.measure"))) {
        return fail("measure button", &window);
    }
    const double seconds = weldSeconds(window.document());
    const sim::ScheduleReport report = sim::scheduleOperations(window.document().operations, window.document().layout);
    std::fprintf(stderr, "E2E weld_seconds=%.3f makespan=%.3f note=%s\n", seconds, report.makespan,
                 window.findChild<QLabel*>("line.measureNote") == nullptr
                     ? ""
                     : window.findChild<QLabel*>("line.measureNote")->text().toUtf8().constData());
    if (!(seconds > 0.05) || !(report.makespan > 0.05)) {
        return fail("cycle time stayed at zero", &window);
    }
    ganttDock->raise();
    QTest::qWait(100);
    window.grab().save(QStringLiteral("/opt/cursor/artifacts/ui/e2e-gantt.png"));

    logStep("krl");
    if (!click(modeButton(&window, 3))) {
        return fail("robot mode for krl", &window);
    }
    QTest::qWait(150);
    if (weldDock != nullptr) {
        weldDock->raise();
    }
    if (!click(window.findChild<QPushButton*>("weldKrl"))) {
        return fail("krl button", &window);
    }
    auto* program = window.findChild<QPlainTextEdit*>("weldProgram");
    const QString krl = program == nullptr ? QString() : program->toPlainText();
    if (!krl.contains(QStringLiteral("E6AXIS")) && !krl.contains(QStringLiteral("DEF"))) {
        return fail("krl preview is empty", &window);
    }
    window.grab().save(QStringLiteral("/opt/cursor/artifacts/ui/e2e-krl.png"));

    logStep("save");
    const int weldCount = static_cast<int>(window.document().welds.size());
    const int jointCount = static_cast<int>(robot->joints.size());
    const int pointCount = static_cast<int>(tour->points.size());
    const double makespan = report.makespan;
    const double savedLower = robot->joints[0].lower;
    if (!window.saveProject(projectPath)) {
        return fail("save failed", &window);
    }

    logStep("reopen");
    sim::MainWindow again(QStringLiteral("zh_CN"));
    again.resize(1440, 900);
    again.show();
    again.openProject(projectPath);
    QTest::qWait(300);
    const sim::KinematicResource* againRobot = sim::findResource(again.document().resources, "kr420");
    const sim::RobotPath* againTour = tourOf(again.document());
    const sim::ScheduleReport againReport = sim::scheduleOperations(again.document().operations, again.document().layout);
    if (againRobot == nullptr || static_cast<int>(againRobot->joints.size()) != jointCount ||
        std::abs(againRobot->joints[0].lower - savedLower) > 0.01) {
        return fail("reopened joints differ", &again);
    }
    if (static_cast<int>(again.document().welds.size()) != weldCount) {
        return fail("reopened weld count differs", &again);
    }
    if (againTour == nullptr || static_cast<int>(againTour->points.size()) != pointCount) {
        return fail("reopened tour differs", &again);
    }
    if (std::abs(weldSeconds(again.document()) - seconds) > 0.02 || std::abs(againReport.makespan - makespan) > 0.02) {
        return fail("reopened cycle time differs", &again);
    }
    if (!click(modeButton(&again, 3))) {
        return fail("reopened robot mode", &again);
    }
    QTest::qWait(150);
    if (!click(again.findChild<QPushButton*>("weldKrl"))) {
        return fail("reopened krl button", &again);
    }
    auto* againProgram = again.findChild<QPlainTextEdit*>("weldProgram");
    if (againProgram == nullptr || againProgram->toPlainText() != krl) {
        return fail("reopened krl text differs", &again);
    }
    auto* againTree = again.findChild<QTreeWidget*>("line.operations");
    if (!click(modeButton(&again, 0))) {
        return fail("reopened model mode", &again);
    }
    QTest::qWait(150);
    if (againTree == nullptr || againTree->topLevelItemCount() < 1) {
        return fail("reopened operation tree is empty", &again);
    }
    logStep("ok");
    return 0;
}
