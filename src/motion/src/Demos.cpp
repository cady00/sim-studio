#include "sim/motion/Motion.hpp"

#include "sim/core/Mesh.hpp"

namespace sim {
namespace {

std::shared_ptr<SceneNode> addBox(Scene& scene, const std::shared_ptr<SceneNode>& parent, const std::string& name,
                                  Vec3 halfExtents, Vec3 color, Vec3 localTranslation) {
    auto node = scene.makeNode(NodeKind::Part, name);
    node->meshId = scene.addMesh(makeBoxMesh(halfExtents));
    node->color = color;
    node->local.translation = localTranslation;
    scene.addChild(parent, node);
    return node;
}

}  // namespace

void buildTurntableLiftDemo(Scene& scene) {
    scene.clear();
    scene.name = "Turntable and lift";
    auto& resource = ensureResource(scene, "turntable_lift");
    const std::string baseId = addLink(scene, resource, "base", "", {}).nodeId;
    Transform tableLocal;
    tableLocal.translation = {0, 0, 40};
    const std::string tableId = addLink(scene, resource, "table", "base", tableLocal).nodeId;
    Transform liftLocal;
    liftLocal.translation = {0, 0, 24};
    const std::string carriageId = addLink(scene, resource, "carriage", "table", liftLocal).nodeId;

    auto baseNode = scene.find(baseId);
    auto tableNode = scene.find(tableId);
    auto carriageNode = scene.find(carriageId);
    addBox(scene, baseNode, "Base", {220, 220, 20}, {0.42, 0.46, 0.52}, {0, 0, 20});
    addBox(scene, tableNode, "Table", {170, 170, 12}, {0.25, 0.48, 0.86}, {0, 0, 12});
    addBox(scene, tableNode, "TableMark", {28, 70, 8}, {0.95, 0.82, 0.22}, {90, 0, 28});
    addBox(scene, carriageNode, "Lift", {36, 36, 90}, {0.90, 0.52, 0.18}, {0, 0, 90});

    KinJoint& spin = addJoint(resource, scene, "spin", "base", "table", JointMotion::Revolute, {0, 0, 1}, {0, 0, 40});
    spin.lower = -180;
    spin.upper = 180;
    spin.maxSpeed = 120;
    spin.maxAccel = 480;
    KinJoint& lift = addJoint(resource, scene, "lift", "table", "carriage", JointMotion::Prismatic, {0, 0, 1}, {0, 0, 24});
    lift.lower = 0;
    lift.upper = 220;
    lift.maxSpeed = 280;
    lift.maxAccel = 900;

    resource.joints[0].position = 0;
    resource.joints[1].position = 0;
    capturePose(resource, "home");
    resource.joints[0].position = 90;
    resource.joints[1].position = 180;
    capturePose(resource, "work");
    resource.joints[0].position = 0;
    resource.joints[1].position = 0;
    applyResource(scene, resource);

    LogicBlock block;
    block.name = "cycle";
    block.resource = resource.name;
    block.runSignal = "cycle";
    block.inputs = {"cycle"};
    block.outputs = {"done"};
    block.python =
        "# Optional. This text is not executed by the block runner.\n"
        "# def on_step(dt):\n"
        "#     simstudio.set_bool(\"done\", True)\n";
    LogicParam dwell;
    dwell.name = "dwell";
    dwell.type = "real";
    dwell.real = 0.25;
    block.parameters.push_back(dwell);
    LogicStep toWork;
    LogicAction moveWork;
    moveWork.kind = "move_pose";
    moveWork.pose = "work";
    moveWork.profile = "trapezoid";
    toWork.actions.push_back(moveWork);
    LogicStep pause;
    LogicAction wait;
    wait.kind = "wait";
    wait.seconds = 0.25;
    pause.actions.push_back(wait);
    LogicStep toHome;
    LogicAction moveHome;
    moveHome.kind = "move_pose";
    moveHome.pose = "home";
    moveHome.profile = "scurve";
    toHome.actions.push_back(moveHome);
    LogicStep finish;
    LogicAction done;
    done.kind = "set_output";
    done.signal = "done";
    finish.actions.push_back(done);
    block.steps = {toWork, pause, toHome, finish};
    scene.logicBlocks.push_back(std::move(block));
    scene.upsertSignal("cycle", SignalType::Bool).flag = true;
    scene.upsertSignal("done", SignalType::Bool).flag = false;
}

void buildScaraDemo(Scene& scene) {
    // Joint ranges follow the public KR SCARA-2 family (KR 6 R500 Z200-2 and
    // KR 6 R700 Z200-2): A1 ±132°, A2 ±145°, A3 -200..0 mm, A4 ±355°,
    // rated speeds A1 420°/s, A2 720°/s, A4 2000°/s.
    // A separate public R600 axis row was not copied. A3 speed is not cited.
    // Geometry is a generic 350+250 mm arm, not the KUKA CAD model.
    scene.clear();
    scene.name = "SCARA pick and place";
    auto& resource = ensureResource(scene, "scara");
    resource.ik = "scara";
    resource.scara.l1 = 350;
    resource.scara.l2 = 250;
    resource.scara.baseZ = 0;

    const std::string baseId = addLink(scene, resource, "base", "", {}).nodeId;
    const std::string armId = addLink(scene, resource, "arm", "base", {}).nodeId;
    Transform forearmLocal;
    forearmLocal.translation = {350, 0, 0};
    const std::string forearmId = addLink(scene, resource, "forearm", "arm", forearmLocal).nodeId;
    Transform quillLocal;
    quillLocal.translation = {250, 0, 0};
    const std::string quillId = addLink(scene, resource, "quill", "forearm", quillLocal).nodeId;
    const std::string toolId = addLink(scene, resource, "tool", "quill", {}).nodeId;

    addBox(scene, scene.find(baseId), "Pedestal", {80, 80, 40}, {0.45, 0.48, 0.55}, {0, 0, 40});
    addBox(scene, scene.find(armId), "UpperArm", {175, 28, 24}, {0.20, 0.55, 0.78}, {175, 0, 80});
    addBox(scene, scene.find(forearmId), "Forearm", {125, 22, 18}, {0.16, 0.36, 0.62}, {125, 0, 80});
    addBox(scene, scene.find(quillId), "Quill", {16, 16, 70}, {0.75, 0.78, 0.84}, {0, 0, -40});
    addBox(scene, scene.find(toolId), "Gripper", {22, 10, 16}, {0.92, 0.78, 0.20}, {30, 0, -90});
    addBox(scene, scene.root(), "PickPart", {18, 18, 12}, {0.85, 0.35, 0.28}, {480, 120, 12});
    addBox(scene, scene.root(), "PlacePad", {40, 40, 8}, {0.35, 0.62, 0.38}, {300, -180, 8});

    KinJoint& j1 = addJoint(resource, scene, "A1", "base", "arm", JointMotion::Revolute, {0, 0, 1}, {0, 0, 0});
    j1.lower = -132;
    j1.upper = 132;
    j1.maxSpeed = 420;
    j1.maxAccel = 1800;
    KinJoint& j2 = addJoint(resource, scene, "A2", "arm", "forearm", JointMotion::Revolute, {0, 0, 1}, {350, 0, 0});
    j2.lower = -145;
    j2.upper = 145;
    j2.maxSpeed = 720;
    j2.maxAccel = 2400;
    KinJoint& j3 = addJoint(resource, scene, "A3", "forearm", "quill", JointMotion::Prismatic, {0, 0, 1}, {250, 0, 0});
    j3.lower = -200;
    j3.upper = 0;
    j3.maxSpeed = 500;
    j3.maxAccel = 2500;
    KinJoint& j4 = addJoint(resource, scene, "A4", "quill", "tool", JointMotion::Revolute, {0, 0, 1}, {0, 0, 0});
    j4.lower = -355;
    j4.upper = 355;
    j4.maxSpeed = 2000;
    j4.maxAccel = 8000;

    NamedFrame baseFrame;
    baseFrame.name = "base";
    resource.frames.push_back(baseFrame);
    NamedFrame tcp;
    tcp.name = "tcp";
    tcp.origin = resource.scara.tcp;
    resource.frames.push_back(tcp);

    const std::vector<double> home{20, 25, -30, 0};
    setJointVector(resource, home);
    applyResource(scene, resource);
    capturePose(resource, "ready");

    RobotPath path;
    path.name = "pick_place";
    path.resource = resource.name;
    PathPoint approach;
    approach.name = "approach";
    approach.move = PathMove::Lin;
    approach.cartesian = true;
    approach.position = {480, 120, -20};
    approach.speed = 250;
    approach.zone = 15;
    PathPoint pick;
    pick.name = "pick";
    pick.move = PathMove::Lin;
    pick.cartesian = true;
    pick.position = {480, 120, -140};
    pick.speed = 150;
    pick.zone = 0;
    PathPoint retreat;
    retreat.name = "retreat";
    retreat.move = PathMove::Lin;
    retreat.cartesian = true;
    retreat.position = {480, 120, -20};
    retreat.speed = 200;
    retreat.zone = 20;
    PathPoint place;
    place.name = "place";
    place.move = PathMove::Circ;
    place.cartesian = true;
    place.position = {300, -180, -30};
    place.via = {520, -40, -20};
    place.speed = 200;
    place.zone = 0;
    path.points = {approach, pick, retreat, place};
    scene.robotPaths.push_back(std::move(path));
    setJointVector(resource, home);
    applyResource(scene, resource);
}

void buildCollisionDemo(Scene& scene) {
    scene.clear();
    scene.name = "Collision sets";
    auto block = [&](const std::string& name, Vec3 half, Vec3 color, Vec3 at) {
        auto node = scene.makeNode(NodeKind::Part, name);
        node->meshId = scene.addMesh(makeBoxMesh(half));
        node->color = color;
        node->local.translation = at;
        scene.addChild(scene.root(), node);
    };
    const Vec3 gray{0.62, 0.64, 0.68};
    block("OverlapA", {30, 30, 30}, gray, {0, 0, 30});
    block("OverlapB", {30, 30, 30}, gray, {50, 0, 30});
    block("ClearA", {30, 30, 30}, gray, {0, 160, 30});
    block("ClearB", {30, 30, 30}, gray, {68, 160, 30});
    scene.proximity.clearanceMm = 15;
    CollisionSet movers;
    movers.name = "movers";
    movers.objects = {"OverlapA", "ClearA"};
    CollisionSet fixtures;
    fixtures.name = "fixtures";
    fixtures.objects = {"OverlapB", "ClearB"};
    scene.proximity.sets = {std::move(movers), std::move(fixtures)};
    scene.proximity.pairs.emplace_back("movers", "fixtures");
}

void buildEditorDevices(Scene& scene) {
    scene.clear();
    scene.name = "editor_devices";
    auto part = [&](const std::string& name, Vec3 half, Vec3 color, Vec3 at) {
        auto node = scene.makeNode(NodeKind::Part, name);
        node->meshId = scene.addMesh(makeBoxMesh(half));
        node->color = color;
        node->local.translation = at;
        scene.addChild(scene.root(), node);
        return node->id;
    };
    std::string error;

    auto& turntable = ensureResource(scene, "turntable");
    addLink(scene, turntable, "base", "", {});
    Transform platen;
    platen.translation = {0, 0, 40};
    addLink(scene, turntable, "platen", "base", platen);
    adoptParts(scene, turntable, "base", {part("TurntableBase", {180, 180, 16}, {0.42, 0.46, 0.52}, {0, 0, 16})}, error);
    adoptParts(scene, turntable, "platen", {part("TurntableTop", {140, 140, 10}, {0.25, 0.48, 0.86}, {0, 0, 50})}, error);
    KinJoint& spin = addJoint(turntable, scene, "spin", "base", "platen", JointMotion::Revolute, {0, 0, 1}, {0, 0, 40});
    spin.lower = -180;
    spin.upper = 180;
    spin.maxSpeed = 90;
    capturePose(turntable, "home");
    spin.position = 90;
    capturePose(turntable, "quarter");
    spin.position = 0;
    applyResource(scene, turntable);

    auto& bed = ensureResource(scene, "rollerbed");
    addLink(scene, bed, "frame", "", {});
    Transform beltLocal;
    beltLocal.translation = {0, 1400, 80};
    addLink(scene, bed, "belt", "frame", beltLocal);
    Transform rollerLocal;
    rollerLocal.translation = {-400, 1400, 110};
    addLink(scene, bed, "roller", "frame", rollerLocal);
    adoptParts(scene, bed, "frame", {part("BedFrame", {700, 80, 20}, {0.35, 0.38, 0.42}, {0, 1400, 40})}, error);
    adoptParts(scene, bed, "belt", {part("BedBelt", {500, 50, 8}, {0.2, 0.22, 0.24}, {0, 1400, 88})}, error);
    adoptParts(scene, bed, "roller", {part("BedRoller", {18, 70, 18}, {0.75, 0.78, 0.82}, {-400, 1400, 110})}, error);
    addJoint(bed, scene, "travel", "frame", "belt", JointMotion::Prismatic, {1, 0, 0}, {0, 1400, 80});
    addJoint(bed, scene, "roll", "frame", "roller", JointMotion::Revolute, {0, 1, 0}, {-400, 1400, 110});
    KinJoint& travel = *findJoint(bed, "travel");
    travel.lower = 0;
    travel.upper = 800;
    travel.maxSpeed = 300;
    KinJoint& roll = *findJoint(bed, "roll");
    roll.coupledTo = "travel";
    roll.coupleRatio = 0.02;
    roll.lower = -3600;
    roll.upper = 3600;
    capturePose(bed, "home");
    travel.position = 400;
    capturePose(bed, "mid");
    travel.position = 0;
    applyResource(scene, bed);

    auto& hoist = ensureResource(scene, "hoist");
    addLink(scene, hoist, "column", "", {});
    Transform carriage;
    carriage.translation = {0, -1400, 200};
    addLink(scene, hoist, "carriage", "column", carriage);
    adoptParts(scene, hoist, "column", {part("HoistColumn", {30, 30, 220}, {0.45, 0.48, 0.52}, {0, -1400, 220})}, error);
    adoptParts(scene, hoist, "carriage", {part("HoistCarriage", {50, 40, 16}, {0.9, 0.55, 0.16}, {0, -1400, 200})}, error);
    KinJoint& rise = addJoint(hoist, scene, "rise", "column", "carriage", JointMotion::Prismatic, {0, 0, 1}, {0, -1400, 200});
    rise.lower = 0;
    rise.upper = 400;
    rise.maxSpeed = 200;
    capturePose(hoist, "down");
    rise.position = 300;
    capturePose(hoist, "up");
    rise.position = 0;
    applyResource(scene, hoist);
}

void buildPublicCell(Scene& scene) {
    scene.clear();
    scene.name = "公开参数焊装示例";
    std::string error;

    KinematicResource& clamp = ensureResource(scene, "clamp");
    addLink(scene, clamp, "frame", "", {});
    Transform jawLocal;
    jawLocal.translation = {0, -700, 400};
    addLink(scene, clamp, "jaw", "frame", jawLocal);
    KinLink* jaw = findLink(clamp, "jaw");
    jaw->hasInertia = true;
    jaw->mass = 8;
    KinJoint& slide = addJoint(clamp, scene, "slide", "frame", "jaw", JointMotion::Prismatic, {0, 1, 0}, jawLocal.translation);
    slide.lower = 0;
    slide.upper = 80;
    slide.maxSpeed = 200;
    slide.effort = 40;
    slide.accelSource = "dynamics";
    slide.maxAccel = 0;
    if (auto frame = scene.find(findLink(clamp, "frame")->nodeId)) {
        addBox(scene, frame, "ClampFrame", {40, 30, 80}, {0.45, 0.48, 0.52}, {0, -700, 360});
    }
    if (auto jawNode = scene.find(jaw->nodeId)) {
        addBox(scene, jawNode, "ClampJaw", {24, 16, 20}, {0.85, 0.55, 0.2}, {0, 0, 0});
    }
    clamp.source = "夹具质量 8 kg，推力 40 N，加速度由这两项估算。";
    applyResource(scene, clamp);

    if (!addCatalogRobot(scene, "IRB120", {0, 0, 0}, error)) {
        scene.name += " (" + error + ")";
    }
    if (KinematicResource* robot = findResource(scene.resources, "IRB120")) {
        robot->payload = 3;
        robot->source += "\n额定负载 3 kg（ABB IRB 120 公开样本）。关节扭矩未提供，加速度只受速度限制。";
        applyResource(scene, *robot);
    }

    Station load;
    load.id = "st_load";
    load.name = "上件";
    load.x = -1600;
    load.kind = StationKind::Manual;
    Station weldStation;
    weldStation.id = "st_weld";
    weldStation.name = "焊接";
    weldStation.x = 0;
    weldStation.kind = StationKind::Robot;
    weldStation.resource = "IRB120";
    Station unload;
    unload.id = "st_out";
    unload.name = "下件";
    unload.x = 1600;
    unload.kind = StationKind::Manual;
    scene.layout.title = "公开参数焊装示例";
    scene.layout.example = true;
    scene.layout.stations = {load, weldStation, unload};

    const Vec3 spots[] = {{420, 40, 280}, {480, -30, 260}, {390, 90, 300}};
    for (int i = 0; i < 3; ++i) {
        WeldSpot spot;
        spot.id = "w" + std::to_string(i + 1);
        spot.name = "焊点" + std::to_string(i + 1);
        spot.position = spots[i];
        spot.normal = {0, 0, 1};
        spot.example = true;
        spot.electrode = "IRB120";
        scene.welds.push_back(spot);
    }

    RobotPath path;
    path.name = "weld";
    path.resource = "IRB120";
    PathPoint home;
    home.name = "home";
    home.move = PathMove::Ptp;
    home.joints = {0, 0, 0, 0, 0, 0};
    PathPoint approach;
    approach.name = "approach";
    approach.move = PathMove::Ptp;
    approach.joints = {15, -25, 20, 10, 25, 0};
    approach.speed = 200;
    path.points = {home, approach};
    scene.robotPaths.push_back(path);
    if (KinematicResource* robot = findResource(scene.resources, "IRB120")) {
        const MotionPlan plan = planRobotPath(scene, *robot, scene.robotPaths.back());
        if (plan.seconds > 0) {
            scene.robotPaths.back().points.back().speed = 200;
        }
    }

    scene.upsertSignal("part_seen", SignalType::Bool).flag = true;
    scene.upsertSignal("weld_done", SignalType::Bool).flag = false;
    scene.upsertSignal("clamp", SignalType::Bool).flag = false;
    LogicBlock block;
    block.name = "clamp";
    block.resource = "clamp";
    block.runSignal = "part_seen";
    block.inputs = {"part_seen"};
    block.outputs = {"clamp"};
    LogicStep step;
    LogicAction action;
    action.kind = "set_output";
    action.signal = "clamp";
    step.actions.push_back(action);
    block.steps.push_back(step);
    scene.logicBlocks.push_back(block);

    auto task = [](const std::string& id, const std::string& name, const std::string& station, double seconds, int index) {
        Operation operation;
        operation.id = id;
        operation.name = name;
        operation.station = station;
        operation.kind = OpKind::Task;
        operation.seconds = seconds;
        operation.index = index;
        return operation;
    };
    Operation loadOp = task("load", "上件", "st_load", 2, 0);
    Operation weldOp = task("weld", "焊接", "st_weld", 4, 1);
    weldOp.after = {"load"};
    weldOp.path = "weld";
    weldOp.logic = "clamp";
    if (KinematicResource* robot = findResource(scene.resources, "IRB120")) {
        const MotionPlan plan = planRobotPath(scene, *robot, scene.robotPaths.front());
        if (plan.ok && plan.seconds > 0) {
            weldOp.seconds = plan.seconds;
            weldOp.timeSource = "plan";
            weldOp.timeNote = "IRB120 路径";
        }
    }
    Operation outOp = task("unload", "下件", "st_out", 2, 2);
    scene.operations = {loadOp, weldOp, outOp};

    SequenceEvent transition;
    transition.id = "tr_weld_out";
    transition.name = "焊接到下件";
    transition.kind = "transition";
    transition.operation = "weld";
    transition.target = "unload";
    transition.delay = 0.4;
    SequenceEvent setDone;
    setDone.id = "sig_done";
    setDone.name = "weld_done";
    setDone.kind = "signal";
    setDone.operation = "weld";
    setDone.edge = "finish";
    setDone.signal = "weld_done";
    setDone.action = "set";
    setDone.flag = true;
    SequenceEvent setSeen;
    setSeen.id = "sig_seen";
    setSeen.name = "part_seen";
    setSeen.kind = "signal";
    setSeen.operation = "load";
    setSeen.edge = "start";
    setSeen.signal = "part_seen";
    setSeen.action = "set";
    SequenceEvent waitDone;
    waitDone.id = "wait_done";
    waitDone.name = "下件等待 weld_done";
    waitDone.kind = "signal";
    waitDone.operation = "unload";
    waitDone.edge = "start";
    waitDone.signal = "weld_done";
    waitDone.action = "wait";
    scene.sequence = {transition, setDone, setSeen, waitDone};
}

}  // namespace sim
