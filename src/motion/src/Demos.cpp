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

}  // namespace sim
