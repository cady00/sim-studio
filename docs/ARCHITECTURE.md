# 架构

Sim Studio 按库拆开，后续阶段只替换或填充已有接口，不把物理、机器人或 PLC 写进界面类。

```
src/core        场景图、变换、单位、网格、拾取、文案目录
src/io          STEP / OBJ、.simproj
src/render      轨道相机与 OpenGL 视口
src/app         Qt 主窗口、导入线程、命令行
src/physics     IPhysicsEngine，当前实现是 Bullet 3
src/scripting   IScriptHost，当前实现是嵌入的 CPython + pybind11
```

内部单位是毫米，上方向是 +Z，与 NX / Process Simulate 一致。变换是平移 + 四元数 + 缩放，矩阵列主序。属性面板的欧拉角是内旋 XYZ（与 three.js 分量公式一致）。

## 场景图

`Scene` 拥有根节点 `World`（id `root`，界面显示「世界」）。子节点用 `shared_ptr`，父节点用 `weak_ptr`。网格按 id 共享，两个引用同一几何的实例只细分一次。

`ChangeKind` 区分结构、变换、选择和仿真。仿真通知刷新视口和信号表，不把项目标成已修改。批量导入时 `beginEdit` / `endEdit` 合并通知。后台线程导入完成后，`steal()` 把那棵场景移到界面场景上，并只发一次结构通知。

每个节点有一份 `Mech`。关节、传感器、发生器和销毁器都挂在节点上，属性面板和 `.simproj` 读写同一份数据。信号表是场景上的命名值（bool / int / real），传感器和脚本写入，界面只读显示。这张表是 Phase 5 PLC / OPC UA 要接的缝，适配器只需要读写 `Scene::signals`，不必碰 Bullet。

## STEP

`StepImporter` 使用 Open CASCADE XCAF：

- `STEPCAFControl_Reader` 读取 AP214 / AP242。
- `BinXCAFDrivers::DefineFormat` 直接注册二进制 XCAF，不加载 Draw，也不依赖 Tcl 插件去打开 STEP。
- 自由形状进入装配树。引用节点的变换只取该次出现的 `GetLocation`。网格在去掉 location 的被引用形状上生成，避免变换乘两次。
- 线性偏差按包围盒对角线的 0.2% 取值，并夹在 0.35–3.0 mm。
- 请求 `xstep.cascade.unit = MM`。若文档长度单位不是毫米，再用 `XCAFDoc_LengthUnit` 换算。
- 出现名为空或 `=>[...]` 时，改用被引用产品名。NX 导出的 KR 6 子件名因此能进树。

资源目录在编译期写入 `SIM_OCCT_RESOURCE_DIR`。运行前若未设置 `CSF_*`，`ensureOcctEnvironment()` 指向该目录。

## 界面线程

`MainWindow::importStepAsync` 使用 `std::thread`，不占用 Qt 主线程。进度经 `QMetaObject::invokeMethod(..., Qt::QueuedConnection)` 回到界面：读取和传输阶段是不确定进度，细分阶段是按零件计数的确定进度。取消标志在零件之间检查。

窗口析构把 `alive_` 置为 false 再 `join`，避免导入回调在窗口销毁后触碰控件。

## 渲染

`ViewportWidget` 是 OpenGL 3.3 Core 的 `QOpenGLWidget`（MSAA 4x）。选择用 CPU 射线与三角形相交（Möller–Trumbore），不读像素。网格法线在 CPU 上由三角形计算。

## 物理

`createPhysicsEngine()` 返回 `BulletPhysicsEngine`。场景单位是毫米，Bullet 用米（×0.001）。重力为 −Z、9.81 m/s²。固定步长 1/60 s。界面计时器约 16 ms 调一次 `Simulation::tick`，乘以速度倍率（0–16）后最多补 240 步。

| 操作 | 行为 |
| --- | --- |
| 运行 | 从停止进入时记下非临时节点的初始局部变换，建立物理世界，调用 `on_start` |
| 暂停 | 世界保留，时间不走 |
| 停止 / 复位 | 调用 `on_stop`，拆掉世界，删除临时节点，恢复初始变换，时间归零 |

没有选 PhysX 5。vcpkg 基线里的 PhysX 端口来自 NVIDIA Omniverse，要编译整套 SDK，而且不是 NVIDIA 支持的发行方式。Ubuntu 24.04 没有可直接安装的 PhysX 包，Linux CI 不能为它再走一遍源码编译。Bullet 在两边都有现成包（发行版 `libbullet-dev`，vcpkg `bullet3`），并且已经藏在 `IPhysicsEngine` 后面，以后可以换实现。

近似，不要当成 NX MCD 的求解器：

- 输送面是运动学刚体。接触时把动态刚体的水平速度设成带速，保留竖直速度。这不是表面材质。
- 移动副的 Bullet 约束轴与作者写的轴相反，建立约束时取了反。转动副保持原轴，限位和电机目标取反，这样正目标对应世界坐标里的正向转角。
- 位置驱动把速度夹在最大速度和加速度之内。移动副电机力固定 4000 N，转动副冲量固定 80，都不是由加速度换算出来的真实力或力矩。
- 光栅和距离传感器是沿局部轴的最近射线。光栅被挡住时布尔信号为真。碰撞传感器是 ghost，与锚点以外的物体重叠即为真。
- 销毁器只删临时刚体，并且用轴对齐盒子判断，忽略销毁器自身的旋转。
- 属性面板的轴是 X/Y/Z 下拉框。保存时输送方向写成所选轴，任意方向向量不能在界面里编辑。

## 脚本

`PythonScriptHost` 进程内初始化一次解释器，不在退出时 `Py_Finalize`（pybind11 嵌入模块不适合反复初始化）。模块名 `simstudio`，脚本里 `import simstudio as sim` 已由宿主注入。回调是 `on_start()`、`on_step(dt)`、`on_stop()`。异常记入脚本日志并停用该回调。

| 函数 | 作用 |
| --- | --- |
| `get_bool` / `set_bool` | 布尔信号。不存在则按该类型创建 |
| `get_int` / `set_int` | 整数信号 |
| `get_real` / `set_real` | 实数信号 |
| `set_joint_target(name, value)` | 位置目标。驱动为「无」时改成位置驱动 |
| `set_joint_speed(name, value)` | 速度驱动，目标即速度 |
| `get_transform(name)` | `(tx, ty, tz, qw, qx, qy, qz)`，毫米 |
| `set_transform(name, x, y, z)` | 只改平移 |
| `spawn(name)` | 按该节点当前位姿克隆，返回新名字 |
| `delete(name)` | 先释放刚体再删节点 |

Windows 安装包把标准库放在 exe 旁的 `Lib/`，并写 `python3xx._pth`。程序在创建解释器之前，若发现这个目录，会设置 `PYTHONHOME`。Linux 使用系统 Python，不改 `PYTHONHOME`。

## 后续阶段怎么接

| 阶段 | 接入点 |
| --- | --- |
| Phase 3 机器人 | 在装配节点上增加关节定义。KR 6 根产品 `394.088-10.104.00` 下的五个子零件 `394.088-10.105.10`、`.104.20`、`.105.30`、`.105.40`、`.104.50` 是待映射的连杆实体。关节轴、零位和限位来自机器人模型，不从 STEP 文件名推断。碰撞查询走现有刚体。 |
| Phase 4 节拍 | 在场景之上加产线时钟和工位，不改 STEP 导入。 |
| Phase 5 虚拟调试 | 独立 I/O 适配器（OPC UA、S7、PLCSIM Advanced）读写场景信号。Python API 继续走 `IScriptHost`。 |

## 依赖策略

- Windows：`vcpkg.json` 清单模式，基线 `2026.06.24`（`cd61e1e26a038e82d6550a3ebbe0fbbfe7da78e3`）。Qt 只开界面所需特性，Open CASCADE 只开 freetype，避免 VTK。三元组 `x64-windows-release` 动态链接。
- Linux 验证与 CI：发行版包（Ubuntu 24.04 上为 Qt 6.4 与 OCCT 7.6）。同一套 `find_package`。从源码编译 Qt 6.11 与 OCCT 8 对当前 CI 不划算，所以 Linux 不走 vcpkg。若 7.6 与 8.0 的网格 API 分叉，导入代码按 `OCC_VERSION_HEX` 分支。

许可证见仓库根目录 `THIRD_PARTY_LICENSES.md`。不要链接 OCCT Draw。不要加入运行时联网或遥测。
