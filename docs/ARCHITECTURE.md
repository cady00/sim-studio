# 架构

Sim Studio 按库拆开。后续阶段只替换或填充已有接口，不把物理、机器人、PLC 或模型服务写进界面类。

```
src/core        场景图、变换、单位、网格、拾取、文案目录、机构字段
src/io          STEP / OBJ、.simproj
src/render      IRenderer；当前后端是 OpenGL 3.3
src/physics     IPhysicsEngine；当前后端是 Jolt。IRobotSolver 只留接口
src/scripting   IScriptHost，嵌入的 CPython + pybind11
src/ai          命令目录与 AI 提供者接口。默认关闭，本版本不联网
src/app         Qt 主窗口、导入线程、命令行
```

内部单位是毫米，上方向是 +Z。变换是平移 + 四元数 + 缩放，矩阵列主序。

## 逻辑只通过信号交换

这是以后加远程执行时不再改结构的规则。

逻辑块和脚本之间不互相调用，也不读取对方的内部状态。它们交换的数据只有场景上的信号表（bool / int / real）。一个块的输出信号就是另一个块的输入信号。Python 的 `get_*` / `set_*` 和命令层的同名操作都写这张表。

因此非实时逻辑可以换执行地点，而不改块的定义：

- 调度、批量仿真、AI 生成的逻辑，以后可以由一台服务器对着信号表运行。那台机器不需要刚体世界。
- 实时物理、关节驱动、输送面和 PLC 同步留在拥有场景的这台机器上。本地侧读取信号、驱动执行器，再把传感器写回信号表。

现在没有远程后端，也没有第二条执行路径。规则先定下来，避免逻辑块以后直接拿别的块的对象指针。

## 场景图

`Scene` 拥有根节点 `World`（id `root`）。子节点用 `shared_ptr`，父节点用 `weak_ptr`。网格按 id 共享。

`ChangeKind` 区分结构、变换、选择和仿真。仿真通知刷新视口和信号表，不把项目标成已修改。

每个节点有一份 `Mech`。关节、传感器、发生器和销毁器都挂在节点上。信号表是 Phase 5 PLC 要接的缝。OPC UA 适配器只读写 `Scene` 上的信号，不进入物理求解器。OPC UA 是计划中的主 PLC 协议；S7、PLCSIM Advanced 如果要做，也是同一张表的别的适配器，不替代 OPC UA 的位置。

## 渲染

比较过三个后端，当前只实现 OpenGL。

| 方案 | 许可证 | 构建 | 结论 |
| --- | --- | --- | --- |
| bgfx | BSD-2-Clause，可私有商用 | vcpkg 有 `bgfx` | 不采用。它要自己的着色器工具链，并且习惯自己管窗口和上下文，和 `QOpenGLWidget` 抢上下文。Windows CI 也不适合再编一套 shaderc。 |
| Diligent Engine | Apache-2.0，可私有商用 | 本基线没有官方 vcpkg 端口，需要整棵 vendored CMake（glslang、SPIRV-Cross） | 不采用。同样要和 Qt 的 OpenGL 上下文共存，依赖体积大。 |
| 自有薄接口 `IRenderer` | 仓库自己的代码 | 不增加第三方库 | 采用。OpenGL 3.3 是现在的后端。Vulkan 或 Direct3D 12 以后实现同一个类，视口的输入和相机不用改。 |

`IRenderer` 负责：上传网格、提交实例、画线框与 gizmo、读回 RGBA。拾取默认仍走 CPU 射线，接口留了 `pick` 以便以后改成 GPU。`selectLodLevel` 按距离选细节级别，当前每个零件只有一级网格，钩子先在提交路径里调用。`intersectsClipFrustum` 在提交前丢掉视锥外的包围盒。同一网格的多个实例一次 `glDrawElementsInstanced` 画完，输送线上重复的工件走这条路径。

离屏读回是 `captureRgba`。截图时视口先走这个接口，失败再退回 Qt 的 `grabFramebuffer`。

## 物理

`createPhysicsEngine()` 返回 `JoltPhysicsEngine`。场景是毫米，Jolt 用米（×0.001）。重力为 −Z、9.81 m/s²。固定步长 1/60 s。播放、暂停、停止的语义与以前相同：运行时记下静止位姿，停止时删掉临时刚体并恢复位姿。

Jolt 5.5（vcpkg 端口 `joltphysics`；Linux 用同一 git 标签 FetchContent 编译，因为 Ubuntu 24.04 没有对等的包）。许可证是 MIT，允许私有商用。没有链接 PhysX。

关节用 Jolt 自己的约束，不再为了 Bullet 的轴向做取反：

- 移动副：较重的一侧是 body 1，运动件是 body 2。正方向就是作者写的轴。限位相对播放开始时的位姿，区间必须包含 0（Jolt 的要求）；若下限大于 0 会把下限收到 0。
- 转动副：同样的 body 顺序。正角速度使转角沿轴的右手方向增加。
- 电机是速度电机。位置驱动先按最大速度和加速度算出这一步的速度，再交给电机。力上限 1e6 N，力矩上限 500 N·m，用来让速度指令在碰到限位或碰撞前被跟上，不是从加速度换算的真实出力。
- 输送面是运动学刚体。接触发生在求解过程中被记下来，下一步开始时把动态刚体的水平速度设成带速。这比接触回调里直接改速度晚一帧，不是皮带材质。
- 光栅和距离传感器是射线，碰撞传感器是形状重叠查询，所以运动学夹具也能被碰到。
- 凸包、球、胶囊、盒子都有。胶囊在 Jolt 里沿 Y，创建时转到 Z，与以前的形状朝向一致。
- 销毁器仍是轴对齐盒子，忽略自身旋转。

`IRobotSolver` 只有接口，没有实现。六轴机器人不叠在这些通用铰链上。Phase 3 的求解器写关节角，需要碰撞时再通过 `adoptSpawned` 把刚体放进 Jolt。

## 用户定义的设备，而不是预制组件

Phase 3 不提供转台、滑台、辊床、升降机、堆垛机这类做好的组件。这些设备由用户按 PDPS 的方式搭：

- 运动学编辑器：连杆、关节、姿态。数据仍落在场景节点和 `Mech` 上，不另起一套只有界面才认识的对象。
- 逻辑块：输入、输出、参数、动作、条件。块与块之间只走信号，见上面的规则。

通用移动副、转动副和固定副继续给这些用户定义的机构用。机器人连杆映射是另一条路，走 `IRobotSolver`。

## 脚本与命令

`PythonScriptHost` 进程内初始化一次解释器。模块名 `simstudio`。回调是 `on_start()`、`on_step(dt)`、`on_stop()`。

`src/ai` 的 `CommandExecutor` 用 JSON 对象调用同一组操作：`{"op":"set_bool","name":"go","value":true}`。目录由 `CommandExecutor::catalog()` 给出，字段说明在 `docs/ai-commands.schema.json`。已实现的操作覆盖信号、关节目标、位姿、生成、删除、列出节点和仿真播放控制。自然语言生成逻辑、布局建议、节拍优化、异常解释在目录里，调用时返回未实现，不改场景。

## AI 提供者

`IAiProvider` 是可替换的模型接口。默认配置是关闭的。`createAiProvider()` 返回的实现 `usesNetwork()` 恒为 false：即使填了本地 OpenAI 兼容地址（Ollama、llama.cpp 的 `/v1/chat/completions`），这个二进制也不会打开套接字。传输层要等以后单独打开编译开关。云厂商是同一种接口的另一种 `kind`，现在没有。

没有遥测，运行时不访问网络。

## STEP 与 OpenUSD

`StepImporter` 仍用 Open CASCADE XCAF 读 AP214 / AP242。网格在引用形状上生成，变换只取该次出现的 location。

OpenUSD 现在不导入也不导出。以后如果要接，放在 `src/io` 的另一个导入器里，把 USD 的 prim 层级写成现有场景节点和共享网格，单位换成毫米。不把 USD 的舞台对象留在场景图里，这样 `.simproj` 仍然只存自己的 JSON 和网格。导出时从场景图写 USD，而不是从 OCCT 文档转。

## 界面线程

`MainWindow::importStepAsync` 使用 `std::thread`。窗口析构把 `alive_` 置为 false 再 `join`。

## 依赖与 Windows 构建

- Windows：`vcpkg.json` 清单模式，基线 `2026.06.24`。Qt 不进清单。CI 用 `jurplel/install-qt-action` 安装预编译的 Qt 6.8.3（MSVC 2022），避免在 vcpkg 里从源码编 Qt。Open CASCADE、Jolt、CPython、pybind11、Catch2、nlohmann-json 仍走 vcpkg，二进制缓存在 `x-gha`。OCCT 8 的 STEP 库是 `TKDESTEP`，CMake 只链接本机存在的 toolkit，因此 Linux 的 OCCT 7.6（`TKSTEP*`）也能链上。
- Linux CI：发行版 Qt 6.4 与 OCCT 7.6。Jolt 在配置时按标签 v5.5.0 拉取并编译。

许可证见 `THIRD_PARTY_LICENSES.md`。不要链接 OCCT Draw。

## Future-proofing & moats

目标是五年后仍能换渲染、物理和模型，而项目、节拍和调试数据还在。壁垒放在自己写的运行时上，不放在某一家 SDK 上。这一版只把缝留对，不提前做规划器、事件日志或无头批处理程序。

**AI 原生。** 智能体只走 `CommandExecutor` 的带类型命令（`docs/ai-commands.schema.json`）。Python `simstudio` 是同一组操作的脚本入口，不是第二套语义。以后的 MCP 工具只是这张目录的传输，不新写一套能改场景的函数。模型调用走 `IAiProvider`，默认关闭，本二进制不联网。自然语言生成逻辑、布局、节拍和异常解释在目录里占位，现在返回未实现。

仿真要能被智能体重放和打分。现在已经成立的是固定步长 `Simulation::kFixedDt`（1/60 s）、播放时记下的静止位姿，以及停止时恢复。还没有随机种子和事件日志。下一步只在 `Simulation` 上加种子和有序事件（信号写入、生成、删除），不改命令的含义。无头批跑是同一 `Simulation` 加上命令层、不创建 Qt；测试里已经这样调用，单独的批处理入口以后再做。

**数据模型。** `.simproj` 的格式版本是 `kProjectFormatVersion`。场景、机构字段、信号和脚本是版本化的 JSON，网格是旁边的二进制。渲染后端和物理后端都不进文件。OpenUSD 只作为进出适配器：读进来写成这套节点，写出去从这套节点生成。舞台对象不留在内存里，避免项目被绑在某次 USD 或 OCCT 的文档上。

**可替换后端。**

| 缝 | 现在 | 以后换什么 |
| --- | --- | --- |
| 渲染 | `IRenderer`，OpenGL 3.3 | Vulkan / Direct3D 12，同一提交接口 |
| 刚体 | `IPhysicsEngine`，Jolt | 多体或 GPU 刚体，场景 API 不动 |
| 机器人运动 | `IRobotSolver` 空接口 | 自有规划器；RRS/RCS 只做外部控制器适配 |
| PLC | 信号表 | OPC UA 为主；PLCSIM Advanced API 是另一个适配器 |
| 计算 | 本机进程 | 非实时逻辑可远程跑信号表；物理、执行器和 PLC 同步留在本机 |

逻辑块和脚本之间仍然只通过信号交换，这样远程逻辑后端不用拆块的内部。

**自己掌握、不外包的部分。** 运动学与逻辑块引擎、自有运动规划和节拍模型、信号与虚拟调试运行时、确定性仿真核、AI 命令层以及在上面积累的产线数据。Jolt、OpenGL、OCCT、Qt、某个本地模型都是可替换的实现。RRS/RCS 和 PLCSIM Advanced 是要对接的外部接口，不是要变成产品核心的库。
