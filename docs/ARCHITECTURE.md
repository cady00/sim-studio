# 架构

Sim Studio 按库拆开，后续阶段只替换或填充已有接口，不把物理、机器人或 PLC 写进界面类。

```
src/core        场景图、变换、单位、网格、拾取、文案目录
src/io          STEP / OBJ、.simproj
src/render      轨道相机与 OpenGL 视口
src/app         Qt 主窗口、导入线程、命令行
src/physics     IPhysicsEngine，目前是空实现
src/scripting   IScriptHost，目前是空实现
```

内部单位是毫米，上方向是 +Z，与 NX / Process Simulate 一致。变换是平移 + 四元数 + 缩放，矩阵列主序。属性面板的欧拉角是内旋 XYZ（与 three.js 分量公式一致）。

## 场景图

`Scene` 拥有根节点 `World`（id `root`，界面显示「世界」）。子节点用 `shared_ptr`，父节点用 `weak_ptr`。网格按 id 共享，两个引用同一几何的实例只细分一次。

`ChangeKind` 区分结构、变换和选择。批量导入时 `beginEdit` / `endEdit` 合并通知。后台线程导入完成后，`steal()` 把那棵场景移到界面场景上，并只发一次结构通知。

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

## 后续阶段怎么接

| 阶段 | 接入点 |
| --- | --- |
| Phase 2 物理 | 实现 `IPhysicsEngine`，由 `createPhysicsEngine()` 返回。场景节点已有变换和网格，可生成刚体和输送线。界面「物理引擎」对话框只展示 `backendName()`。 |
| Phase 2 脚本 | 实现 `IScriptHost`，嵌入 pybind11。脚本看到的是场景图和物理句柄，不直接碰 Qt。 |
| Phase 3 机器人 | 在装配节点上增加关节定义。KR 6 根产品 `394.088-10.104.00` 下的五个子零件是待映射的连杆实体。关节轴、零位和限位来自机器人模型，不从 STEP 文件名推断。碰撞查询走物理或专用网格。 |
| Phase 4 节拍 | 在场景之上加产线时钟和工位，不改 STEP 导入。 |
| Phase 5 虚拟调试 | 独立 I/O 适配器（OPC UA、S7、PLCSIM Advanced）驱动场景信号。Python API 复用 `IScriptHost`。 |

## 依赖策略

- Windows：`vcpkg.json` 清单模式，基线 `2026.06.24`（`cd61e1e26a038e82d6550a3ebbe0fbbfe7da78e3`）。Qt 只开界面所需特性，Open CASCADE 只开 freetype，避免 VTK。三元组 `x64-windows-release` 动态链接。
- Linux 验证与 CI：发行版包（Ubuntu 24.04 上为 Qt 6.4 与 OCCT 7.6）。同一套 `find_package`。从源码编译 Qt 6.11 与 OCCT 8 对当前 CI 不划算，所以 Linux 不走 vcpkg。若 7.6 与 8.0 的网格 API 分叉，导入代码按 `OCC_VERSION_HEX` 分支。

许可证见仓库根目录 `THIRD_PARTY_LICENSES.md`。不要链接 OCCT Draw。不要加入运行时联网或遥测。
