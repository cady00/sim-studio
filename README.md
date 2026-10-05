# 应点仿真（Ondot Sim）

应点仿真是应点 Ondot 品牌系列中的产线仿真软件，英文名 Ondot Sim。它是一套私有、可离线使用的 Windows 桌面程序。长期目标对齐 Siemens Tecnomatix Process Simulate / Process Designer 与 NX Mechatronics Concept Designer：机器人运动、碰撞、节拍、机电物理、PLC 虚拟调试，以及类似 MELSOFT Gemini / Visual Components 的 Python 脚本。

当前仓库是 **0.4**：场景图、三维视口、STEP 装配导入、Jolt 刚体与关节、信号表、嵌入的 Python 脚本、用户定义的运动学和逻辑块、SCARA / 球腕六轴路径，以及默认关闭的命令 / AI 接口。没有遥测，运行时不访问网络。请保持仓库私有。

## 现在能做什么

- 深色工程界面：三维视口、装配树、属性面板、工具栏、状态栏。
- 默认简体中文，可切换 English。语言存在本机 `QSettings`，不上传。
- 视口操作（Z 轴向上，单位毫米）：
  - 中键拖动：旋转
  - Shift + 中键：平移
  - 滚轮：缩放
  - 左键：选择零件
  - 工具栏「适应全部」和前/后/左/右/俯/仰/等轴测
  - 「移动」「旋转」操作器，或在属性面板里改 X/Y/Z（mm）和 RX/RY/RZ（度）
- 导入 STEP（`.stp` / `.step`），装配层级进入左侧树，曲面细分后显示。
- 导入 OBJ。
- 保存 / 打开 `.simproj`（JSON + 显示网格缓存，格式版本 6，仍能打开版本 1 到 5）。说明见 [docs/PROJECT_FORMAT.md](docs/PROJECT_FORMAT.md)。
- 示例装配 `samples/step/sample_fixture.step`（底板 + 两个共用网格的销）。
- 示例产线 `samples/conveyor_line.simproj`：发生器、输送面、光栅、气动式移动副推杆、销毁器，由 Python 脚本 `pusher` 驱动。脚本副本在 `samples/scripts/`。
- 工具栏：运行、暂停、停止、复位，速度倍率（0–16），仿真时间。复位与停止相同，恢复按下运行前的变换并删掉运行中生成的零件。
- 装配树和属性面板可改机构类型（刚体、碰撞体、固定副、转动副、移动副、输送面、碰撞/距离传感器、光栅、发生器、销毁器）。底部是信号表和脚本编辑器。
- 右侧「运动学 / 逻辑 / 机器人」：实体列表、「建议分组」、Ctrl 多选后「选中零件加入连杆」、可切换的「圆柱面定轴」，以及用数字或两次左键拾取定轴。记录姿态并按梯形或 S 曲线走过去；逻辑块只通过信号驱动这些姿态；机器人页规划 PTP / LIN / CIRC。没有预制的转台或堆垛机。
- 文件菜单「机器人库」浏览本机 Process Simulate `.cojt` 和嵌套的 `.cojt.zip`，用 `preview.bmp` 做预览，导入后把运动学挂进工程。库路径存在本机设置里，不进 `.simproj`。说明见 [docs/manual/README.md](docs/manual/README.md) 第 8 章。
- 虚拟调试使用内置软 PLC：与/或（支路里可再或一层）、上升沿和下降沿、CTU/CTD、TON/TOF、强制和 I/O 映射。导入机器人会追加未接线的 `Rn_AtHome`、`Rn_Busy`、`Rn_Fault`、`Rn_Start`、`Rn_Grip` 和外部轴 `En_InPos`。`--demo commission` 里光电联锁输送，并让 `R1_Start` 驱动一台两轴机器人；本机若有 KR420 或 KR280 的 `.cojt`，A1 上挂的是该 JT 网格，不再只用色块手臂。手臂进入包围盒干涉区 `Z1` 后软 PLC 拉低启动，甘特图按资源分行：输送在光电锁存期间为忙，气缸画出伸出过程，机器人画出取放和干涉阻塞，瓶颈标在最忙的那一行。网格碰撞可写成 `R1_Collision`。`--demo commission-force` 强制 `R1_Start`，光电没有件时手臂仍会动。欢迎页「虚拟调试演示」打开 commission 工程。`--demo mcd-compat` 把光电、气缸和吸盘接到同一张软 PLC，和 MCD 装置的对应写在 `docs/MCD_COMPAT.md`。详见手册第 9 章。
- 内置演示（不写入仓库）：`--demo turntable` 是用户定义的转台加升降，由逻辑块驱动；`--demo scara` 是通用 SCARA 抓放路径；`--demo collision` 是两组盒子，一组重叠（红），一组间隙小于近距阈值（黄）；`--demo kr420` 在本机有 `testdata/local/KR420_R3080_KR280_R3080.stp` 时导入这台扁平 STEP，建议 6 根转轴并走一段 PTP/LIN；`--demo kr420track` 把同一台手臂挂到用户建的约 10 m 地轨上，走一条比臂展更长的直线。地轨不是预制组件，见 [docs/EXTERNAL.md](docs/EXTERNAL.md)。底部「碰撞 / 近距」可改碰撞集和毫米阈值。这些 STEP 不进 git。KR 6 的装配 STEP 仍只显示成树，不自动猜轴。KR 420 的分组方法和样本来源见 [docs/KR420.md](docs/KR420.md)。

导入大 STEP 时，读取和细分在后台线程进行。状态栏显示阶段进度，可点「取消」。取消发生在零件之间；Open CASCADE 正在读文件或传输装配时，取消要等到该阶段结束。界面事件循环保持可响应。

## 五阶段路线

1. **Phase 1** 场景、视口、STEP 装配显示、项目文件、中文界面。
2. **Phase 2** Jolt 刚体、关节、传感器、输送线、信号表，以及嵌入的 Python。渲染走 `IRenderer`（现在是 OpenGL）。AI 提供者默认关闭，这个二进制不联网。
3. **当前仓库** 用户定义的运动学和逻辑块，SCARA 与球腕六轴的正解/逆解，外部轴（地轨、变位机，`E1`…`E6`），扁平 STEP 的连杆建议，PTP/LIN/CIRC，以及可更换后端的近距查询（见 [docs/COLLISION.md](docs/COLLISION.md)、[docs/KR420.md](docs/KR420.md) 和 [docs/EXTERNAL.md](docs/EXTERNAL.md)）。KUKA KR 6 R600 Z200 X 的 STEP 子零件 `394.088-10.105.10`、`.104.20`、`.105.30`、`.105.40`、`.104.50`（根产品 `394.088-10.104.00`）仍只显示成装配树，不自动猜关节轴。KR 420 的扁平实体会建议分组，用户可以改。演示机械臂的限位来源写在 `docs/ARCHITECTURE.md` 和 `docs/KR420.md`。
4. **Phase 4** 产线布局与节拍。
5. **Phase 5** 虚拟调试：OPC UA、S7、PLCSIM Advanced。它们读写现在这张信号表。

模块边界见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 在 Windows 上编译（第一次）

需要 Visual Studio 2022（含「使用 C++ 的桌面开发」）、CMake 3.24+、Git，以及 [vcpkg](https://github.com/microsoft/vcpkg)。第一次会从源码编译 Qt 与 Open CASCADE，耗时很长，请留足磁盘（约数十 GB）并保持机器插电。

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat -disableMetrics
$env:VCPKG_ROOT = "C:\vcpkg"
$env:VCPKG_DISABLE_METRICS = "1"

cd <本仓库>
cmake --preset windows-msvc
cmake --build --preset windows-msvc-release
ctest --preset windows-msvc-release --output-on-failure
```

程序在 `out\build\windows-msvc\src\app\Release\simstudio.exe`。vcpkg 使用清单模式和 `triplets\x64-windows-release.cmake`（动态 CRT / 动态库、仅 Release）。清单里有 Open CASCADE、Jolt、CPython 和 pybind11。Qt 用本机已安装的 Qt 6（CI 里是预编译的 6.8），不在 vcpkg 里从源码编译。配置前设置 `QT_ROOT_DIR`。GitHub Actions 的 Windows 任务用 `x-gha` 缓存 vcpkg 二进制。

## 在 Windows 上直接运行（不用自己编译）

Windows 安装包不随每次推送自动编。在 GitHub 的 Actions 页手动运行工作流 `CI`（`workflow_dispatch`）才会编 `OndotSim-win64`，里面是 `OndotSim-<版本>-win64.zip`。这次还没有生成该压缩包：请在本机 Windows 上配置 `QT_ROOT_DIR` 后用 `windows-msvc` 预设编译，或等配额允许时只手动运行一次 `workflow_dispatch`。这台 Linux 环境不能交叉编译出 MSVC 的 win64 包。解压后运行 `OndotSim\simstudio.exe`。快捷方式和文件属性里的显示名是「应点仿真」。压缩包带有 Qt / OCCT / Jolt / Python 的 DLL、`Lib\` 标准库、`python3xx._pth`、`plugins\` 和 `samples\`。不需要安装 Python 或 Visual Studio。私有仓库的 Windows 分钟按两倍计算，所以这条任务不要改回自动触发。上一版自动任务在链 OCCT 8 时找不到 `TKSTEPBase.lib`（7.8 起改为 `TKDESTEP`），配置阶段还从源码编了 Qt。现在只链接存在的 toolkit，Qt 用预编译包。拉取请求只跑 Linux 测试；只改 `docs/` 或 Markdown 不会启动任务。

1. 在该次 Actions 运行的 Artifacts 里下载 `OndotSim-win64` 并解压。
2. 双击 `simstudio.exe`。
3. 文件 → 打开，选择压缩包里的 `samples\conveyor_line.simproj`。
4. 点工具栏「运行」。约 1.2 秒后零件落到输送面上并向 +X 移动；经过光栅时信号 `barrier` 变为真，推杆沿 +Y 伸出。速度倍率默认 x1。
5. 「停止」或「复位」后，零件消失，推杆回到起始位置，时间回到 0。

本机自己打包（需要已经按上面的步骤编译成功，并且在 VS 2022 开发者环境里能找到 `dumpbin`）：

```powershell
powershell -File cmake\package_windows.ps1
```

产物在 `dist\`。

## 在本机试用

1. 启动 `simstudio.exe`。
2. 文件 → 导入 STEP，选择 `samples\step\sample_fixture.step`。
3. 左侧应出现装配 `Fixture`，其下有 `BasePlate`、`Pin_A`、`Pin_B`。两个销共用一个网格。
4. 中键旋转，滚轮缩放，点「适应全部」和「等轴测」。
5. 选中零件，在右侧改平移（毫米）或旋转（度），或用工具栏的移动 / 旋转操作器。
6. 文件 → 另存为，得到 `.simproj`，再打开确认树和网格还在。
7. 打开 `samples\conveyor_line.simproj`，点「运行」，看零件、光栅信号和推杆。脚本面板里可以改 `pusher`，或改选 `counter` 再运行。`sequence` 需要场景里有名为 `Slider` 和 `Hinge` 的关节。

无界面检查：

```powershell
.\simstudio.exe --import samples\step\sample_fixture.step --screenshot fixture.png --lang zh_CN
```

## 本地机器人模型（不要提交）

KUKA KR 6 R600 Z200 X 的 NX STEP 是供应商模型，体积大，不进入 Git。把解压后的 `.stp` 放在 `testdata\local\` 或 `models\`（这两个目录已忽略）。根装配产品号约为 `394.088-10.104.00`，五个子零件约为 `394.088-10.105.10`、`.104.20`、`.105.30`、`.105.40`、`.104.50`。导入后树里应能看到这五层。在这台机器上该文件约 15 秒完成读取、装配传输和细分，界面不冻结。

可选测试（CI 不跑）：

```powershell
$env:SIM_LOCAL_STEP = "testdata\local\KR6_R600_Z200_X.stp"
ctest --preset windows-msvc-release --output-on-failure
```

## Linux 上跑测试

虚拟机和 Linux CI 使用发行版的 Qt 6 与 Open CASCADE，避免在 CI 里从源码编译这两套库。CMake 仍然是同一套 `find_package`。Windows 交付以 vcpkg 清单为准。

```bash
sudo apt-get install -y build-essential ninja-build pkg-config cmake \
  qt6-base-dev qt6-base-dev-tools libgl1-mesa-dev libglu1-mesa-dev \
  libocct-foundation-dev libocct-modeling-data-dev libocct-modeling-algorithms-dev \
  libocct-data-exchange-dev libocct-ocaf-dev libocct-visualization-dev \
  nlohmann-json3-dev catch2 libtbb-dev libfontconfig1-dev \
  pybind11-dev python3-dev git
cmake --preset linux-system
cmake --build --preset linux-system
ctest --preset linux-system --output-on-failure
```

预设把编译器固定为 `/usr/bin/g++`。若 `c++` 指向 Clang 且找不到 `libstdc++`，不要删掉这一项。

## 当前限制

- 物理后端是 Jolt，不是 PhysX。输送面用接触速度注入，比接触回调晚一帧，不是皮带材质。关节电机是速度电机，位置环在我们这边按最大速度和加速度积分。正方向就是作者写的轴。
- AI 命令层可以改信号和仿真，但不能调用模型。填写本地端点也不会发请求。逻辑块之间只通过信号交换数据，远程执行还没做。
- 机器人逆解覆盖 SCARA 和球腕六轴；其它链是数值解。路径 `zone` 仍是关节空间重叠，不是工艺圆弧。近距查询的范围和误差写在 `docs/COLLISION.md`。内置软 PLC 已经扫梯形图并映射到信号表；PLCSIM Advanced 和 S7 还没接。OPC UA 只交换十进制节点号上的布尔。
- 刚体形状仍是盒子、球、胶囊和凸包，由 Jolt 求解。近距查询是另一条路径：盒子、球、胶囊走解析支撑，其它网格先做自写的体素凸分解。没有从 B-rep 生成的精确碰撞，也没有接入 CoACD 或 V-HACD。
- 导入只保留显示网格。B-rep 不写入项目；`.simproj` 记录 STEP 源路径，不内嵌原始 STEP。
- STEP 的 `ReadFile` / `Transfer` 没有逐实体进度，这两段时间状态栏是不确定进度。取消不能打断正在进行的 OCCT 调用。
- 关闭窗口时如果导入还在跑，进程会等该线程结束。
- 导入会替换整个场景。
- glTF 导入接口在，Phase 1 返回未实现。
- 项目包是 store（不压缩）ZIP，单文件约 4 GiB 上限，无 ZIP64。
- 操作器是局部轴的平移箭头和旋转环，不是完整 CAD 三元组。
- Windows 上的 Open CASCADE 8 与 Linux 软件包里的 7.6 若 API 不一致，需要在导入代码里加版本分支。
- 窗口图标、关于页和托盘图标使用蓝色圆角方标（白 C 与深色蛋形），没有文字。Windows 包里的 `simstudio.ico` 与 `tray.ico` 是同一套多尺寸图标。
