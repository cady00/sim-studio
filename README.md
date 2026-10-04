# Sim Studio

Sim Studio 是一套私有、可离线使用的 Windows 桌面产线仿真软件。长期目标对齐 Siemens Tecnomatix Process Simulate / Process Designer 与 NX Mechatronics Concept Designer：机器人运动、碰撞、节拍、机电物理、PLC 虚拟调试，以及类似 MELSOFT Gemini / Visual Components 的 Python 脚本。

当前仓库是 **0.3**：场景图、三维视口、STEP 装配导入、Jolt 刚体与关节、信号表、嵌入的 Python 脚本，以及默认关闭的命令 / AI 接口。没有遥测，运行时不访问网络。请保持仓库私有。

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
- 保存 / 打开 `.simproj`（JSON + 显示网格缓存，格式版本 2，仍能打开版本 1）。说明见 [docs/PROJECT_FORMAT.md](docs/PROJECT_FORMAT.md)。
- 示例装配 `samples/step/sample_fixture.step`（底板 + 两个共用网格的销）。
- 示例产线 `samples/conveyor_line.simproj`：发生器、输送面、光栅、气动式移动副推杆、销毁器，由 Python 脚本 `pusher` 驱动。脚本副本在 `samples/scripts/`。
- 工具栏：运行、暂停、停止、复位，速度倍率（0–16），仿真时间。复位与停止相同，恢复按下运行前的变换并删掉运行中生成的零件。
- 装配树和属性面板可改机构类型（刚体、碰撞体、固定副、转动副、移动副、输送面、碰撞/距离传感器、光栅、发生器、销毁器）。底部是信号表和脚本编辑器。

导入大 STEP 时，读取和细分在后台线程进行。状态栏显示阶段进度，可点「取消」。取消发生在零件之间；Open CASCADE 正在读文件或传输装配时，取消要等到该阶段结束。界面事件循环保持可响应。

## 五阶段路线

1. **Phase 1** 场景、视口、STEP 装配显示、项目文件、中文界面。
2. **当前仓库** Jolt 刚体、关节、传感器、输送线、信号表，以及嵌入的 Python。渲染走 `IRenderer`（现在是 OpenGL）。AI 提供者默认关闭，这个二进制不联网。六轴求解器和用户定义的运动学编辑器还没做，接口留在 `docs/ARCHITECTURE.md`。
3. **Phase 3** 六轴机器人运动学与碰撞。KUKA KR 6 R600 Z200 X 的 STEP 子零件 `394.088-10.105.10`、`.104.20`、`.105.30`、`.105.40`、`.104.50`（根产品 `394.088-10.104.00`）会映射到关节连杆。当前只把它们显示成装配树，不猜测关节轴。
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

Windows 安装包不随每次推送自动编。在 GitHub 的 Actions 页手动运行工作流 `CI`（`workflow_dispatch`）才会编 `SimStudio-win64`，里面是 `SimStudio-0.3.0-win64.zip`。解压后运行 `SimStudio\simstudio.exe`。压缩包带有 Qt / OCCT / Jolt / Python 的 DLL、`Lib\` 标准库、`python3xx._pth`、`plugins\` 和 `samples\`。不需要安装 Python 或 Visual Studio。私有仓库的 Windows 分钟按两倍计算，所以这条任务不要改回自动触发。上一版自动任务在链 OCCT 8 时找不到 `TKSTEPBase.lib`（7.8 起改为 `TKDESTEP`），配置阶段还从源码编了 Qt。现在只链接存在的 toolkit，Qt 用预编译包。拉取请求只跑 Linux 测试；只改 `docs/` 或 Markdown 不会启动任务。

1. 在该次 Actions 运行的 Artifacts 里下载 `SimStudio-win64` 并解压。
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
- 没有机器人正解/逆解、节拍或 PLC。信号表已经在，Phase 5 再接到 OPC UA / S7。
- 碰撞形状支持盒子、球、胶囊和网格凸包。没有凹多面体，也没有从 B-rep 生成的精确碰撞。
- 导入只保留显示网格。B-rep 不写入项目；`.simproj` 记录 STEP 源路径，不内嵌原始 STEP。
- STEP 的 `ReadFile` / `Transfer` 没有逐实体进度，这两段时间状态栏是不确定进度。取消不能打断正在进行的 OCCT 调用。
- 关闭窗口时如果导入还在跑，进程会等该线程结束。
- 导入会替换整个场景。
- glTF 导入接口在，Phase 1 返回未实现。
- 项目包是 store（不压缩）ZIP，单文件约 4 GiB 上限，无 ZIP64。
- 操作器是局部轴的平移箭头和旋转环，不是完整 CAD 三元组。
- Windows 上的 Open CASCADE 8 与 Linux 软件包里的 7.6 若 API 不一致，需要在导入代码里加版本分支。
- 窗口图标来自品牌 SVG 光栅化的 PNG，以及随附的 `app.ico`。
