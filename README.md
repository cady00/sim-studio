# Sim Studio

Sim Studio 是一套私有、可离线使用的 Windows 桌面产线仿真软件。长期目标对齐 Siemens Tecnomatix Process Simulate / Process Designer 与 NX Mechatronics Concept Designer：机器人运动、碰撞、节拍、机电物理、PLC 虚拟调试，以及类似 MELSOFT Gemini / Visual Components 的 Python 脚本。

当前仓库是 **Phase 1**：场景图、三维视口、STEP 装配导入、自有项目格式。没有遥测，运行时不访问网络。请保持仓库私有。

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
- 保存 / 打开 `.simproj`（JSON + 显示网格缓存）。说明见 [docs/PROJECT_FORMAT.md](docs/PROJECT_FORMAT.md)。
- 示例装配 `samples/step/sample_fixture.step`（底板 + 两个共用网格的销）。

导入大 STEP 时，读取和细分在后台线程进行。状态栏显示阶段进度，可点「取消」。取消发生在零件之间；Open CASCADE 正在读文件或传输装配时，取消要等到该阶段结束。界面事件循环保持可响应。

## 五阶段路线

1. **Phase 1（本仓库）** 场景、视口、STEP 装配显示、项目文件、中文界面。
2. **Phase 2** 在 `sim_physics` 后接入 NVIDIA PhysX 5：刚体、关节、传感器、输送线。`sim_scripting` 嵌入 pybind11，提供 Python 设备脚本。
3. **Phase 3** 六轴机器人运动学与碰撞。KUKA KR 6 R600 Z200 X 这类 STEP 里的子零件会映射到关节连杆。Phase 1 只把它们显示成装配树，不猜测关节轴。
4. **Phase 4** 产线布局与节拍。
5. **Phase 5** 虚拟调试：OPC UA、S7、PLCSIM Advanced，以及对外的 Python API。

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

程序在 `out\build\windows-msvc\src\app\Release\simstudio.exe`。vcpkg 使用清单模式和 `triplets\x64-windows-release.cmake`（动态 CRT / 动态库、仅 Release），以便按 LGPL 动态链接 Qt。

打包成可拷贝目录（并在安装了 NSIS 时生成安装包）：

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
  nlohmann-json3-dev catch2 libtbb-dev libfontconfig1-dev
cmake --preset linux-system
cmake --build --preset linux-system
ctest --preset linux-system --output-on-failure
```

预设把编译器固定为 `/usr/bin/g++`。若 `c++` 指向 Clang 且找不到 `libstdc++`，不要删掉这一项。

## 当前限制

- 没有物理步进、机器人正解/逆解、碰撞、节拍、PLC 或 Python 运行时。对应菜单只说明后续阶段。
- 导入只保留显示网格。B-rep 不写入项目；`.simproj` 记录 STEP 源路径，不内嵌原始 STEP。
- STEP 的 `ReadFile` / `Transfer` 没有逐实体进度，这两段时间状态栏是不确定进度。取消不能打断正在进行的 OCCT 调用。
- 关闭窗口时如果导入还在跑，进程会等该线程结束。
- 导入会替换整个场景。
- glTF 导入接口在，Phase 1 返回未实现。
- 项目包是 store（不压缩）ZIP，单文件约 4 GiB 上限，无 ZIP64。
- 操作器是局部轴的平移箭头和旋转环，不是完整 CAD 三元组。
- Windows 上的 Open CASCADE 8 与 Linux 软件包里的 7.6 若 API 不一致，需要在导入代码里加版本分支。
- 窗口图标来自品牌 SVG 光栅化的 PNG，以及随附的 `app.ico`。
