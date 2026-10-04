# 依赖与闭源发行义务

Sim Studio 本体是专有软件，见仓库根目录 `LICENSE.txt`。本文列出进入构建或发行物的第三方成分，包括传递依赖、只在 CI 里出现的工具，以及仓库里自己实现、但容易被误认为拷来的算法。短表在 `THIRD_PARTY_LICENSES.md`。

本机核对日期：2026-10-04，系统是 Ubuntu 24.04，二进制是 `out/build/linux-system/src/app/simstudio` 的 `ldd`。Windows 端口版本以 `vcpkg.json` 的基线 `cd61e1e26a038e82d6550a3ebbe0fbbfe7da78e3` 为准；这台机器没有安装 vcpkg，没有在本机解析出 Windows 端口的精确版本号。

仓库里没有 vendored 第三方源码树。Jolt 在 Linux 上由 CMake `FetchContent` 拉到构建目录，不进 git。

## 结论

没有只以 GPL / AGPL 授权、又链进产品的库，因此没有需要从工程里删掉的依赖。双许可库按下面的非 GPL 条款选用：

| 库 | 可选条款 | 本产品选用 | 原因 |
| --- | --- | --- | --- |
| FreeImage | GPL-2.0 或 GPL-3.0 或 FIPL-1.0 | FIPL-1.0 | 允许作为更大作品的一部分发行。我们不修改它的源文件。 |
| libdbus | GPL-2.0-or-later 或 AFL-2.1 | AFL-2.1 | 只动态链接共享库。GPL-only 的 dbus 命令行工具不进产品。 |
| Graphite2 | LGPL-2.1 或 MPL-1.1 或 GPL-2.0 | LGPL-2.1 | Qt / HarfBuzz 的传递依赖，动态链接。 |

OCCT 的 Draw 模块没有链接。`ldd` 里没有 `TKDraw`、`TKTopTest`。

## 直接依赖

| 组件 | 用途 | 版本 | 许可证 | 链接 | 闭源商业发行义务 | 风险 |
| --- | --- | --- | --- | --- | --- | --- |
| Qt 6 | 窗口、控件、OpenGL 视口 | Linux 发行版 6.4.2。Windows CI 预编译 6.8.3（`jurplel/install-qt-action`）。不进 vcpkg。 | LGPL-3.0（Qt 也提供商业许可） | 动态：`libQt6Core/Gui/Widgets/OpenGL/OpenGLWidgets` | 随二进制提供对应 Qt 动态库和 LGPL 文本。接收者必须能替换这些库。不静态链接 Qt。不把 Qt 源码改动并进专有模块。 | 弱 copyleft。动态链接和可替换满足 LGPL 的常见做法。Qt 的商业许可可以另购，本仓库没有。 |
| Open CASCADE 7.6 / 8 | STEP 读入与网格化 | Linux 7.6.3+dfsg1-7.1build1。Windows 为 vcpkg 基线的 `opencascade`，`default-features: false`，只开 `freetype`。 | LGPL-2.1，带 `OCCT_LGPL_EXCEPTION.txt` | 动态：`libTK*.so`。头文件只出现在 `src/io/src/StepImporter.cpp` 和开发工具 `tools/generate_sample_step.cpp`。 | 例外允许专有程序调用未修改的 OCCT 库。我们没有改 OCCT 源码。不要静态链进专有目标，不要链接 Draw。若修改了 OCCT 本身，修改部分仍须按 LGPL 提供。 | 弱 copyleft，例外覆盖应用程序。Ubuntu 的 `libTKService` 还 `NEEDED` FreeImage，见下。 |
| Jolt Physics | 可选刚体后端 | 5.5.0，git 标签 `v5.5.0`。Windows 端口名 `joltphysics`。 | MIT | Linux 为静态库 `libJolt.a`，链进 `sim_physics`。Windows triplet `x64-windows-release` 是动态 CRT 和动态库。 | 在发行说明里保留版权和 MIT 许可文本。 | 宽松。可关。`SIMSTUDIO_WITH_JOLT=OFF` 后产品不链接它。 |
| nlohmann/json | 项目 JSON 与文案 | Linux 包 3.11.3 | MIT | 仅头文件 | 保留 MIT 文本。 | 低。 |
| CPython | 嵌入脚本 | Linux 3.12.3。Windows 为 vcpkg `python3`。 | PSF License 2.0 | 动态：`libpython3.12.so` | 保留 PSF 许可。Windows 包带上 `python3xx.dll` 和标准库 `Lib/`。 | 低。 |
| pybind11 | `simstudio` 嵌入模块 | Linux 2.11.1。Windows 为 vcpkg `pybind11`。 | BSD-3-Clause | 仅头文件，链到 libpython | 保留 BSD 文本。 | 低。 |
| Catch2 | 单元测试 | Linux 3.4.0 | Boost Software License 1.0 | 测试可执行文件。不进 `simstudio`。 | 不随产品发行则无义务。若发行测试程序，保留 BSL 文本。 | 无产品风险。 |

## 传递依赖（本机 `ldd` 实际加载）

这些库不是 `CMakeLists.txt` 里的 `find_package` 目标。它们跟着 Qt 或 OCCT 的动态库进来。发行 Linux 包时，系统库通常由发行版提供；发行 Windows 包时，要带上 vcpkg / Qt 安装器复制出来的 DLL，并附上各自的许可文本。

| 组件 | 从哪来 | 本机版本 | 许可证 | 链接 | 义务与风险 |
| --- | --- | --- | --- | --- | --- |
| oneTBB | OCCT | 2021.11.0 | Apache-2.0 | 动态 `libtbb.so` | 保留 NOTICE。低。 |
| FreeType | OCCT / Qt | 2.13.2 | Freetype License（类 BSD）或 GPLv2，我们用 FTL | 动态 | 保留 FTL 文本。不采用 GPL 选项。 |
| Fontconfig | Qt | 2.15.0 | fontconfig 许可（MIT 风格） | 动态 | 保留文本。低。 |
| FreeImage | Ubuntu 的 `libTKService.so.7` | 发行版 `libfreeimage3` | GPL-2.0 或 GPL-3.0 或 FIPL-1.0 | 动态 | **选用 FIPL-1.0。** 文件级义务：改了 FreeImage 的源文件就要公开那些修改。我们没有改。专有程序可以作为 Larger Work 链接它。Windows 清单关掉了 OCCT 默认特性，没有打开 freeimage。Linux 发行版包卸不掉这根 `NEEDED`，除非自己编一份不带 FreeImage 的 OCCT。商业安装包应优先用那份不带 FreeImage 的 OCCT，或确认随包的 FreeImage 按 FIPL 提供。 |
| zlib | Qt | 发行版 | zlib License | 动态 | 保留文本。低。 |
| libpng | Qt | 发行版 | libpng License | 动态 | 保留文本。低。 |
| HarfBuzz | Qt | 发行版 | MIT | 动态 | 低。 |
| ICU | Qt | 发行版 libicu74 | Unicode License | 动态 | 保留许可。低。 |
| GLib | Qt | 发行版 | LGPL-2.1 | 动态 | 与 Qt 相同：动态链接，允许替换。 |
| Graphite2 | HarfBuzz | 发行版 | 见上表，选用 LGPL-2.1 | 动态 | 弱 copyleft。 |
| libdbus | Qt | 发行版 | 选用 AFL-2.1 | 动态 | 不用 GPL 条款。 |
| Mesa / libGL | 视口 | 25.2.8 | MIT | 动态 | 低。 |
| libX11、libxcb | Qt | 发行版 | MIT / X11 | 动态 | 低。 |
| expat、bzip2、brotli、pcre2、md4c、double-conversion、libb2 | Qt 或 OCCT | 发行版 | MIT、BSD 或 bzip2 许可 | 动态 | 保留各自文本。低。 |
| libjpeg、libjxr | FreeImage | 发行版 | IJG 与 BSD 风格 | 动态 | 随 FreeImage 出现。Windows 不带 FreeImage 时不出现。 |
| libstdc++ / libgcc | GCC 运行时 | 系统 | GPL-3.0 **带 GCC Runtime Library Exception** | 动态 | 例外允许专有程序链接这些运行时。保留例外文本即可。 |
| glibc | 系统 | 系统 | LGPL-2.1 | 动态 | 使用系统 C 库，不静态链接。 |

没有链接 NVIDIA PhysX、Bullet、bgfx、Diligent、CoACD、V-HACD、libccd、zlib 源码（zip 容器自己写，见下）、OpenUSD、OPC UA 协议栈。OCCT 的 CMake 配置把 `HAVE_RAPIDJSON` 传进了编译定义。本机 `ldd` 没有加载 RapidJSON，我们的源文件也不包含它。RapidJSON 若被 OCCT 以头文件用到，许可证是 MIT。

## 只在 CI 或构建机上出现

这些不进 `simstudio` 的链接表。

| 工具 | 版本 / 引用 | 许可证 | 说明 |
| --- | --- | --- | --- |
| CMake | 本机 3.28.3 | BSD-3-Clause | 构建。 |
| Ninja | 发行版 | Apache-2.0 | 构建。 |
| GCC / libstdc++ 编译器 | 发行版 | GPL-3.0，运行时见上 | 构建。 |
| vcpkg | Windows CI 克隆的 microsoft/vcpkg | MIT | 只在 Windows 打包作业里。 |
| `actions/checkout@v4` | GitHub Action | MIT | CI。 |
| `actions/upload-artifact@v4` | GitHub Action | MIT | 仅手动 `workflow_dispatch` 的 Windows 作业。 |
| `actions/github-script@v7` | GitHub Action | MIT | 同上。 |
| `jurplel/install-qt-action@v4` | GitHub Action | 以该仓库许可证为准，安装的是官方预编译 Qt | 只下载 Qt，不把 Action 源码链进产品。 |
| git | CI 拉 Jolt 标签 | GPL-2.0 | 构建机工具，不发行。 |

## 自写代码，不是拷来的片段

检索了 `src/` 里的版权头、`adapted from`、`copied` 和第三方头文件。产品模块里没有粘贴的第三方函数。

| 位置 | 做什么 | 来源 |
| --- | --- | --- |
| `src/collision` | 宽相、GJK 距离、支撑平面穿透、体素凸分解、窄带符号距离、路径扫掠 | 按公开算法自己写。没有 libccd、Bullet 或 Jolt 的碰撞源码。 |
| `src/io/src/ZipArchive.cpp` | 存储法（method 0）的 ZIP 与 CRC-32 | 按 PKZIP APPNOTE 的记录布局自己写。不链接 minizip 或 zlib。 |
| `src/physics/src/BuiltinPhysicsEngine.cpp` | 关节积分、半隐式欧拉、盒子分离、射线 | 本仓库。Jolt 的头文件只出现在 `JoltPhysicsEngine.cpp`。 |
| `src/io/src/ObjImporter.cpp` | OBJ | 本仓库。 |
| `src/motion` | 解析逆解、阻尼最小二乘、梯形 / 五次多项式 | 本仓库。 |

GJK 和 ZIP 记录布局是公开规格，实现是这里写的。没有发现需要整段换掉的拷贝。

## 接口：哪些可以换

| 缝 | 接口 | 现在的实现 | 第三方是否漏出头文件 |
| --- | --- | --- | --- |
| 刚体 | `IPhysicsEngine`，`PhysicsBackend` | 默认 `jolt`；`builtin` 已能驱动移动副、转动副，做盒子接触、光栅和落体静止 | Jolt 头文件只在 `src/physics/src/JoltPhysicsEngine.cpp` |
| 近距 | `ICollisionQuery` | `src/collision`，不调用 Jolt | 无 |
| CAD | `IGeometryKernel` / `ICadImporter` | `createGeometryKernel()` 名为 `occt`。`step` 走 `StepImporter`，`obj` 走自写解析器 | OCCT 头文件只在 `StepImporter.cpp` 和 `tools/generate_sample_step.cpp` |
| 渲染 | `IRenderer` | OpenGL 3.3 | Qt 只在 `src/app` 和 `src/render`。`IRenderer` 没有 Qt 类型 |
| 界面 | 应用壳 | `src/app` 的 `QMainWindow` | 核心、运动、碰撞、物理、IO、脚本、命令层不包含 Qt |
| 脚本 | `IScriptHost` | CPython + pybind11 | 头文件在 `src/scripting` |

换 OCCT 或 Qt 要换对应实现，不改场景、信号表和 `.simproj`。builtin 刚体还不能替代 Jolt 跑整条输送线：没有凸包接触、摩擦和力限制电机。那是下一步，不是把 Jolt 从默认路径拿掉的理由。默认 `createPhysicsEngine()` 在 `SIMSTUDIO_WITH_JOLT` 打开时仍是 Jolt。

## 发行时要放进包里的文本

- 本产品 `LICENSE.txt`
- Qt 的 LGPL 文本，以及实际携带的 Qt 动态库
- OCCT 的 `LICENSE_LGPL_21.txt` 与 `OCCT_LGPL_EXCEPTION.txt`
- Jolt 的 MIT 文本（若该构建链接了 Jolt）
- nlohmann/json 的 MIT、pybind11 的 BSD、CPython 的 PSF
- 随 Qt / OCCT 带上的传递库许可，至少包括 FreeType、oneTBB、FreeImage 的 FIPL（仅当该构建的 OCCT 链了 FreeImage）
- GCC 运行时例外的说明（若发行物依赖 libstdc++）
