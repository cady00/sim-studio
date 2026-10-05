# 第三方许可证

Sim Studio 本体是专有软件，见 `LICENSE.txt`。完整清单、传递依赖、链接方式和闭源发行义务在 `docs/DEPENDENCIES.md`。下面的组件保留各自许可证。本仓库不包含只以 GPL 授权、没有可替代许可的库。私有商业使用在下列条件下允许。

双许可库按非 GPL 条款选用：FreeImage 选 FIPL-1.0，D-Bus 库选 AFL-2.1，Graphite2 选 LGPL-2.1。不链接 OCCT Draw。

| 组件 | 版本（清单 / 本机验证） | 许可证 | 用法 |
| --- | --- | --- | --- |
| Qt | Windows CI 为预编译 Qt 6.8.3；Linux CI 为发行版 Qt 6.4 | LGPL-3.0 | 动态链接。不从 vcpkg 源码编译 Qt。用户可替换对应 Qt 动态库。 |
| Open CASCADE Technology | vcpkg 基线 Open CASCADE 8.0.0；Linux CI 为 7.6.3 | LGPL-2.1 WITH Open CASCADE Exception 1.0 | 头文件指向 `OCCT_LGPL_EXCEPTION.txt`。vcpkg 端口元数据可能只写 LGPL-2.1-only，以 OCCT 源码中的例外文本为准。不静态链接进专有代码，不链接 Draw。 |
| nlohmann/json | 3.11+ | MIT | 头文件，用于项目 JSON 与界面文案。 |
| Catch2 | 3.x | Boost Software License 1.0 | 仅测试。 |
| Jolt Physics | 5.5.0（vcpkg `joltphysics`；Linux 为同一 git 标签） | MIT | 刚体、关节、射线。通常静态链进 `sim_physics`。 |
| CPython | vcpkg python3 3.12.13；Linux 为系统 Python 3.12 | PSF License 2.0 | 嵌入解释器。Windows 包带上 `python3xx.dll` 和 `Lib/`。 |
| pybind11 | vcpkg 3.0.1；Linux 为发行版 2.11 | BSD-3-Clause | 头文件，嵌入模块 `simstudio`。 |
| oneTBB | OCCT 的传递依赖 | Apache-2.0 | 由 OCCT 动态链接。 |
| FreeType、Fontconfig、FreeImage | OCCT / Qt 的传递依赖 | 各自的 FTL / MIT 类许可 | 动态链接。 |

没有链接 NVIDIA PhysX、Bullet、bgfx 或 Diligent Engine。渲染是仓库内的 `IRenderer`，当前后端是 OpenGL。bgfx（BSD-2-Clause）和 Diligent（Apache-2.0）只在 `docs/ARCHITECTURE.md` 里做过选型，没有编译进程序。

近距查询（宽相 BVH、GJK 距离、支撑平面穿透、体素凸分解、窄带符号距离、路径扫掠）是仓库内的 `src/collision`，没有引入 CoACD、V-HACD、libccd 或其他碰撞库。Jolt 仍只做刚体，不作为 `ICollisionQuery` 的后端。

Qt 动态链接义务：随二进制提供 Qt 许可证文本与对应动态库，并允许接收者替换这些库。Open CASCADE 例外允许专有程序使用 OCCT 库，前提是不把 OCCT 源码改动藏进专有模块；我们没有修改 OCCT 源码。

获取许可证全文：

- Qt：https://www.qt.io/licensing/
- Open CASCADE：随 OCCT 源码的 `OCCT_LGPL_EXCEPTION.txt` 与 `LICENSE_LGPL_21.txt`
- nlohmann/json：https://github.com/nlohmann/json/blob/develop/LICENSE.MIT
- Catch2：https://github.com/catchorg/Catch2/blob/devel/LICENSE.txt
- Jolt Physics：https://github.com/jrouwe/JoltPhysics/blob/master/LICENSE
- CPython：https://docs.python.org/3/license.html
- pybind11：https://github.com/pybind/pybind11/blob/master/LICENSE
