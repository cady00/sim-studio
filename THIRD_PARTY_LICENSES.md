# 第三方许可证

Sim Studio 本体是专有软件，见 `LICENSE.txt`。下面的组件保留各自许可证。本仓库不包含 GPL-only 库。私有商业使用在下列条件下允许。

| 组件 | 版本（清单 / 本机验证） | 许可证 | 用法 |
| --- | --- | --- | --- |
| Qt | vcpkg 基线中的 Qt 6.11.1；Linux CI 为发行版 Qt 6.4 | LGPL-3.0 | 动态链接。Windows 三元组 `VCPKG_LIBRARY_LINKAGE dynamic`。用户可替换对应 Qt 动态库。 |
| Open CASCADE Technology | vcpkg 基线 Open CASCADE 8.0.0；Linux CI 为 7.6.3 | LGPL-2.1 WITH Open CASCADE Exception 1.0 | 头文件指向 `OCCT_LGPL_EXCEPTION.txt`。vcpkg 端口元数据可能只写 LGPL-2.1-only，以 OCCT 源码中的例外文本为准。不静态链接进专有代码，不链接 Draw。 |
| nlohmann/json | 3.11+ | MIT | 头文件，用于项目 JSON 与界面文案。 |
| Catch2 | 3.x | Boost Software License 1.0 | 仅测试。 |
| Bullet Physics | vcpkg bullet3 3.25；Linux 为发行版 Bullet 3.24 | Zlib | 动态链接。刚体、关节、射线。 |
| CPython | vcpkg python3 3.12.13；Linux 为系统 Python 3.12 | PSF License 2.0 | 嵌入解释器。Windows 包带上 `python3xx.dll` 和 `Lib/`。 |
| pybind11 | vcpkg 3.0.1；Linux 为发行版 2.11 | BSD-3-Clause | 头文件，嵌入模块 `simstudio`。 |
| oneTBB | OCCT 的传递依赖 | Apache-2.0 | 由 OCCT 动态链接。 |
| FreeType、Fontconfig、FreeImage | OCCT / Qt 的传递依赖 | 各自的 FTL / MIT 类许可 | 动态链接。 |

没有链接 NVIDIA PhysX。vcpkg 中的 PhysX 5 端口构建整套 Omniverse SDK，Ubuntu 没有对应的二进制包，因此本阶段用 Bullet 实现 `IPhysicsEngine`。若以后替换，许可证再按实际链接的版本补充。

Qt 动态链接义务：随二进制提供 Qt 许可证文本与对应动态库，并允许接收者替换这些库。Open CASCADE 例外允许专有程序使用 OCCT 库，前提是不把 OCCT 源码改动藏进专有模块；我们没有修改 OCCT 源码。

获取许可证全文：

- Qt：https://www.qt.io/licensing/
- Open CASCADE：随 OCCT 源码的 `OCCT_LGPL_EXCEPTION.txt` 与 `LICENSE_LGPL_21.txt`
- nlohmann/json：https://github.com/nlohmann/json/blob/develop/LICENSE.MIT
- Catch2：https://github.com/catchorg/Catch2/blob/devel/LICENSE.txt
- Bullet：https://github.com/bulletphysics/bullet3/blob/master/LICENSE.txt
- CPython：https://docs.python.org/3/license.html
- pybind11：https://github.com/pybind/pybind11/blob/master/LICENSE
