# 第三方许可证

Sim Studio 本体是专有软件，见 `LICENSE.txt`。下面的组件保留各自许可证。本仓库不包含 GPL-only 库。私有商业使用在下列条件下允许。

| 组件 | 版本（清单 / 本机验证） | 许可证 | 用法 |
| --- | --- | --- | --- |
| Qt | vcpkg 基线中的 Qt 6.11.1；Linux CI 为发行版 Qt 6.4 | LGPL-3.0 | 动态链接。Windows 三元组 `VCPKG_LIBRARY_LINKAGE dynamic`。用户可替换对应 Qt 动态库。 |
| Open CASCADE Technology | vcpkg 基线 Open CASCADE 8.0.0；Linux CI 为 7.6.3 | LGPL-2.1 WITH Open CASCADE Exception 1.0 | 头文件指向 `OCCT_LGPL_EXCEPTION.txt`。vcpkg 端口元数据可能只写 LGPL-2.1-only，以 OCCT 源码中的例外文本为准。不静态链接进专有代码，不链接 Draw。 |
| nlohmann/json | 3.11+ | MIT | 头文件，用于项目 JSON 与界面文案。 |
| Catch2 | 3.x | Boost Software License 1.0 | 仅测试。 |
| oneTBB | OCCT 的传递依赖 | Apache-2.0 | 由 OCCT 动态链接。 |
| FreeType、Fontconfig、FreeImage | OCCT / Qt 的传递依赖 | 各自的 FTL / MIT 类许可 | 动态链接。 |

计划在 Phase 2 引入、**当前未链接**：

| 组件 | 许可证 |
| --- | --- |
| NVIDIA PhysX 5 | BSD-3-Clause |
| pybind11 | BSD-3-Clause |

Qt 动态链接义务：随二进制提供 Qt 许可证文本与对应动态库，并允许接收者替换这些库。Open CASCADE 例外允许专有程序使用 OCCT 库，前提是不把 OCCT 源码改动藏进专有模块；我们没有修改 OCCT 源码。

获取许可证全文：

- Qt：https://www.qt.io/licensing/
- Open CASCADE：随 OCCT 源码的 `OCCT_LGPL_EXCEPTION.txt` 与 `LICENSE_LGPL_21.txt`
- nlohmann/json：https://github.com/nlohmann/json/blob/develop/LICENSE.MIT
- Catch2：https://github.com/catchorg/Catch2/blob/devel/LICENSE.txt
