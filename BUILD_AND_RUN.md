# 构建与运行（G0）

这次只在 Linux 上编译和测试。Windows 安装包没有编，也没有手动触发 GitHub Actions 的 `workflow_dispatch`。配额说明写在工作流注释里：Windows 分钟在私有仓库按两倍计，任务只在手动运行时出现。本记录不把它写成已验证。

## 身份

| 树 | 提交 | 用途 |
| --- | --- | --- |
| `main` / `development` | `eadee91a1b26267a578af61f3c2e4911f8b9bfdd` | 基线。`development` 从这里拉出，没有合入 #21 |
| PR #21 `cursor/step-assembly-complete-edd5` | `83d5ba33dfe0cd139808b3851e89e29410af13e1` | 候选。只编译和测试，不合入 |

依赖和编译器版本见 `SOURCE_VERSION.txt`。

## Linux

工作目录是仓库根。预设 `linux-system` 把编译器固定为 `/usr/bin/g++`，生成器是 Ninja，类型 `RelWithDebInfo`。

```bash
cmake --preset linux-system
cmake --build --preset linux-system
ctest --preset linux-system --output-on-failure
```

这次实际跑的是已经链好的测试程序，并保存控制台输出：

```bash
./out/build/linux-system/tests/simstudio_tests --reporter console --durations yes
```

两版都用同一个构建目录先后编译：先在 #21 的提交上编并测，再切到 `eadee91` 重新编译并测。不是两台机器。OpenGL 是 Mesa llvmpipe，未加速。测试进程没有开窗口。

## Windows

未验证。不要从这次记录推断 `simstudio.exe` 或 `OndotSim-win64` 能在干净的 Windows 上启动。仓库里的预设是 `windows-msvc`（Visual Studio 2022、vcpkg 清单、`QT_ROOT_DIR`）。工作流 `CI` 的 `windows` 作业只在 `workflow_dispatch` 时运行。这次没有触发它。

## 回退

`development` 与 `main` 在 G0 记录时指向同一提交。丢掉后续文档提交时：

```bash
git checkout development
git reset --hard eadee91a1b26267a578af61f3c2e4911f8b9bfdd
```

不要对已经推送的 `development` 强行重置，除非确认没有别人基于它提交。PR #21 保持原分支，不合并就是回退。
