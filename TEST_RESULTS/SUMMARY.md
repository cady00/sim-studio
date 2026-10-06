# 测试汇总

机器与编译器见 `../SOURCE_VERSION.txt`。原始控制台在同目录。SKIP 单独列出，不计入通过。没有 XFAIL。没有把失败改写成通过。

| 运行 | 提交 | 用例 | 通过 | 失败 | 跳过 | 断言 | 进程退出 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| main 全量 | `eadee91a1b26267a578af61f3c2e4911f8b9bfdd` | 179 | 178 | 0 | 1 | 4804 通过 | 0 |
| PR #21 全量 | `83d5ba33dfe0cd139808b3851e89e29410af13e1` | 182 | 181 | 0 | 1 | 4870 通过 | 0 |
| G1 复现 | `eadee91` 加复现用例 | 8 | 8 | 0 | 0 | 33 通过 | 0 |
| G0 合成基准 | 同上 | 1 | 1 | 0 | 0 | 见基准摘要 | 0 |
| 基线加复现和基准的全量 | 测试分支，底层仍是 `eadee91` | 188 | 187 | 0 | 1 | 4846 通过 | 0 |

唯一的 SKIP，两版相同：

`public catalog meshes keep the example cycle`（`tests/test_motion.cpp:1443`）  
消息：`set SIM_CATALOG_MESH_LIVE=1 to download the public meshes`

PR #21 比 main 多 3 个用例、66 条断言，全是该分支上新增或改过的 STEP / 设备 / cojt 测试。两版都退出 0。这不能当成 #21 可以合入：它同时改了渲染和 builtin 抓取，而视口没有在这次跑截图。

Windows 全量测试和安装包：未运行。
