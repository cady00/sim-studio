# 已知问题（G0 / G1 开头）

基线是 `development` = `main` = `eadee91a1b26267a578af61f3c2e4911f8b9bfdd`。行号对应该提交。PR #21（`83d5ba3`）没有合入。复现测试在 `tests/test_g1_defects.cpp`，原始输出在 `TEST_RESULTS/g1-repro.txt`。下面「已复现」表示测试进程里看到了所说的行为，不是说产品已经修了。

## F01 镂空框架被包围盒当成实体遮挡 — 已复现

`hiddenByOcclusion` / `rasterizeOccluder`（`src/render/include/sim/render/Renderer.hpp`）把包围盒的屏幕矩形写成深度。视口在 `src/render/src/ViewportWidget.cpp` 里用 160×90 的缓冲，按距离从近到远光栅化。

测试构造一个框的包围盒和框心更远处的小盒。小盒的屏幕矩形完全落在框的矩形里。结果 `hidden=1`。框上有没有洞，算法看不到。

证据：`G1 F01 REPRODUCED hidden=1 frame_px=31,17-128,72 hole_px=76,43-84,47`。

这次没有启动视口，所以没有窗口截图。算法测试和视口调用的是同一对函数。

## F02 按距离换 LOD，粗网格丢掉三角形 — 已复现

`selectLodLevel`（`Renderer.hpp`）只看距离：超过 8000 mm 用最粗级，超过 3000 mm 用中间级。没有屏幕尺寸，没有几何误差。

视口在 `ViewportWidget.cpp` 里对多于 8 个三角形的网格准备 3 级，粗化格子是对角线的 20%（1 级）或 55%（2 级），结果只放进 `lodCache_`，供绘制。碰撞和拾取读的是场景网格，不读这个缓存。显示仍然会少三角形。

证据：8 m 选第 2 级，4 m 选第 1 级，1 m 选第 0 级。200 个三角形的格子在 55% 对角线（777.8 mm）上变成 2 个三角形。

## F03 导入线程没有统一异常边界 — 部分复现

`MainWindow::importStepAsync`（`src/app/src/MainWindow.cpp`）的工作线程直接调用 `StepImporter::importFile`。该函数在 `ReadFile` / `Transfer` / 细分之外没有包住 `Standard_Failure`、`std::exception` 和未知异常。文档关闭时的 `catch` 只包 `XCAFApp` 的 `Close`。

这次喂了一份只有 STEP 壳、没有实体的文件。OCCT 在读文件时打印 `Interruption ReadFile par Exception` / `NCollection_Array1::Create`，然后返回失败字符串，进程没有退出。也就是说这条输入被 OCCT 自己收成了读失败，没有变成未捕获异常。

因此：缺边界这件事在源码里成立；这份损坏文件没有把进程打崩。还没有一份能稳定抛出并穿透 `importFile` 的输入，所以不把「进程会终止」写成已测到的事实。工作进程隔离没有做。

## F04 零三角形仍报 imported — 已复现

成功条件是 `context.created != 0`，然后 `finish(true, "imported")`（`StepImporter.cpp` 末尾）。`ImportResult` 只有 `ok`，没有 complete / partial / structure_only / failed / cancelled。

一个只含顶点的 AP214 文件：`ok=1 message=imported nodes=1 tris=0`。

## F05 深度超过 64 静默丢掉叶子 — 已复现

`importLabel` 和 `countPartLabels` 在 `depth > 64` 时返回 0，不写警告。70 层装配再加一个 8 mm 方盒：`ok=1 message=imported nodes=65 named_L=65 leaf=0 tris=0`。叶子和方盒的三角形不在场景里，调用方仍看到成功。

装配标签自己的实体也没有进网格。`AddShape(box, true)` 再挂一个子方盒：子方盒在，父节点 `body_mesh=0`，三角形只有 12（子方盒）。父实体被装配分支跳过。这是「装配自带形状」遗漏，不是重复网格。

## F06 项目只存源路径 — 已复现

`ProjectStore::save` 把 `scene.sourceFiles` 写成 `sources`。没有内容哈希，没有嵌入 STEP，没有几何版本，没有实例映射。重新打开后路径还在，网格是显示缓存。

证据：导入 `samples/step/sample_fixture.step` 再保存，读回的 `sources` 是该路径。sha256 `59a6e4cbc591ebcf1e585394e559a6e923bbd9f3153fd650935264eb47e1ccc2`。

## F07 交互导入用 steal 换掉整个场景 — 已复现；撤销这条被否定

`onImportFinished` 在成功时 `scene_.steal(*scene)`。测试里 `steal` 之后原来的节点 `Kept` 不在了，只剩 `Replacement`。

同一场景上的 `EditHistory` 在 `steal` 的结构通知里记下了替换前的快照。`undo()` 把 `Kept` 找回来，并去掉 `Replacement`（`undo kept=1 replacement_gone=1`）。所以「替换发生」成立；「撤销不能恢复」在现有历史上不成立。主窗口已经 `history_.watch(scene_)`，交互导入走的是同一条 `steal`。

仍然没有插入、替换、更新三种入口，也没有绑定冲突列表。下一次 G1 要补的是这个选择，不是再证明撤销按钮完全没有。

## 未在本轮修

F03、F04、F05、F07 的修复留作单独的小 PR，基线是 `development`。这轮只加了复现测试，没有改导入行为。

## 跳过项

两版全量测试都有 1 条 SKIP：`tests/test_motion.cpp` 的 public catalog meshes，因为没有设置 `SIM_CATALOG_MESH_LIVE=1`。这条不是通过。它不阻塞 G0 记录，但按交接要求，关键能力若只剩 SKIP，阶段不能写成通过。目录网格的视觉验收仍是未测。
