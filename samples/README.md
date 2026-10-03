# 示例

`step/sample_fixture.step` 由 `tools/generate_sample_step.cpp` 用 Open CASCADE 写出，可以提交。它不是供应商模型。

内容：

- 装配 `Fixture`
- 零件 `BasePlate`：120 × 80 × 12 mm 长方体，颜色约 (0.17, 0.22, 0.40)
- 零件 `Pin`：半径 6 mm、高 25 mm 的圆柱，颜色约 (0.57, 0.58, 0.95)
- 实例 `Pin_A` 位于 (30, 25, 12) mm，`Pin_B` 位于 (90, 55, 12) mm，共用一个网格

模式 AP214，单位毫米。

重新生成：

```bash
cmake --build --preset linux-system --target simstudio_stepgen
./out/build/linux-system/simstudio_stepgen samples/step/sample_fixture.step
```

`testdata/local/` 与 `models/` 已在 `.gitignore` 中。KUKA 或其他厂商的 STEP 只放在那里，不要 `git add`。

## 输送线

`conveyor_line.simproj` 是 Phase 2 演示，不依赖供应商模型。场景里有地板、沿 +X 以 240 mm/s 运动的输送面、隐藏的 50 mm 立方体原型 `Blank`、每 1.2 s 在输送起点生成零件的发生器、沿 +Y 的光栅（信号 `barrier`）、以 `PusherBase` 为参照的移动副推杆、以及末端销毁器。默认脚本 `pusher`：光栅在推杆上游。光束被挡住时把伸出保持 0.9 秒，目标 120 mm，然后收回。

另外两份脚本也保存在项目里，并导出到 `scripts/`：

- `sequence.py`：先把名为 `Slider` 的关节推到 80，1.5 s 后收回并令 `Hinge` 转到 40 度。演示项目里没有这两个关节，换到自建场景再用。
- `counter.py`：光栅上升沿给整数信号 `parts_seen` 加一。

重新生成项目和这三份脚本（内容来自 `DemoScene`，避免和程序内字符串分叉）：

```bash
cmake --build --preset linux-system --target simstudio_demogen
./out/build/linux-system/simstudio_demogen samples/conveyor_line.simproj samples/scripts
```

无界面跑几秒并截图：

```bash
LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb \
  ./out/build/linux-system/src/app/simstudio \
  --open samples/conveyor_line.simproj --play --sim-seconds 3.4 \
  --screenshot conveyor_line.png --lang zh_CN
```
