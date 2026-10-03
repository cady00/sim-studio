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
