# 与 MCD 装置、信号的对应

这份说明对的是 Siemens Mechatronics Concept Designer 里常见的装置和信号，以及本仓库已经能仿真的那一层。教学样件放在 `testdata/local/mcd/`，`.gitignore` 已经盖住 `testdata/local/`，不进版本库。更大的 NX `.prt`（流水线机器人约 30 MB 等）还没到，这一轮以 STEP 和已经在目录里的小件为准。

`--demo mcd` 仍是原来的装置演示（齿轮、凸轮、气缸、抓手）。`--demo mcd-compat` 是新的兼容演示：光电、气缸、吸盘接到同一张软 PLC。码垛龙门的 STEP 不嵌进这个演示，用 `--import` 单独读进场景。

## 装置

场景里每个节点有一份 `Mech`，种类是 `MechKind`（`src/core/include/sim/core/Mech.hpp`）。信号是 `Signal`，类型只有布尔、整数、实数。

| MCD 里常见的叫法 | 本仓库 | 兼容演示里怎么用 |
| --- | --- | --- |
| 刚体 | `MechKind::Rigid`，运动 `BodyMotion::Free` 或 `Surface` | 工件一开始是 `Surface`，停在输送面上。吸盘吸住后改成 `Free`，跟着吸盘走 |
| 碰撞体 | `MechKind::Collision` | 地面 |
| 固定 / 铰链 / 滑块 / 球铰 | `FixedJoint`、`Hinge`、`Slider`、`Ball` | 这一条演示没用关节 |
| 输送面 | `MechKind::Conveyor`，方向、速度、布尔 `signal` | 节点 `Conveyor`，信号 `belt_run`，速度 180 mm/s |
| 气缸 | `MechKind::Cylinder`，`lower` / `upper` 毫米，`axis`，布尔 `signal`，`throttle`，`cushion` | 节点 `Cylinder`，轴 −Z，行程 0..70 mm，最大速度 350 mm/s，缓冲 0.04 s，信号 `extend` |
| 吸盘 / 抓手 | `MechKind::Gripper`，布尔 `graspSignal`，`graspRadius` | 节点 `Suction`，信号 `vacuum`，半径 36 mm。父节点是气缸，局部平移 (0, 0, −40)，吸盘跟着杆走 |
| 光电 / 光栅 | `MechKind::LightBarrier`，轴、量程、布尔 `signal` | 节点 `Photoeye`，轴 Y，量程 220 mm，信号 `photo` |
| 碰撞传感器、距离传感器 | `CollisionSensor`、`DistanceSensor` | 这一条演示没用 |
| 发生器、收集器、限位 | `Source`、`Sink`、`Limit` | 这一条演示没用 |

气缸和抓手只在内置求解器里积分。Jolt 不跑这些装置，所以兼容演示强制用内置物理。

抓手吸半径内最近的自由刚体。停在输送面上的刚体（`Surface`，或已经骑在带上）也可以吸：吸上的那一拍改成 `Free`，清掉贴带，之后每步把工件放在吸盘当时的位置加上吸住时的偏移。真空撤销后松开，工件重新吃重力。兼容演示里吸盘是气缸的子节点，气缸缩回时工件跟着抬高，离开光电的射线。

## 信号和软 PLC

设备信号和梯形图地址是两张表，靠 `softPlc.bindings` 接上。方向 `in` 是设备进 PLC，`out` 是 PLC 写出到设备。扫描在物理步进之前，所以光电要到下一步才能进梯形图。

| 设备信号 | 地址 | 方向 | 含义 |
| --- | --- | --- | --- |
| `photo` | `I0.0` | in | 光电被挡住 |
| `extend` | `Q0.0` | out | 气缸伸出命令 |
| `belt_run` | `Q0.1` | out | 输送运行许可 |
| `vacuum` | `Q0.2` | out | 吸盘真空 |
| `seen` | `M0.0` | out | 光电锁存，给示波器和信号表看 |
| `parts` | `C0` | out | 计数完成位 |

梯形图：

- `I0.0` 上升沿置位 `M0.0`，并给 `C0` 加一（预置 1）。
- `I0.0` 下降沿复位 `M0.0`。
- `T0` 是 `I0.0` 的接通延时，预置 0.02 s。
- `Q0.0` = `T0` 与 `M0.0`，且 `M1.0` 为假。`M1.0` 是抬起命令。
- `T1` 是 `Q0.0` 的接通延时，预置 0.40 s。伸出 70 mm、最大速度 350 mm/s，0.40 s 时活塞已经靠在终点，吸盘才允许吸。
- `T1` 置位 `Q0.2`。这是锁存，不是跟着 `T1` 掉下去的线圈，所以气缸开始缩回时真空还在。
- `T2` 是 `Q0.2` 的接通延时，预置 0.12 s。到点置位 `M1.0`，`Q0.0` 落下，气缸缩回，工件被抬离带面。
- `Q0.1` = `I0.0` 常闭且 `M0.0` 常闭。工件离开光电后锁存解开，输送再走。

伸出过程位 `气缸_Extending` 不是线圈写的。扫描里若某条节拍规则监视 `资源名_Extending`，而该名字对得上一个气缸，或者对不上但场景里只有一个气缸，就把这个位置位：伸出命令为真，并且活塞离指令终点还超过 1 mm。命令读输出映射线圈，比设备信号新一拍。演示的气缸节点叫 `Cylinder`，资源名是中文「气缸」，走的是「场景里只有一个气缸」这条。缓冲让杆渐近终点，1 mm 大约是这条气缸 0.3 s 量级的收口。

## 节拍行

`softPlc.cycles` 按资源分行。兼容演示三行：

| 资源 | 工序名 | 开始 / 结束 |
| --- | --- | --- |
| 输送 | 锁存 | `M0.0` 上升沿到下降沿 |
| 气缸 | 伸出 | `气缸_Extending` 上升沿到下降沿 |
| 吸盘 | 吸取 | `Q0.2` 上升沿到下降沿 |

布局标题是「MCD 兼容」。无头截图打开软 PLC 停靠栏，甘特图藏起来。视口旁注读同一拍的光电、气缸指令行程、吸盘、工件高度和输送：光电开/关、气缸毫米、吸盘开/关、工件毫米、输送走/停。工件抬过光电射线之后，旁注里光电是关，气缸接近 0 mm，吸盘是开。

## 几何样件

码垛龙门 `palletizer/Cartesian Gantry Palletizing.stp` 是 `ISO-10303-21`，AP214，`FILE_NAME` 为 `Cartesian Gantry Palletizing`，时间 `2018-08-28T18:54:57+07:00`。Open CASCADE 的 STEP 导入器能读出真实网格。读入时有一条未解析引用的警告，导入仍然成功，三角形数不为 0。同一份几何还在 `palletizer/源模型.zip` 里，另外有一份约 85 MB 的 `.igs` 和几张预览图。这一轮没有解出那份 IGES。

同目录和 `cylinder/`、`suction/`、`path6/`、`stamping/` 里的 `.prt` 都读不了：

- `气缸举升.prt`、`气缸控制.prt`、`传送带.prt`、`使用吸盘吸取产品.prt`、`机器人按路径运动.prt`、`冲头.prt`、`流水线.prt`、`流水线冲压件实例.prt` 的文件头是 `SPLMSSTR`（Siemens PLM 流）。
- `工件.prt`、`毛坯.prt`、`盒子.prt`、`传感器-1.prt`、`传感器-2.prt`、`光电传感器.prt` 是 OLE 复合文档（`D0 CF 11 E0`）。
- 两者都不是 `ISO-10303-21`。用现有 STEP 导入器打开吸盘样件会失败，三角形数是 0，不生成占位网格。
- `cylinder/气缸、液压缸.zip` 里的条目名是 GBK，没有 UTF-8 标志。这一轮没有解进场景。

失败记录也写在 `docs/LIMITATIONS.md`。

要从 NX 得到能导入的文件：打开 `.prt` 后用 File → Export → STEP，应用协议选 AP214（或 AP203），写出 `.stp`。不要用 File → Save 把 `.prt` 另存一下就当成交了，那仍然是 Siemens 的部件文件。本程序读 `ISO-10303-21`。`源模型.zip` 里已经有一份这样导出的码垛龙门 STEP，还有同名 `.igs`。教学包里没有 JT。`cylinder/`、`suction/`、`path6/`、`stamping/` 里没有 STEP 或 JT。

`testdata/local/` 里另外两份 STEP 不在 MCD 包内，导入器同样能读，也都不进版本库。`KR420_R3080_KR280_R3080.stp` 导入后按圆柱拟合分成 7 段连杆。`KR6_R600_Z200_X.stp` 这一次导入是 15 个节点、12 个网格、308978 个三角形。
