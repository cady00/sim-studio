# .simproj 项目格式

当前版本 6，格式标识 `simstudio.project`。扩展名 `.simproj`。读取器接受版本 1 到 6。版本 1 没有机构、信号和脚本字段，打开后这些内容为空。版本 2 没有运动学、逻辑块和路径。版本 3 没有碰撞集和凸分解缓存，打开后近距阈值用默认 5 mm，碰撞集为空。版本 4 的节点没有 `faceAxes`，打开后该数组为空。版本 5 没有外部轴字段，打开后关节不是外部轴，冗余模式是固定，路径锁住外部轴。

文件是 **store-only ZIP**（压缩方法 0，UTF-8 文件名标志，CRC32）。不使用 ZIP64，单个成员建议小于 4 GiB。读取器拒绝方法 8，因此不要用通用压缩工具重新打包。

## 成员

| 路径 | 内容 |
| --- | --- |
| `project.json` | UTF-8 JSON |
| `meshes/<id>.bin` | 显示网格，魔数 `SIMMESH1` |

`project.json`：

```json
{
  "format": "simstudio.project",
  "version": 2,
  "name": "sample_fixture",
  "units": "mm",
  "upAxis": "Z",
  "sources": ["/absolute/path/sample_fixture.step"],
  "root": { }
}
```

`sources` 只记录导入路径，不嵌入原始 STEP。换机器打开项目仍能看到网格；要重新导入需自行提供源文件。

节点：

```json
{
  "id": "n1",
  "name": "Pin_A",
  "kind": "part",
  "visible": true,
  "color": [0.57, 0.58, 0.95],
  "transform": {
    "translation": [30, 25, 12],
    "rotation": [1, 0, 0, 0],
    "scale": [1, 1, 1]
  },
  "mesh": "m1",
  "children": []
}
```

`kind` 为 `assembly` 或 `part`。`rotation` 是四元数 `[w, x, y, z]`。平移单位为毫米。多个节点可以引用同一个 `mesh` id。

根节点 id 固定为 `root`，名称为 `World`。界面按当前语言显示「世界」。

## 网格二进制

小端：

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 | 8 字节 | `SIMMESH1` |
| 8 | uint32 | 版本，必须为 1 |
| 12 | uint32 | 顶点数 |
| 16 | uint32 | 索引数 |
| 20 | float32 × 顶点数 × 3 | 位置 xyz |
| 之后 | float32 × 顶点数 × 3 | 法线 xyz |
| 之后 | uint32 × 索引数 | 三角形索引 |

网格是细分结果，不是 B-rep。项目文件不能恢复精确曲面。

## 版本 2：机构、信号、脚本

`project.json` 增加三个可选字段：

```json
{
  "activeScript": "pusher",
  "signals": [
    {"name": "barrier", "type": "bool", "value": false},
    {"name": "barrier_distance", "type": "real", "value": 280}
  ],
  "scripts": [
    {"name": "pusher", "source": "def on_step(dt):\n    pass\n"}
  ]
}
```

`signals[].type` 为 `bool`、`int` 或 `real`。`value` 按类型是布尔、整数或实数。仿真过程中传感器和脚本改写的实时值不把项目标成已修改；停止或复位后，保存下去的是停止时刻场景里的信号值。

节点在 `kind != none` 时带 `mech`。字段对所有机构类型都写出，用不到的保持默认：

| 字段 | 含义 |
| --- | --- |
| `kind` | `none` `rigid` `collision` `fixed` `hinge` `slider` `conveyor` `collisionSensor` `distanceSensor` `lightBarrier` `source` `sink` |
| `mass` / `autoMass` | 质量（kg）。自动质量按碰撞体积 × 900 kg/m³，下限 0.05 kg |
| `kinematic` | 运动学刚体。输送面和碰撞体在求解器里按运动学处理 |
| `shape` | `box` `sphere` `capsule` `convex`。凸包取节点网格的顶点 |
| `halfExtents` | 盒子半长，毫米 |
| `radius` / `height` | 球或胶囊，毫米 |
| `other` | 关节另一刚体的节点名。空则锚在世界 |
| `axis` | 关节轴、光栅方向，节点局部坐标 |
| `limited` / `lower` / `upper` | 移动副单位 mm，转动副单位度 |
| `drive` | `none` `position` `speed` |
| `target` / `maxSpeed` / `maxAccel` | 位置目标或速度目标。移动副 mm 与 mm/s，转动副度与度/s |
| `direction` / `speed` | 输送面方向（局部）和带速 mm/s |
| `range` | 距离传感器和光栅的量程，毫米 |
| `signal` / `valueSignal` | 布尔信号名，以及距离的实数信号名 |
| `interval` / `prototype` | 发生器间隔（秒，最小 0.05）和原型节点名 |

运行中由发生器克隆出来的零件带 `transient` 标记，不写入项目。停止或复位会删掉它们，并把非临时节点的局部变换恢复到按下「运行」之前。

## 版本 3：运动学、逻辑块、路径

`project.json` 增加可选数组 `kinematics`、`logic`、`paths`。转动副的位置是度，移动副是毫米。耦合关节的位置由主关节乘 `coupleRatio` 得到，不单独插值。逻辑块的 `python` 只是随项目保存的文本。机构库 `.simlib` 是另一份 JSON（`format` 为 `simstudio.library`），只含资源、逻辑和路径，不含网格。

## 版本 4：碰撞集与凸分解缓存

`project.json` 增加可选对象 `proximity` 和可选数组 `convexCache`。旧文件没有这两项时，近距阈值为 5 mm，集合为空。

`proximity`：

```json
{
  "clearanceMm": 15,
  "sets": [
    {"name": "movers", "objects": ["OverlapA", "ClearA"]},
    {"name": "fixtures", "objects": ["OverlapB", "ClearB"]}
  ],
  "pairs": [["movers", "fixtures"]]
}
```

`objects` 是零件节点名。`pairs` 的每一项是两个碰撞集的名字。集合为空时，查询按机器人相关配对（连杆对非相邻连杆，或连杆对外面的零件）过滤；集合非空时只查这些对。

`convexCache` 按网格 id 保存局部坐标的凸片，避免每次打开都分解：

```json
[{"mesh": "mesh-1", "fingerprint": 1, "pieces": [[[0, 0, 0], [1, 0, 0]]]}]
```

`fingerprint` 由顶点坐标算出。网格变了就会重算。符号距离网格不写入项目。界面目前只编辑前两个碰撞集和第一对，文件里可以有更多。

## 版本 5：圆柱面轴线

零件节点可以带可选数组 `faceAxes`。旧文件没有这项时为空。每项是导入时从圆柱面或平面上的圆提取的轴线，坐标在该零件网格的局部系，单位毫米：

```json
"faceAxes": [
  {"origin": [0, 0, 120], "direction": [0, 0, 1], "radius": 40}
]
```

运动学编辑器用它做「圆柱面定轴」和按轴分组。它不是 B-rep，改了网格也不会自动重算。

## 版本 6：外部轴

运动学关节可以带 `external`（`E1`…`E6` 或空）和 `externalKind`（`track`、`positioner` 或空）。连杆可以带 `mountedOn` 和 `mount`（安装到另一根连杆上的静止变换，中间没有关节）。命名坐标系可以带 `link`，空表示链的末端。资源可以带 `redundancy`：

```json
"redundancy": {"mode": "fixed", "travel": 1, "midrange": 0.2, "manipulability": 0.2, "fixed": [0]}
```

`mode` 是 `fixed`、`travel`、`midrange` 或 `manipulability`。路径可以带 `external`（`locked`、`sync`、`positioner`）和 `workpiece`。路径点可以带 `external` 和 `externalValues`。旧文件没有这些字段时按上面的默认值。说明见 [EXTERNAL.md](EXTERNAL.md)。
