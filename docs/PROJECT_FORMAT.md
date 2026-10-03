# .simproj 项目格式

版本 1，格式标识 `simstudio.project`。扩展名 `.simproj`。

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
  "version": 1,
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

网格是细分结果，不是 B-rep。Phase 1 不能从项目文件恢复精确曲面。
