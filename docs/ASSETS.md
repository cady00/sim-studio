# 资源导入

`IAssetImporter` 按扩展名匹配，扩展名对不上再看文件头。`createBuiltinRegistry()` 注册下面这些格式。

| 格式 | 实现 | 范围 |
| --- | --- | --- |
| STEP | OCCT | 现有装配读取 |
| IGES、BREP | OCCT | 读成一个零件网格 |
| STL | 自写 | ASCII 和二进制 |
| OBJ | 自写 | 三角形和多边形扇 |
| PLY | 自写 | ASCII 与 binary little endian 的顶点加面 |
| glTF / GLB | 自写 | 模式 4 三角形、float 位置、uint/ushort 索引、节点矩阵或平移旋转缩放、data URI 与 GLB。单位按米换算成毫米。没有皮肤、变形、PBR |
| DXF | 自写 | ASCII 和二进制的 LINE、LWPOLYLINE、CIRCLE、ARC、SPLINE、INSERT、图层。样条按均匀三次采样，不使用文件里的节点矢量。没有 3DFACE、ELLIPSE、HATCH、标注 |
| 图片 | 自写 | PNG/JPEG 字节写入项目 ZIP |
| URDF | 自写 | 连杆、转动/移动/固定/连续关节、盒子视觉。网格文件视觉不展开。长度按米换算成毫米 |
| XML | 自写 | 本地名 product/part/component/item、process/operation/task/activity、plant/resource/station/robot/equipment。这不是某家产线软件的完整模式 |
| JT | 插件槽 | 不解码。需要商业 JT Open Toolkit，或自行替换名为 `jt` 的导入器。不链接 GPL JT 库 |

导出：可见网格可写成二进制 STL 或 glTF/GLB（顶点烤成世界坐标，单位米）。节拍表仍是 CSV 和自写的 XLSX。
