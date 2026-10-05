# 资源导入

`IAssetImporter` 按扩展名匹配，扩展名对不上再看文件头。`createBuiltinRegistry()` 注册下面这些格式。

| 格式 | 实现 | 范围 |
| --- | --- | --- |
| STEP | OCCT | 现有装配读取 |
| IGES、BREP | OCCT | 读成一个零件网格 |
| STL | 自写 | ASCII 和二进制 |
| OBJ | 自写 | 三角形和多边形扇 |
| PLY | 自写 | ASCII 与 binary little endian 的顶点加面 |
| glTF / GLB | 自写 | 模式 4 三角形、float 位置、uint/ushort/ubyte 索引与关节、节点矩阵或平移旋转缩放、data URI 与 GLB。单位按米换算成毫米。读取 pbrMetallicRoughness（基色、金属度、粗糙度）并参与着色。蒙皮读取 JOINTS_0、WEIGHTS_0 和逆绑定矩阵，按线性混合蒙皮烘焙顶点。没有变形目标 |
| DXF | 自写 | ASCII 和二进制的 LINE、LWPOLYLINE、CIRCLE、ARC、SPLINE、INSERT、3DFACE、ELLIPSE、HATCH 边界、DIMENSION 的定义点和延伸点、图层。样条按均匀三次采样，不使用文件里的节点矢量。HATCH 只取边界顶点，不生成填充图案线。标注没有箭头和文字 |
| 图片 | 自写 | PNG/JPEG 字节写入项目 ZIP |
| URDF | 自写 | 连杆、转动/移动/固定/连续关节、盒子、圆柱、球体、STL/OBJ 网格视觉、惯性（质量、质心、惯量张量，千克和千克·米²）。网格按米再乘 scale 换成毫米。`package://` 不解析 |
| XML | 自写 | 本地名 product/part/component/item、process/operation/task/activity、plant/resource/station/robot/equipment。这不是某家产线软件的完整模式 |
| JT | 自写 | 8.x/9.x/10.x 文件头、TOC、zlib 与 XZ/LZMA 段、LSG 装配/变换/材质、量化 Tri-Strip（Null、Bitlength、Deering 法向）。算术编码和拓扑压缩不生成三角形。XT B-rep 不解码，回退到网格。不链接 JT Open Toolkit，也不链接 GPL/LGPL JT 库 |

导出：可见网格可写成二进制 STL 或 glTF/GLB（顶点烤成世界坐标，单位米）。节拍表仍是 CSV 和自写的 XLSX。
