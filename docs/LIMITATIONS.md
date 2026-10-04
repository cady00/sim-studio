# 局限与未实现路径

这份清单来自一次全仓检索：空函数体、`TODO` / `FIXME`、`not implemented`、`placeholder`、导入器直接失败、以及会把通用数字当成数据表的路径。能接上真实行为的已经接上。其余条目保持失败或明确拒绝，不返回假数据。

## 这次补上的空路径

- `MainWindow::runScriptSnippet` 原先是空函数，脚本停靠栏也没有入口。现在「运行片段」把编辑器里的源码交给 `IScriptHost::execute`，标准输出和异常写进脚本日志。场景里还没有脚本时会先建一条名为 `snippet` 的脚本。
- 运动学面板「建议连杆」原先对每根关节都套 `RevoluteLimit` 的缺省值（±180°、90°/s）。资源名包含 `kr420` 且建议出 6 个转动关节时，改为 `kr420DatasheetLimits()`（A1–A6 的公开行程和速度，见 `docs/KR420.md`）。

## 保持失败，不伪装成功

| 路径 | 实际行为 |
| --- | --- |
| JT（`JtImporter`） | `probeContent` 恒为假。`importFile` 返回失败，说明本构建没有解码器，也不链接 Siemens JT Open Toolkit。 |
| IGES | `createImporter` 只接受 `step` / `stp` / `obj`。`iges` 得到空指针。 |
| 二进制 DXF | 读到二进制头时返回 `binary DXF is not supported`。 |
| 压缩 ZIP | 工程包只接受存储法（method 0）。压缩条目报错，不假装解压成功。 |
| `nl_to_logic`、`layout_suggest`、`cycle_optimize`、`explain_anomaly` | 返回 `is a documented hook and is not implemented`。单元测试锁定这句。 |
| AI 提供方 | `complete` 不发网络请求。未启用、无端点、或传输未编译时返回错误和空 JSON。`capabilities()` 为空。 |
| `IRcsAdapter` | 只有纯虚接口，没有厂商 RCS 实现。规划走内置求解器。 |
| PLCSIM Advanced | 卡片只显示地址和「本机没有安装运行时」。不打开套接字。 |
| OPC UA 互通 | 自研客户端只和本仓库的回环服务器对测。没有 open62541 互通，也没有 PLCSIM 连接。 |
| 通用 PPR / eMS XML | 只覆盖本程序写出的产品、工艺、工厂结构，不是完整 eMS 模式。 |

## 算法和数据用了简化，调用方看得到边界

- 建议连杆写上的关节加速度一律是 180°/s²，不是某份数据表。非 KR420 的建议限位仍是 ±180°、90°/s。
- `addKr420TrackDemonstration` 里滑台的速度和加速度是 1500 mm/s 与 1500 mm/s²。公开 KL 数据表没有用在这两个数上，源码注释已标明。
- DXF：直线、圆弧、圆、多段线、均匀三次样条。没有填充图案线，没有尺寸箭头和文字。嵌套 INSERT 不递归。
- glTF：没有形态目标。
- URDF：不解析 `package://`。网格按米读入后再乘 1000 和缩放，变成毫米。
- 图像嵌入是 ZIP 里的字节，不是 GIS。
- 焊点自旋搜索不对每个转角做碰撞，只在选中的姿态上做一次接近检查。修正建议是一次正逆解试探。
- 连杆包络是网格包围盒角点沿关节样本（步长不超过 20°）的凸包。没有连杆网格时，球腕模型用四段解析臂杆。不是三角面沿螺旋运动的连续扫掠。
- 避障是关节空间双向 RRT。直线段已无碰撞时结果就是起点和终点。连接失败会退回直线段。
- KUKA：有 PTP、LIN、SPL、SLIN、SPLINE / ENDSPLINE、`$TOOL` / `$BASE`（XYZABC）、INTERRUPT 和 IO。没有完整 BAS，没有工具标定数据。中断文本会保存并再次写出，仿真不执行中断程序。`$VEL_AXIS` 不作为指令往返。
- ABB：`confdata` 置零，未使用的外部轴写成 `9E9`。
- FANUC：只有 ASCII LS，没有二进制 TP。
- 齿轮、凸轮、电子凸轮、位置/速度控制器、限位、气缸、抓手、发生器和收集器只在内置求解器里积分。Jolt 不跑这些装置。
- Jolt 没有沿曲面骑行，网格碰撞体是凸包。网格对网格、连续碰撞和多点接触在内置求解器里。没有重写 Jolt 的 CCD。
- 甘特图拖动色条不建立前后约束。
- OPC UA Basic256Sha256 的对称 IV 是每条消息明文前置的 16 字节随机数。证书是 RSA-2048 自签。
- 撤销目前只恢复上一次意图条记下的姿态和信号。场景里的结构编辑还没有撤销栈（后续批次补）。
- 十万零件压测（llvmpipe、MSAA 4、1440×900）：提交 10 万实例、2560 万三角形时整帧约 2938 ms（0.34 fps），RSS 约 1156 MB；近处视锥后约 81 ms（12.3 fps）。平铺场的软件遮挡是 0；贴墙擦过遮挡 17877。GPU 视锥剔除是一次 430 路计算调度，不是每帧路径。
- 不生成 Windows 安装包。KR420 STEP 留在 `testdata/local/`，不进版本库。
