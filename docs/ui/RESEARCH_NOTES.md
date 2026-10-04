# 调研笔记 · 工业仿真与 AI 原生 UI（2026-10）

> 隐私说明：所有检索仅使用通用公开主题（产品名、UI 模式、标准），未在任何查询或外部服务中提及本项目或用户信息；未上传任何文件。
> 标注说明：**[已检索]** = 本次调研中检索/阅读到的来源；**[常识]** = 基于业内公开常识，本次未重新核实，落地前建议复核。

---

## 1. 工业仿真软件现状

### 1.1 Siemens Process Simulate（Tecnomatix / PDPS）
- 主窗口由菜单/工具栏（新版为 Ribbon）、对象树（Object Tree / Logical Collection）、操作树（Operation Tree）、图形视图（Graphic Viewer）、序列编辑器（Sequence Editor）、路径编辑器 / 碰撞查看器组成，视图可编组排列。[已检索] — TU Darmstadt Tecnomatix 课程：<http://www.lzm.maschinenbau.tu-darmstadt.de/kursunterlagen/tecnomatix_11/312_working_environment__graphical_user_interface.html>
- 运行仿真前需在操作树中“Set Current Operation”，然后在序列编辑器播放。[已检索] <https://www.lzm.maschinenbau.tu-darmstadt.de/kursunterlagen/tecnomatix_11/321_compound_operation.html>
- 序列编辑器自带甘特窗格（红色时间线），Gantt Viewer 显示操作与资源的持续时间与依赖。[已检索] <https://www.iim.maschinenbau.tu-darmstadt.de/kursunterlagen/tecnomatix_11/3121_sequence_editor.html>、<http://www.lzm.maschinenbau.tu-darmstadt.de/kursunterlagen/tecnomatix_11/2151_gantt_viewer.html>
- 官方培训显示功能面极广（放置操纵器、运动学、导轨/龙门/变位机、复合设备、对象流、事件、电影管理器、VR），基础课程 3.5 天以上，侧面说明学习曲线。[已检索] <https://training.plm.automation.siemens.com/ilt/iltdescription.cfm?pID=ILT046013_____TEC__17.0___B1105>
- Process Simulate Human 教程（2025）说明 Ribbon/工具栏组织方式与序列编辑器播放流程。[已检索] <https://assets.ctfassets.net/17si5cpawjzf/2FYGDma0qDVpp1h1gh8Oyi/100ab67cdcad428cade427521a4b17bd/Tutorial-Process-Simulate-Human-2408-30Jul2025.pdf>

**用户抱怨 / 痛点**
- 高端硬件上仍然很慢、CPU 占用低、调整设置无效。[已检索] <https://community.sw.siemens.com/s/question/0D5Vb00000OhCo5KAF/process-simulate-performance-problems>
- 仿真极慢、PSZ 加载 15 分钟。[已检索] <https://community.sw.siemens.com/s/question/0D54O000087gDU7SAM/simulation-proceeds-extremely-slowly>、<https://community.sw.siemens.com/s/question/0D54O00008B0zCCSAZ/process-simulate-psz-slow-loading-time-15-minutes>
- 大数据量崩溃、启动崩溃、PSZ 无效。[已检索] <https://community.sw.siemens.com/s/article/Process-Simulate-is-crashing-while-working-on-large-size-data>、<https://community.sw.siemens.com/s/article/Tecnomatix-Crash-on-launch>、<https://community.sw.siemens.com/s/question/0D54O000061xpKSSAY/process-simulate-psz-file-is-invalid>
- 机器人程序循环（REPEAT…UNTIL）会锁死 GUI，因为应用非多线程。[已检索] <https://community.sw.siemens.com/s/question/0D54O000061xo2aSAA/robot-program-locks-gui>
- 硬件选型讨论（GPU 饱和等）。[已检索] <https://www.reddit.com/r/Siemens/comments/199kq2l/process_simulation_hardware/>
- 虚拟调试实践讨论。[已检索] <https://www.robot-forum.com/robotforum/thread/38224-virtual-commissioning-with-tecnomatix-siemens-process-simulate/>

→ 设计启示：UI 永不阻塞、渐进加载、任务中心可取消、播放所选无需“设为当前”、功能通过命令面板而不是深层菜单发现。

### 1.2 Siemens AI：Process Simulate Copilot / Eigen Engineering Agent
- Process Simulate Copilot 内置于云端的 Process Simulate X，用于机器人路径优化、碰撞排查、基于文档生成 XML 模板与引导。[已检索] <https://blogs.sw.siemens.com/tecnomatix/stepping-into-the-future-with-siemens-process-simulate-copilot-ai-driven-simulation-video/>
- TIA Portal 的 AI 已演进为 **Eigen Engineering Agent**：从“给建议”转向“规划、执行、验证”的端到端代理，读取项目上下文、多步推理与自我校正，输出前验证。[已检索] <https://press.siemens.com/global/en/pressrelease/siemens-takes-ai-physical-world-next-level-two-new-eigen-engineering-agent>、<https://www.siemens.com/en-us/products/tia-portal/engineering-copilot-tia-standard/>、<https://assets.new.siemens.com/siemens/assets/api/uuid:7c20b450-d7b7-4e8b-9a02-cb4bde2c520c/HQCOPR202604187387EN.pdf>

→ 设计启示：行业方向是“代理执行 + 验证”。差异化：**本地、默认关闭、结构化命令、沙盒预演、整体撤销、可审计**，并且在 3D 中可视化预览（Siemens 方案为云端）。

### 1.3 NX Mechatronics Concept Designer（MCD）
- 新版改进：物理导航器按名称/类型/所属组件排序、查找所属组件、Fit View；信号映射对话框可按数据块/适配器限定范围并筛选信号。[已检索] <https://community.sw.siemens.com/s/article/mechatronics-concept-designer-usability-enhancements>
- 两种信号类型（信号适配器信号 vs 全局信号）在 Runtime Behavior 中类型不同（`Signal` vs `MCDSignal`），初学者易混淆，帮助文档对新手不友好。[已检索] <https://community.sw.siemens.com/s/question/0D54O000061xWGISA2/mcd-runtime-behavior-signals>
- MCD Player：只读播放 + 信号映射到 PLC / PLCSIM Advanced 用于虚拟调试。[已检索] <https://community.sw.siemens.com/s/article/mechatronics-concept-designer-mcd-player>
- 复杂物理（碰撞网格、过多刚体/关节）导致性能问题，官方建议使用基本形状碰撞体；学习曲线陡。[已检索] <https://community.sw.siemens.com/s/question/0D54O000061xNEcSAM/mechatronics-concept-designer-time-delay>、<https://digitaltwinproject.eu/digital-twin-and-nx-mcd-the-future-of-virtual-commissioning-in-manufacturing/>
- 第三方对比（realvirtual.io 论坛）。[已检索] <https://forum.realvirtual.io/communities/1/topics/11-what-is-the-difference-to-other-tools>

→ 设计启示：统一信号模型；自动碰撞代理；检查器显示碰撞代理精度；时间线 UI（MCD 也已有 Timeline UI 文章）。

### 1.4 Visual Components 4.x / 5.0
- 4.10：CADENAS 3Dfindit 库在 Modeling 页签；**引导式工作流（Guided workflow）** 可在 UI 中显示逐步说明；点云 LOD；统计不在预热期收集；SVG 图标支持。[已检索] <https://www.visualcomponents.com/wordpress/wp-content/uploads/2025/10/Visual-Components-4.10-release-notes-1.pdf>、<https://www.visualcomponents.com/wordpress/wp-content/uploads/2025/03/Visual-Components-4.10-release-notes.pdf>
- Process 页签的 Process Flow Editor、Process Executor（语句）用于快速搭建工艺流。[已检索] <https://academy.visualcomponents.com/app/uploads/2025/05/introduction-to-layout-configuration_pm_v2.pdf>
- Python API 中的统计（利用率、忙/闲）。[已检索] <https://help.visualcomponents.com/4.10/Premium/en/Python_API/vcStatistics.htm>
- **Visual Components 5.0（2026-03-12）**：统一界面、Python 3 API、更快碰撞检测、MQTT、MBD/PMI 生成路径、22 品牌后处理器，并明确为 AI 工作流做准备。[已检索] <https://www.visualcomponents.com/press/why-visual-components-5-0-could-redefine-digital-factory-planning/>、<https://help.visualcomponents.com/5.0/Professional/en/English/Release_Notes/release_notes_5.0.htm>

→ 设计启示：引导式工作流（我们在运动学编辑器中做“链有效性 + 引导”）；VC 强在组件库，我们没有预置组件 → 运动学编辑器必须极快，并支持“保存为设备类型”。

### 1.5 RoboDK
- 站点树（Station Tree）表达依赖关系（目标挂在参考坐标系下、工具挂在机器人下）。[已检索] <https://robodk.com/doc/en/Interface.html>
- Teach Target `Ctrl+T`、表面示教 `Ctrl+Shift+T`；目标分笛卡尔/关节两类。[已检索] <https://robodk.com/doc/en/Interface-Program-Menu.html>、<https://robodk.com/doc/en/Interface-Robot-Target.html>
- 5.7 起提供现代图标；工具栏含碰撞检查、快速仿真（按住空格）。[已检索] <https://robodk.com/doc/en/Basic-Guide-Toolbar-Menu.html>
- 链接目标生成无碰撞路径。[已检索] <https://robodk.com/doc/en/Collision-Avoidance-Link-Targets.html>

→ 采纳：`Ctrl+T` 示教、表面取点、目标类型显式。

### 1.6 NVIDIA Omniverse / Isaac Sim
- Stage 面板（USD 场景树）+ Property 面板（含 Raw USD 属性）；面板可拖拽停靠，布局可快速保存/加载或用 JSON 布局文件程序化加载。[已检索] <https://docs.isaacsim.omniverse.nvidia.com/latest/gui/reference_user_interface.html>、<https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.app.setup/docs/index.html>

→ 采纳：布局 JSON 导入导出、团队共享布局。

---

## 2. 通用 3D / 设计工具

### 2.1 Blender 4.x
- 工作区（Workspaces）作为顶栏页签，`Ctrl+PageUp/PageDown` 切换；`F3` 菜单搜索；`Q` 快速收藏；饼菜单。[已检索] <https://docs.blender.org/manual/en/4.3/interface/window_system/workspaces.html>、<https://docs.blender.org/manual/en/4.2/interface/controls/templates/operator_search.html>、<https://docs.blender.org/manual/en/4.3/interface/controls/buttons/menus.html>
→ 采纳：任务模式 = 工作区；命令搜索；径向菜单（右键长按）。

### 2.2 Unreal Engine 5 / Unity 6 编辑器
- Unity 6 Scene View Overlays：可停靠/浮动的视口工具条，基于 UI Toolkit，可自定义。[已检索] <https://docs.unity3d.com/6000.6/Documentation/Manual/overlays-custom.html>
- Unreal 编辑器：多标签停靠、细节面板（Details）、内容浏览器、Sequencer 时间线。[常识]
- Blueprint 可读性：执行流左→右、减少交叉、转接点（双击连线插入）、注释框分区、折叠为函数。[已检索] <https://github.com/JonasReich/OpenUnrealConventions/blob/master/Blueprint/README.md>、<https://jdn.dev/avoiding-blueprint-spaghetti/>、<https://dev.epicgames.com/documentation/en-us/unreal-engine/comments-in-unreal-engine>
→ 采纳：视口浮层工具条、节点图规范（转接点、编组、子图）。

### 2.3 Houdini 网络编辑器
- 节点标志位（旁路/显示/模板）、网络框 `Shift+O`、便签 `Shift+P`、可着色可最小化。[已检索] <https://www.sidefx.com/docs/houdini/network/flags.html>、<https://www.sidefx.com/docs/houdini/network/organize.html>、<https://www.sidefx.com/docs/houdini/network/shortcuts.html>
→ 采纳：便签、编组框、节点状态标志（断点/禁用）。

### 2.4 Fusion 360 / Onshape
- Fusion：`S` 键快捷工具箱（可搜索、按工作区区分、可固定），右键标记菜单（径向）。[已检索] <https://www.autodesk.com/products/fusion-360/blog/quick-tip-the-s-key/>、<https://www.autodesk.com/learn/ondemand/tutorial/using-the-marking-menu>
- Onshape：特征列表 + 回滚条（Roll to here / Roll to end）。[已检索] <https://cad.onshape.com/help/Content/PartStudio/features_and_parts_lists.htm>、<https://www.onshape.com/en/resource-center/tech-tips/tech-tip-roll-to-here-roll-to-end>
→ 采纳：径向菜单；“回滚条”思想用于 AI 活动记录与检查点（回到某一步之前的状态）。

### 2.5 Figma UI3 / Linear / Spline
- Figma UI3：画布居中，面板可调宽，`Shift+\` 最小化 UI，`Ctrl+\` 隐藏 UI，选中时属性面板临时展开。[已检索] <https://www.figma.com/blog/behind-our-redesign-ui3/>、<https://help.figma.com/hc/en-us/articles/23954856027159-Navigating-UI3>、<https://help.figma.com/hc/en-us/articles/41414918021271-Hide-or-minimize-the-UI>
- Linear：`Ctrl/Cmd+K` 命令菜单，按上下文分组、显示快捷键、模糊搜索、全键盘操作；上下文菜单的细节设计。[已检索] <https://linear.app/now/invisible-details>、<https://linearguide-6914.notaku.site/command-menu>、<https://uxpatterns.dev/patterns/advanced/command-palette.mdx>
- Spline：浏览器内 3D 设计工具，轻量的左侧对象/右侧属性 + 视口浮动工具条，玻璃质感深色 UI。[常识]
→ 采纳：最小化 UI、专注模式；命令面板显示快捷键；数值拖拽擦洗。

### 2.6 Apple visionOS / 空间 UI
- 系统玻璃材质随环境自适应，文字用 vibrancy 保证可读；“装饰件（ornaments）”浮在窗口前方用于持久控件；用 Z 轴深度建立层级，避免叠加浅色材质降低对比度。[已检索] <https://developer.apple.com/videos/play/wwdc2023/10076/>、<https://apple-docs.everest.mt/docs/design/human-interface-guidelines/materials/>、<https://apple-docs.everest.mt/docs/design/human-interface-guidelines/ornaments/>
→ 采纳：视口浮层统一“玻璃”材质（半透明 + 模糊 + 内高光），浮层不叠多层玻璃；为未来 XR 评审预留空间布局思路。

---

## 3. AI 原生交互

### 3.1 Cursor / VS Code Copilot
- Cursor Agent：变更边做边应用，逐文件内联 diff 接受/拒绝，Keep All / Undo All；**检查点**在重大修改前自动快照，可恢复。[已检索] <https://cursor.com/docs/agent/overview>、<https://cursor.com/help/ai-features/agent>
- Copilot **Next Edit Suggestions**：预测“下一处编辑的位置与内容”，`Tab` 跳转并接受；幽灵文本内联补全。[已检索] <https://code.visualstudio.com/blogs/2025/02/12/next-edit-suggestions>、<https://code.visualstudio.com/docs/editing/ai-powered-suggestions>
→ 采纳：计划卡 = diff 审阅；活动记录 = 检查点；“下一处修改”建议用于批量路径点修改（Tab 接受）。

### 3.2 代理 UX 模式（研究与行业指南）
- 计划先行（plan-then-execute）提高安全性与可控性。[已检索] <https://arxiv.org/pdf/2509.08646>
- 多步代理任务中确认频率的权衡（中间检查点优于只在结尾确认）。[已检索] <https://arxiv.org/html/2510.05307>
- 可人工干预的代理框架（暂停、编辑、接管）。[已检索] <https://aclanthology.org/2025.emnlp-demos.69.pdf>
- 企业代理设计模式：计划预览、风险分级审批、实时轨迹、回滚。[已检索] <https://www.aufaitux.com/blog/agentic-ai-design-patterns-enterprise-guide/>、<https://www.agentforgehub.com/posts/human-in-the-loop-agent-patterns>
→ 采纳：R0–R3 风险分级、权限面板、意图理解卡（含“不会修改的内容”）、执行过程轨迹、Esc 随时中止。

---

## 4. 工业 HMI 规范

- **ANSI/ISA-101.01-2015**：高性能 HMI；中性灰背景；常态用灰阶，饱和色只用于报警/异常；冗余编码（形状、文字、亮度），不单靠颜色。[已检索] <https://aulavirtual.fio.unam.edu.ar/pluginfile.php/407435/mod_folder/content/0/ISA101-01.pdf?forcedownload=1>
- Rockwell 过程 HMI 风格指南（浅灰 `#E0E0E0` 示例）。[已检索] <https://literature.rockwellautomation.com/idc/groups/literature/documents/wp/proces-wp023_-en-p.pdf>
- “Going Gray”与高性能 HMI 推广材料。[已检索] <https://control.com/technical-articles/going-gray/>、<https://grahamnasby.com/files_publications/NasbyG_2017_HighPerformanceHMIs_IntelligentWastewaterSeminar_WEAO_sept14-2017_slides-public.pdf>、<https://www.thecassandraproject.org/wp-content/uploads/2025/02/High-Performance-HMI-with-Control-HMI-Getting-Plant-Operations-Buy-In.pdf>
→ 采纳：信号表与状态着色遵循“正常安静、异常醒目”；深色主题以海军蓝中性色替代纯灰（工程软件而非控制室，但保留原则）。

---

## 5. 虚拟调试技术约束（影响 UI）

- S7-PLCSIM Advanced：时间同步模式（同步主调用 `StartProcessing(t)`，客户端到同步点冻结）；**虚拟时间缩放**（1 = 实时，0.5 = 半速，>1 加速）。[已检索] <https://docs.tia.siemens.cloud/r/simatic_s7_1500_et_200mp_manual_collection_enus_20/comprehensive-information/cpu-function-manuals/s7-plcsim-advanced-api/iinstance/virtual-and-real-time?contentId=CEu3u98Z1V6MK3UPDYcB4A>、<https://cache.industry.siemens.com/dl/files/691/109977691/att_1308933/v1/s7-plcsim_advanced_function_manual_en-US_en-US.pdf>
- OPC UA 用例：在 TIA 中启用 CPU OPC UA 服务器，通过全局 DB 交换标签，PLC 循环 OB 周期与模型周期匹配（如 20 ms）。[已检索] <https://cache.industry.siemens.com/dl/files/187/109749187/att_935977/v1/109749187_DIGI_Usecases_OPC_DOC_V10_en.pdf>
- NX MCD + PLCSIM Advanced 虚拟调试示例视频。[已检索] <https://www.youtube.com/watch?v=ZvrE2IfrfgI>
→ UI：连接卡显示同步模式/周期/倍速；“单周期”按钮；同步健康度曲线（抖动、超周期）。

---

## 6. 实现相关的许可证与技术

| 项 | 结论 | 来源 |
|---|---|---|
| Qt Advanced Docking System | **LGPL-2.1**（项目明确倾向 2.1 而非 3）；闭源可用，需动态链接/可替换、保留声明，修改需开源修改部分 | [已检索] <https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System/blob/master/README.md>、<https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System/issues/561>、<https://githubuser0xffff.github.io/Qt-Advanced-Docking-System/> |
| Qt Widgets vs Qt Quick | Widgets 适合传统复杂桌面 UI；Quick 适合流畅动画；`QQuickWidget` 增加离屏渲染并禁用线程化渲染；固定区域可用 `QQuickView` + `createWindowContainer`；与 `QOpenGLWidget` 同窗需同一图形 API | [已检索] <https://doc.qt.io/qt-6/qquickwidget.html>、<https://doc.qt.io/qt-6.10/qtquick-embeddedinwidgets-example.html>、<https://www.qt.io/blog/qt-quick-and-widgets-qt-6.4-edition> |
| HarmonyOS Sans | 可免费商用（含嵌入软件）；不得修改、不得单独再分发/出售字体；需保留版权与许可，并在软件中声明使用 | [已检索] <https://github.com/openharmony/global_system_resources/blob/master/LICENSE_Fonts>、<https://developer.huawei.com/consumer/cn/forum/topic/0202171228621328598> |
| Source Han Sans（思源黑体） | SIL Open Font License 1.1 | [常识] <https://github.com/adobe-fonts/source-han-sans> |
| Inter / JetBrains Mono | SIL OFL 1.1 | [常识] <https://github.com/rsms/inter>、<https://github.com/JetBrains/JetBrainsMono> |
| KDDockWidgets（ADS 备选） | GPL + KDAB 商业双许可，闭源需商业授权 | [常识，需核实] <https://github.com/KDAB/KDDockWidgets> |
| QtNodes（节点编辑器备选） | 需核实许可证与维护状态 | [常识，需核实] <https://github.com/paceholder/nodeeditor> |

---

## 7. 综合结论（驱动设计的 12 条洞察）

1. **性能与阻塞是现有工具的头号痛点** → 多线程架构必须在 UI 上“看得见”（任务中心、渐进加载、可取消）。
2. **操作链路长**（Set Current Operation 等） → 选择即上下文；“播放所选”。
3. **功能多但难以发现** → 命令注册表 + 命令面板 + 快捷键提示（Linear/Blender F3/Fusion S 键）。
4. **任务差异大** → 工作区/模式（Blender、Omniverse 布局）。
5. **视口被面板挤压** → 浮层工具条（Unity Overlays）、玻璃材质（visionOS）、最小化 UI（Figma UI3）。
6. **节点图易成“意大利面”** → 执行流左→右、类型着色、转接点、编组、子图（Blueprint/Houdini 规范）。
7. **信号模型混乱**（MCD 两类信号） → 单一信号模型 + 来源属性。
8. **设备建模依赖组件库**（VC） → 运动学编辑器一等公民 + 设备类型复用。
9. **行业 AI 走向“代理执行”**（Siemens Eigen） → 我们以“本地 + 结构化 + 预演 + 撤销 + 审计”差异化。
10. **AI 可信需要可视化预览** → 3D 幽灵叠加 + 指标对比 + 为什么（Cursor diff/检查点的工业版）。
11. **ISA-101 克制色彩** → 正常安静、异常醒目；AI 有独立“墨水色”，不与报警色混淆。
12. **虚拟调试需要时间确定性** → 同步模式、虚拟时间倍速、单周期、逻辑分析仪成为一等 UI。
