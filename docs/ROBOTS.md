# 机器人目录

四条运动学的限位和连杆原点来自公开页面，没有厂商 CAD 授权。样本没有给出关节扭矩和连杆惯量，所以加速度来源是「未提供扭矩或惯量，节拍只受速度限制」，不再写 180°/s²。用户可以在关节上填写扭矩，或直接输入加速度；有质量时负载会按平行轴定理把加速度折低。每条的来源写在资源的 `source` 里，界面选中该机构时会显示。数字待用户对照原件核实。

连杆网格用 ROS-Industrial 仓库里的 STL。各包的 `package.xml` 写了 BSD 或 Apache-2.0，但没有把网格再分发单独写清楚，ABB 的说明还写惯量来自厂商提供的 CAD。这些文件不放进本仓库。首次加入型号时从下面的公开地址下载到 `~/.local/share/SimStudio/catalog-meshes`（可用 `XDG_DATA_HOME`）。`SIM_CATALOG_MESH_DIR` 指向已有目录时只读那个目录。`SIM_CATALOG_MESH_OFFLINE=1` 不访问网络。某个连杆下载失败时，该连杆仍是方盒。视觉原点在这些 xacro 里是单位阵，STL 按米读入后乘 1000。

| 型号 | 网格 | 地址 |
| --- | --- | --- |
| KR6_R900 | 碰撞 STL。底座、1、3、5、6 来自 `kr6_agilus`，2 和 4 来自 `kr6r900sixx`。视觉是 DAE，没有采用 | `kuka_experimental` 的 `indigo-devel`，`kuka_kr6_support/meshes/` |
| IRB120 | `irb120_3_58/visual` 的 `base_link` 与 `link_1`…`link_6` | `abb` 的 `kinetic-devel` |
| LRMate200iD | `lrmate200id/visual` 的 `base_link` 与 `link_1`…`link_6` | `fanuc` 的 `noetic-devel` |
| GP8 | `meshes/visual/gp8_base_link` 与 `gp8_link_1_s`、`_2_l`、`_3_u`、`_4_r`、`_5_b`、`_6_t` | `motoman` 的 `noetic-devel` |

没有找到 KR 280 R3080 的公开网格。双机演示里的第二台保持橙色方盒。用户可以自己导入 STEP。GP8 法兰在 T 轴前方 80 mm，没有对应的公开网格，这一小段仍是方盒。

| 名称 | 限位和速度 | 连杆原点 |
| --- | --- | --- |
| KR6_R900 | [KUKA KR AGILUS sixx 说明书 4.2 节](https://www.robostav.cz/cvut/robostav/download/KR_AGILUS_sixx_en.pdf)，[my.kuka.com 产品页](https://my.kuka.com/s/product/kr-6-r900-sixx/01t58000003eVQqAAM?language=en_US) | [kr6r900sixx_macro.xacro](https://github.com/ros-industrial/kuka_experimental/blob/indigo-devel/kuka_kr6_support/urdf/kr6r900sixx_macro.xacro) |
| IRB120 | [ABB 样本 ROB0149EN_D](https://library.e.abb.com/public/7aa0711a20fa41c49a8fdf3bbc3d5bb0/IRB120-Rev.J-ROBO149EN_D.pdf)，规格书 3HAC035960 | [irb120_3_58_macro.xacro](https://github.com/ros-industrial/abb/blob/kinetic-devel/abb_irb120_support/urdf/irb120_3_58_macro.xacro) |
| LRMate200iD | [FANUC America 产品页](https://www.fanucamerica.com/products/robot/lr-mate-200id) 给的是行程合计。带符号的限位用 xacro，其中 J3 为 −70° 到 +205°，跨度 275°，和产品页的 420° 不一致 | [lrmate200id_macro.xacro](https://github.com/ros-industrial/fanuc/blob/noetic-devel/fanuc_lrmate200id_support/urdf/lrmate200id_macro.xacro) |
| GP8 | [Motoman GP7/GP8 样本](https://www.motoman.com/getmedia/1a40ce78-99c3-4e43-bbce-b9318263f464/GP7_GP8.pdf.aspx)，速度与 [Yaskawa EU GP8](https://www.yaskawa.eu.com/robotics/robots/handling-mounting/productdetail/product/gp8_694) 一致。另一份转载把 L 写成 +150°/−65°、U 写成 +255°/−113°，目录没有采用 | [gp8_macro.xacro](https://github.com/ros-industrial/motoman/blob/kinetic-devel/motoman_gp8_support/urdf/gp8_macro.xacro) |

`--demo robots` 把四台一起放进场景，沿 X 每隔 1400 mm 一台。运动学页的「加入型号」只加入选中的那一台。

## 加速度

这四条没有公开的关节扭矩，也没有公开的连杆惯量张量，所以关节的加速度来源是速度限制：梯形规划只受上表里的 `maxSpeed` 约束，不再写 180°/s²。用户可以在关节上填写扭矩，或直接输入加速度。填写了扭矩、并且连杆有质量时，加速度改为扭矩除以惯量；法兰上的负载按平行轴定理把加速度折低。界面上的来源分别是「动力学」「用户」和「未提供扭矩或惯量，节拍只受速度限制」。

导入 URDF 或 xacro 时，若 `<limit effort>` 和连杆质量都在，该关节的来源就是动力学，视觉网格随连杆进入机器人资源。

欢迎页的「打开示例工程」使用 IRB 120。法兰负载 3 kg 取自样本里的额定负载。六根臂关节仍是速度限制。旁边的夹具是一根移动关节，质量 8 kg、推力 40 N，加速度由这两项估成 5000 mm/s²，来源标成动力学。
