# 机器人目录

四条运动学都是参数加方盒，没有厂商 CAD。样本没有给出关节扭矩和连杆惯量，所以加速度来源是「未提供扭矩或惯量，节拍只受速度限制」，不再写 180°/s²。用户可以在关节上填写扭矩，或直接输入加速度；有质量时负载会按平行轴定理把加速度折低。每条的来源写在资源的 `source` 里，界面选中该机构时会显示。数字待用户对照原件核实。

| 名称 | 限位和速度 | 连杆原点 |
| --- | --- | --- |
| KR6_R900 | [KUKA KR AGILUS sixx 说明书 4.2 节](https://www.robostav.cz/cvut/robostav/download/KR_AGILUS_sixx_en.pdf)，[my.kuka.com 产品页](https://my.kuka.com/s/product/kr-6-r900-sixx/01t58000003eVQqAAM?language=en_US) | [kr6r900sixx_macro.xacro](https://github.com/ros-industrial/kuka_experimental/blob/indigo-devel/kuka_kr6_support/urdf/kr6r900sixx_macro.xacro) |
| IRB120 | [ABB 样本 ROB0149EN_D](https://library.e.abb.com/public/7aa0711a20fa41c49a8fdf3bbc3d5bb0/IRB120-Rev.J-ROBO149EN_D.pdf)，规格书 3HAC035960 | [irb120_3_58_macro.xacro](https://github.com/ros-industrial/abb/blob/kinetic-devel/abb_irb120_support/urdf/irb120_3_58_macro.xacro) |
| LRMate200iD | [FANUC America 产品页](https://www.fanucamerica.com/products/robot/lr-mate-200id) 给的是行程合计。带符号的限位用 xacro，其中 J3 为 −70° 到 +205°，跨度 275°，和产品页的 420° 不一致 | [lrmate200id_macro.xacro](https://github.com/ros-industrial/fanuc/blob/noetic-devel/fanuc_lrmate200id_support/urdf/lrmate200id_macro.xacro) |
| GP8 | [Motoman GP7/GP8 样本](https://www.motoman.com/getmedia/1a40ce78-99c3-4e43-bbce-b9318263f464/GP7_GP8.pdf.aspx)，速度与 [Yaskawa EU GP8](https://www.yaskawa.eu.com/robotics/robots/handling-mounting/productdetail/product/gp8_694) 一致。另一份转载把 L 写成 +150°/−65°、U 写成 +255°/−113°，目录没有采用 | [gp8_macro.xacro](https://github.com/ros-industrial/motoman/blob/kinetic-devel/motoman_gp8_support/urdf/gp8_macro.xacro) |

`--demo robots` 把四台一起放进场景，沿 X 每隔 1400 mm 一台。运动学页的「加入型号」只加入选中的那一台。
