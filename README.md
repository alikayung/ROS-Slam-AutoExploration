# ROS-Slam-AutoExploration
## A ROS-based robot project for autonomous exploration and SLAM mapping.
## 基于ROS的机器人自主探索与建图项目
项目运用现有机器人平台TurtleBot3 Burger，在Gazebo仿真环境中，构建机器人基于前沿点的自主探索建图的算法，使机器人自主地探索完成整个world地形，构建出完整正确的地图，通过RViz可视化窗口显示机器人探索建图过程和效果。完成地图构建后，利用此地图，实现机器人去往图中任意位置的定点导航，并以此验证建图的准确性。

## 项目简介
本项目旨在解决移动机器人在未知环境下的自主感知与建图问题。系统核心包含三大功能模块：
1.使用Gmapping SLAM实时构建 2D 栅格地图。
2.基于 Python 编写的决策节点，利用 make_plan 服务进行路径预校验，配合黑名单机制与防卡死策略，实现了高效、鲁棒的自主探索。
3.定点导航：利用 AMCL 定位与 Move_base 导航栈（DWA算法），实现避障与最优路径规划。

## 主要功能
1.自主探索：机器人自动识别地图边界（Frontier），规划路径探索未知区域 。
2.智能避障：集成 move_base，利用代价地图（Costmap）处理静态障碍物与膨胀层 。
3.路径预校验：在导航前调用全局规划器预演路径，自动修正不可达目标点，防止机器人卡死在墙内 。
4.异常恢复：具备超时检测机制，当机器人陷入死角时自动触发原地旋转恢复行为 。
5.可视化监控：提供完整的 RViz 配置，实时显示雷达扫描、规划路径、地图与目标点 。

## 开发环境
操作系统: Ubuntu 20.04 LTS
ROS 版本: ROS Noetic Ninjemys
仿真平台: Gazebo 11
编程语言: Python 3, XML (Launch)

## 文件结构
catkin_ws2/                   # ROS 工作空间根目录
├── build/                    # 编译生成文件
├── devel/                    # 开发环境配置与目标文件
└── src/                      # 源代码目录
    ├── CMakeLists.txt       # 顶层 CMake 文件
    │
    ├── my_project/          # 主程序文件夹
    │   ├── launch/          # 启动脚本文件夹
    │       │── run_my_project.launch  # 启动仿真、导航和探索节点的脚本
    │       └── run_nav.launch          # 根据地图启动定点导航的脚本
    │   ├── maps/               # 地图存储文件夹
    │   │   ├── map1.pgm        # 探索完成后保存的地图图片
    │   │   └── map1.yaml       # 地图的配置文件
    │   ├── src/                # 源代码文件夹
    │   │   └── robot_control.py  # 自主探索逻辑 Python 脚本
    │   ├── rviz/               # 可视化配置文件夹
    │   │   └── my_rviz.rviz   # 预设的 RViz 视图配置
    │   ├── worlds/            # 仿真环境文件夹
    │   │   └── world4.world  # Gazebo 物理仿真环境文件
    │   ├── CMakeLists.txt    # 编译规则文件
    │   └── package.xml       # 依赖描述文件
    │
    └── turtlebot3_navigation/  # [本设计调参后的依赖包] 导航参数配置 (costmap, planner)

## 使用指南
（1）环境配置与编译
打开终端，进入工作空间目录catkin_ws2，编译项目：
``` cd ~/catkin_ws2 ```
catkin_make
刷新环境变量
source devel/setup.bash
为 Python 自主探索脚本赋予可执行权限
chmod +x src/my_project/src/robot_control.py

（2）启动仿真环境与导航系统
在终端中执行启动命令：
roslaunch my_project run_my_project.launch
若运行正常，可发现gazebo仿真窗口打开，机器人与世界模型出现，RViz可视化窗口打开，显示出在原点处实时识别出的地图和雷达扫描线等。

（3）运行自主探索控制脚本
打开新的终端，加载环境变量：
source devel/setup.bash
运行脚本：
python3 ~/catkin_ws2/src/my_project/src/robot_control.py
若运行正常，可发现机器人开始在地形中自主探索，RViz窗口中显示出绿色圆形目标点和机器人红色轨迹，直至探索完成。

（4）保存地图
探索完成后，打开新终端，保存探索出的地图在maps文件夹中：（这里命名为my_map1）
rosrun map_server map_saver -f ~/catkin_ws2/src/my_project/maps/my_map1

（5）根据地图实现定点导航
关掉运行的终端，打开新终端，启动定点导航的launch文件。
roslaunch my_project run_nav.launch
若运行正常，可看到gazebo窗口和RViz窗口打开，RViz窗口中显示的是探索完成的地图。
选中RViz窗口中的2D Pose Estimate，根据gazebo中机器人的位姿校正位置。
选中RViz窗口中的2D Nav Goal，选定目标点和方向，机器人会自动导航到目标点。

## 参数配置
膨胀半径 (Inflation Radius): 在 costmap_common_params_burger.yaml 中，为了适应狭窄地形，膨胀半径已优化为 0.15 。
目标容差: 在 dwa_local_planner_params_burger.yaml 中，xy_goal_tolerance 设置为 0.2 米，以提高到达判定成功率 。

## 开发者信息
作者: 梁雅天 
单位: 大连海事大学 船舶电气工程学院 自动化专业 
日期: 2026年1月

## 本项目为个人练习项目，欢迎提出意见和建议。未经允许严禁用于商业用途，如有疑问请联系作者
