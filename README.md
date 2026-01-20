[中文指南](./README_CN.md) | [English Guide](./README.md)
# ROS-SLAM-AutoExploration

## A ROS-based robot project for autonomous exploration and SLAM mapping.

This project utilizes the **TurtleBot3 Burger** robot platform within the **Gazebo** simulation environment to implement an autonomous exploration and mapping algorithm based on **frontier detection**. The robot autonomously explores the entire terrain, constructs a complete and accurate 2D map, and visualizes the process in real-time via **RViz**. Upon completion, the generated map is verified through fixed-point navigation tasks.

## Project Overview

This project aims to solve the problem of autonomous perception and mapping for mobile robots in unknown environments. The system consists of three core functional modules:

1. **SLAM Mapping**: Real-time construction of 2D grid maps using **Gmapping**.
2. **Autonomous Decision Making**: A Python-based decision node that uses the `make_plan` service for path pre-validation, combined with a "blacklist" mechanism and anti-stuck strategies to ensure efficient and robust exploration.
3. **Fixed-point Navigation**: Utilizes **AMCL** for localization and the **Move_base** navigation stack (DWA algorithm) for obstacle avoidance and optimal path planning.

## Key Features

1. **Autonomous Exploration**: Automatically identifies map **Frontiers** and plans paths to explore unknown areas.
2. **Intelligent Obstacle Avoidance**: Integrated with `move_base`, utilizing **Costmaps** to handle static obstacles and inflation layers.
3. **Path Pre-validation**: Invokes the global planner before navigation to pre-validate paths, automatically correcting unreachable goals to prevent the robot from getting stuck near walls.
4. **Exception Recovery**: Features a timeout detection mechanism that triggers in-place rotation recovery behaviors when the robot is trapped in a dead corner.
5. **Visual Monitoring**: Provides complete **RViz** configuration for real-time display of LiDAR scans, planned paths, maps, and goals.

## Development Environment

* **OS**: Ubuntu 20.04 LTS
* **ROS Version**: ROS Noetic Ninjemys
* **Simulation**: Gazebo 11
* **Languages**: Python 3, XML (Launch)

## File Structure

```text
catkin_ws2/                       # ROS Workspace Root
├── build/                        # Build files
├── devel/                        # Development setup & object files
└── src/                          # Source code
    ├── CMakeLists.txt            # Top-level CMake file
    │
    ├── my_project/               # Main Project Package
    │   ├── launch/               # Launch scripts folder
    │   │   ├── run_my_project.launch   # Scripts to launch simulation, nav, and exploration
    │   │   └── run_nav.launch          # Scripts for fixed-point navigation based on saved maps
    │   ├── maps/                 # Map storage folder
    │   │   ├── map1.pgm          # Saved map image after exploration
    │   │   └── map1.yaml         # Map configuration file
    │   ├── src/                  # Source code folder
    │   │   └── robot_control.py  # Autonomous exploration logic script (Python)
    │   ├── rviz/                 # Visualization configuration
    │   │   └── my_rviz.rviz      # Preset RViz view configuration
    │   ├── worlds/               # Simulation environment folder
    │   │   └── world4.world      # Gazebo physics world file
    │   ├── CMakeLists.txt        # Build rules
    │   └── package.xml           # Dependency description
    │
    └── turtlebot3_navigation/    # [Tuned Dependency] Navigation params (costmap, planner)

```

# Usage Guide

## Prerequisites

This project is developed based on **Ubuntu 20.04** and **ROS Noetic**. Before running the code, please ensure the following dependencies are installed.

### 1. Basic Environment

* **OS**: Ubuntu 20.04 LTS
* **ROS**: Noetic Ninjemys
* **Python**: 3.8+

### 2. Install ROS Packages

Open a terminal and execute the following command to install the official dependencies required for TurtleBot3 simulation, navigation, and mapping:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3 \
                     ros-noetic-turtlebot3-msgs \
                     ros-noetic-turtlebot3-simulations \
                     ros-noetic-navigation \
                     ros-noetic-gmapping \
                     ros-noetic-map-server \
                     ros-noetic-move-base \
                     ros-noetic-dwa-local-planner

```

## Running the Project

### (1) Setup & Compile

Open a terminal, navigate to the workspace directory `catkin_ws2`, and compile the project:

```bash
cd ~/catkin_ws2
catkin_make

```

Source the environment variables:

```bash
source devel/setup.bash

```

Grant execution permission to the Python exploration script:

```bash
chmod +x src/my_project/src/robot_control.py

```

### (2) Launch Simulation & Navigation

Execute the launch command in the terminal:

```bash
roslaunch my_project run_my_project.launch

```

> **Note:** If running correctly, the **Gazebo** simulation window will open showing the robot and the world model. The **RViz** visualization window will also open, displaying the real-time map and LiDAR scans starting from the origin.

### (3) Run Autonomous Exploration Script

Open a **new terminal** and load the environment variables:

```bash
source devel/setup.bash

```

Run the script:

```bash
python3 ~/catkin_ws2/src/my_project/src/robot_control.py

```

> **Note:** If running correctly, the robot will start exploring the terrain autonomously. In RViz, you will see a **green circular goal point** and the robot's **red trajectory** until exploration is complete.

### (4) Save the Map

Once exploration is finished, open a **new terminal** to save the generated map into the `maps` folder (naming it `my_map1`):

```bash
rosrun map_server map_saver -f ~/catkin_ws2/src/my_project/maps/my_map1

```

### (5) Navigation with Saved Map

Close the terminals from the previous steps. Open a **new terminal** and launch the fixed-point navigation file:

```bash
roslaunch my_project run_nav.launch

```

**Instructions:**

1. If running correctly, Gazebo and RViz windows will open. RViz will display the map you just saved.
2. Select **2D Pose Estimate** in RViz to calibrate the robot's position based on its actual pose in Gazebo.
3. Select **2D Nav Goal** in RViz, set a target position and orientation. The robot will automatically navigate to the target.

## Parameter Configuration

* **Inflation Radius**: Adjusted to **0.15** in `costmap_common_params_burger.yaml` to adapt to narrow terrains.
* **Goal Tolerance**: `xy_goal_tolerance` is set to **0.2 meters** in `dwa_local_planner_params_burger.yaml` to improve the success rate of goal verification.

## Developer Info

* **Author**: Alikayu
* **Affiliation**: Dalian Maritime University, College of Marine Electrical Engineering, Automation Major
* **Contact**: liangyt0111@163.com
* **Date**: January 2026

## Disclaimer

This project is for personal practice and learning purposes. Feedback and suggestions are welcome. Commercial use is strictly prohibited without permission. For any questions, please contact the author.
