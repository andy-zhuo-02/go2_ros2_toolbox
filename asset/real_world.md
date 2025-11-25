# Real World

## 📋 Prerequisites

> **⚠️ Note: The features of this repository have only been tested on the onboard expansion dock computer of Go2 EDU. Compatibility and functionality on other environments (like PC wired to Go2 dock computer) have not been verified.**

This toolbox is developed and tested on Unitree Go2 EDU with the expansion dock environment:

-**OS**: Ubuntu 20.04

-**ROS2**: Foxy

-**Firmware**: v1.1.7 (tested)

## 🛠️ Installation

### 1. Install Official Unitree ROS2 Package

First, install the official Unitree ROS2 package:

```bash

# Follow the official installation guide

# https://github.com/unitreerobotics/unitree_ros2

```

### 2. Install Dependencies

#### ROS2 Packages

```bash

sudoapt-getinstallros-foxy-navigation2\

                     ros-foxy-nav2-bringup \

ros-foxy-pcl-ros\

                     ros-foxy-tf-transformations \

ros-foxy-slam-toolbox

```

### 3. Build the Workspace

```bash

# Create workspace

mkdir-pgo2_ros2_ws/src

cdgo2_ros2_ws/src


# Clone repository

gitclonehttps://github.com/andy-zhuo-02/go2_ros2_toolbox.git


# Build

cd..

colconbuild

```

## 🎯 Usage

### Quick Start

```bash

# Source the workspace

sourceinstall/setup.bash


# Launch the robot

ros2launchgo2_corego2_startup.launch.py

```

### SLAM Operations

-**Map Serialization**: Save generated maps for later use

-**Map Deserialization**: Load previously saved maps

### Navigation

1. Open RViz2
2. Select the 'Navigation2 Goal' button
3. Click on the map to set navigation goals
4. Drag to adjust the target orientation

## 🔧 Development

### Frame Reference

| Frame          | Description     | Source                      |

| -------------- | --------------- | --------------------------- |

| `/odom`      | Odometry frame  | Unitree Go2 odometry topic  |

| `/map`       | Map frame       | SLAM Toolbox                |

| `/base_link` | Base link frame | Unitree Go2 odometry topic |

### ROS Topics

#### Publishers

| Component           | Topic                       | Type        | Frame     |

| ------------------- | --------------------------- | ----------- | --------- |

| Robot Pose          | `/utlidar/robot_pose`     | PoseStamped | `/odom` |

| LiDAR (Unitree)     | `/utlidar/cloud_deskewed` | PointCloud2 | `/odom` |

| LiDAR (Accumulated) | `/trans_cloud`            | PointCloud2 | `/odom` |

| Camera Image        | `/camera/image_raw`       | Image       | -         |

#### Subscribers

| Component        | Topic        | Type  | Frame        |

| ---------------- | ------------ | ----- | ------------ |

| Velocity Command | `/cmd_vel`   | Twist | `/base_link` |
