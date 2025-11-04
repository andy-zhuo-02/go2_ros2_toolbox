# Go2 ROS2 Toolbox

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-green.svg)](https://docs.ros.org/en/foxy/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg)](https://ubuntu.com/)

[🇨🇳 中文版 README](./README_zh.md)

A comprehensive ROS2 toolbox for Unitree Go2 EDU robot, providing SLAM and navigation capabilities for autonomous operation.

If you find this project helpful, please give it a Star ⭐️ to support us!

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

## 🚀 Features

- **LiDAR Integration**: Real-time point cloud processing and accumulation
- **Camera Support**: GStreamer-based camera capture and streaming
- **SLAM Capabilities**: Integration with SLAM Toolbox for mapping
- **Navigation Stack**: Full Navigation2 integration for autonomous navigation
- **ROS2 Native**: Built specifically for ROS2 Foxy ecosystem

## 📋 Prerequisites

> **⚠️ Note: The features of this repository have only been tested on the onboard expansion dock computer of Go2 EDU. Compatibility and functionality on other environments (like PC wired to Go2 dock computer) have not been verified.**

This toolbox is developed and tested on Unitree Go2 EDU with the expansion dock environment:

- **OS**: Ubuntu 20.04
- **ROS2**: Foxy
- **Firmware**: v1.1.7 (tested)

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
sudo apt-get install ros-foxy-navigation2 \
                     ros-foxy-nav2-bringup \
                     ros-foxy-pcl-ros \
                     ros-foxy-tf-transformations \
                     ros-foxy-slam-toolbox
```

### 3. Build the Workspace

```bash
# Create workspace
mkdir -p go2_ros2_ws/src
cd go2_ros2_ws/src

# Clone repository
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git

# Build
cd ..
colcon build
```

## 🎯 Usage

### Quick Start

```bash
# Source the workspace
source install/setup.bash

# Launch the robot
ros2 launch go2_core go2_startup.launch.py
```

### SLAM Operations

- **Map Serialization**: Save generated maps for later use
- **Map Deserialization**: Load previously saved maps

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
| `/base_link` | Base link frame | Unitree Go2 odometry topic |

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

## 🤝 Contributing

We welcome contributions! Please feel free to submit issues and pull requests.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Unitree Robotics for the Go2 EDU platform
- ROS2 community for the excellent navigation and SLAM tools
- Contributors and users of this toolbox

## 📞 Support

If you encounter any issues or have questions, please:

1. Check the [Issues](https://github.com/andy-zhuo-02/go2_ros2_toolbox/issues) page
2. Create a new issue with detailed information
3. Include system information and error logs

---

**Note**: This is an unofficial toolbox and is not affiliated with Unitree Robotics.

## Simulation
This simulation is developed  with the environment:

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **Isaac Sim**: 4.5
- **Isaac Lab**: 2.1.0

### Setup enviorment

**Step 0:** Install [Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) (Download and extract contents in `${HOME}/isaacsim`)

#### Verifying the Isaac Sim installation
```bash
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/isaacsim"
# Isaac Sim python executable
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

Check that the simulator runs as expected:

```bash
# note: you can pass the argument "--help" to see all arguments possible.
${ISAACSIM_PATH}/isaac-sim.sh
```

Check that the simulator runs from a standalone python script:

```bash
# checks that python path is set correctly
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"
# checks that Isaac Sim can be launched from python
${ISAACSIM_PYTHON_EXE} ${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py
```

**Step 1:** Install Isaac Lab 2.1.0 


### Installing Isaac Lab

You can follow the [IsaacLab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#verifying-the-isaac-sim-installation) or just follow the turtorial below

Clone the Isaac Lab repository into your project’s workspace:

```bash
./isaaclab.sh --help

usage: isaaclab.sh [-h] [-i] [-f] [-p] [-s] [-t] [-o] [-v] [-d] [-n] [-c] -- Utility to manage Isaac Lab.

optional arguments:
   -h, --help           Display the help content.
   -i, --install [LIB]  Install the extensions inside Isaac Lab and learning frameworks (rl_games, rsl_rl, sb3, skrl) as extra dependencies. Default is 'all'.
   -f, --format         Run pre-commit to format the code and check lints.
   -p, --python         Run the python executable provided by Isaac Sim or virtual environment (if active).
   -s, --sim            Run the simulator executable (isaac-sim.sh) provided by Isaac Sim.
   -t, --test           Run all python pytest tests.
   -o, --docker         Run the docker container helper script (docker/container.sh).
   -v, --vscode         Generate the VSCode settings file from template.
   -d, --docs           Build the documentation from source using sphinx.
   -n, --new            Create a new external project or internal task from template.
   -c, --conda [NAME]   Create the conda environment for Isaac Lab. Default name is 'env_isaaclab'.
   -u, --uv [NAME]      Create the uv environment for Isaac Lab. Default name is 'env_isaaclab'.
```

#### Creating the Isaac Sim Symbolic Link

Set up a symbolic link between the installed Isaac Sim root folder and _isaac_sim in the Isaac Lab directory. This makes it convenient to index the python modules and look for extensions shipped with Isaac Sim.

```bash 
# enter the cloned repository
cd IsaacLab
# create a symbolic link
ln -s ${ISAACSIM_PATH} _isaac_sim

ln -s ${HOME}/isaacsim _isaac_sim

```

#### Creating a Conda Environment (optional)

```bash
#Default environment name 'env_isaaclab'
./isaaclab.sh --conda
### Activate environment
conda activate env_isaaclab 
```

#### Installation

```bash
./isaaclab.sh --install
```

#### Verifying the Isaac Lab installation

```bash
python scripts/tutorials/00_sim/create_empty.py
```
![Verify IsaacLab](asset/verify_isaaclab.jpg)

If you see this, congratulations!



