# 仿真环境（Simulation）

本节介绍如何基于 **Isaac Sim 4.5** 与 **Isaac Lab 2.1.0** 搭建 Unitree Go2 的仿真环境，便于在桌面端进行开发、调试与验证。

推荐环境如下：

- **操作系统**：Ubuntu 22.04  
- **ROS2**：Humble  
- **Isaac Sim**：4.5  
- **Isaac Lab**：2.1.0  

---

## 1. 安装 Isaac Sim 4.5

请按照 NVIDIA 官方文档安装 Isaac Sim：  
[Isaac Sim 4.5 安装指南](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)

安装完成后，假设 Isaac Sim 解压/安装在：

```bash
export ISAACSIM_PATH="${HOME}/isaacsim"
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

### 1.1 验证 Isaac Sim 安装

检查 Isaac Sim 图形界面是否可以正常启动：

```bash
# 可以加上 "--help" 查看支持的所有启动参数
${ISAACSIM_PATH}/isaac-sim.sh
```

检查 Isaac Sim 的 Python 接口是否可用：

```bash
# 检查 Python 环境是否正常
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"

# 检查是否可以通过 Python 启动示例场景
${ISAACSIM_PYTHON_EXE} ${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py
```

如果上述命令都能正常运行，说明 Isaac Sim 安装成功。

---

## 2. 安装 Isaac Lab 2.1.0

你可以参考 Isaac Lab 官方文档，或按下面的步骤进行安装：  
[Isaac Lab 安装指南](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html)

### 2.1 克隆 Isaac Lab 仓库并创建符号链接

```bash
cd ~
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# 创建指向 Isaac Sim 根目录的符号链接
ln -s ${ISAACSIM_PATH} _isaac_sim
```

### 2.2（可选）创建 Conda 环境

Isaac Lab 提供脚本快速创建 Conda 环境：

```bash
./isaaclab.sh --conda   # 默认环境名为 env_isaaclab

# 激活环境
conda activate env_isaaclab
```

你也可以根据自己的需要使用其他虚拟环境管理工具。

### 2.3 安装 Isaac Lab 依赖

```bash
./isaaclab.sh --install
```

该命令会安装 Isaac Lab 所需的扩展与依赖（包括强化学习框架等）。

### 2.4 验证 Isaac Lab 安装

```bash
python scripts/tutorials/00_sim/create_empty.py
```

若能成功弹出一个空场景窗口（类似下图），说明 Isaac Lab 安装完成：

![Verify IsaacLab](./verify_isaaclab.jpg)

---

## 3. 安装 ROS2 Humble

在仿真侧建议使用 **ROS2 Humble**，请按照官方文档进行安装：  
[ROS2 Humble 官方安装指南](https://docs.ros.org/en/humble/index.html)

安装完成后，记得根据需要配置好 `rosdep`、环境变量等。

---

## 4. 运行 Unitree Go2 仿真

当 Isaac Sim、Isaac Lab 与 ROS2 Humble 都正确安装后，可以使用开源项目 `isaac-go2-ros2` 启动 Go2 仿真：

```bash
# 激活 Isaac Lab 环境（若使用了 Conda）
conda activate env_isaaclab

# 克隆仿真仓库
git clone https://github.com/Zhefan-Xu/isaac-go2-ros2.git
cd isaac-go2-ros2

# 启动仿真
python isaac_go2_ros2.py
```

仿真场景加载完成后，你可以使用键盘对 Go2 进行遥操作。具体按键说明请参考该仓库的 README 或脚本说明。


