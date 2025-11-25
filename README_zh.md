# Go2 ROS2 Toolbox（中文说明）


> 本文件为[英文原版 README](./README.md)的中文翻译，若有疑问请参考原文。

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-green.svg)](https://docs.ros.org/en/foxy/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg)](https://ubuntu.com/)

本项目是为 Unitree Go2 EDU 机器人开发的 ROS2 工具箱，提供 SLAM 与导航能力，实现自主运行。

如果觉得本项目有用，请点个 Star ⭐️ 支持一下！

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

## 🚀 功能特性

- **激光雷达集成**：实时点云处理与累积
- **相机支持**：基于 GStreamer 的相机采集与推流
- **SLAM 能力**：集成 SLAM Toolbox 进行建图
- **导航系统**：完整集成 Navigation2 实现自主导航
- **原生 ROS2**：专为 ROS2 Foxy 生态构建

## 🖥️ 仿真环境（Simulation）

我们为 Unitree Go2 EDU 提供了基于 Isaac Sim 与 Isaac Lab 的仿真环境，便于在桌面环境中进行开发与调试。

- **操作系统**：Ubuntu 22.04  
- **ROS2**：Humble  
- **Isaac Sim**：4.5  
- **Isaac Lab**：2.1.0  

完整的仿真环境安装与使用说明请参考文档：  
👉 [仿真环境说明（Simulation，中文）](./asset/simulation_zh.md)

## 🤖 实机环境（Real World）

实机（真实机器人）部署与使用说明已拆分到单独文档中，涵盖前置条件、安装步骤、使用方法与开发说明等内容。

请参考：  
👉 [实机环境说明（Real World，中文）](./asset/real_world_zh.md)

当前项目在以下真实环境中开发与测试：

- **操作系统**：Ubuntu 20.04
- **ROS2**：Foxy
- **固件**：v1.1.7（已测试）

## 🤝 贡献

欢迎贡献代码！请随时提交 issue 或 pull request。

## 📄 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。

## 🙏 鸣谢

- Unitree Robotics 提供 Go2 EDU 平台
- ROS2 社区的导航与 SLAM 工具
- 本工具箱的贡献者与用户

## 📞 支持

如遇问题或有疑问，请：

1. 查看 [Issues](https://github.com/andy-zhuo-02/go2_ros2_toolbox/issues) 页面
2. 创建新 issue 并详细描述问题
3. 附上系统信息与错误日志

---

**注意**：本工具箱为非官方项目，与 Unitree Robotics 无直接关联。
