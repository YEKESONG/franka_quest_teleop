# franka_quest_teleop

Meta Quest 3S 手柄遥操 Franka **FR3** 的完整可迁移环境（ROS2 Humble + MoveIt Servo + Docker）。

已在真机双臂 FR3 上跑通：**绝对位置 + 相对姿态跟随，全程限速安全**。

## 快速开始 / 换设备重建
👉 见 **[MIGRATION.md](MIGRATION.md)**（版本对齐、编译顺序、踩坑修复、运行命令一应俱全）。

## 内容
- `docker_launch_files/` — Docker 构建配方
- `ws_franka_vr/src/` — 核心自研包 franka_vr（VR 桥接节点 + MoveIt Servo 伺服节点）+ Quest APK
- `ws_moveit2/src/` — MoveIt2 2.13.0 源码
- `ros2_ws/src/` — franka_ros2 v2.3.0 + franka_description 1.6.1
- `setup_env.sh` — 容器内一键 source

> 本仓库为**源码快照**，不含 `build/install/log`（换机器 `colcon build` 重新生成）。
