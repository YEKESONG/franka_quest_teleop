# Meta Quest 3S 遥操 Franka FR3 —— 环境迁移与重建手册

本仓库是一套**已在真机 FR3（双臂）上跑通**的 Quest 3S VR 遥操方案的**源码快照**。
换新设备时，按本文档重建即可，无需再从零踩坑。

> 本快照**只含源码**，已剔除 `build/ install/ log/` 和各依赖的嵌套 `.git`。
> 换机器后用 `colcon build` 重新编译即可。

---

## 0. 硬件与前提

| 项 | 值 |
|---|---|
| 机器人 | Franka **FR3 双臂**（左臂 `172.16.0.2`，右臂 `172.16.0.3`；本方案默认遥操右臂） |
| 机器人服务器版本 | version **10**（决定了 libfranka 必须 ≥ 0.20） |
| VR 设备 | Meta **Quest 3S**（手柄遥操） |
| 宿主系统 | Ubuntu 22.04 + Docker + Docker Compose v2 |
| 容器基础 | ROS2 **Humble** |

---

## 1. 关键版本（务必对齐，版本错了会不兼容）

| 组件 | 版本 | 说明 |
|---|---|---|
| libfranka | **0.20.4** | 真机服务器 version 10 要求；用 0.13.x 会报 `Incompatible library version (server 10, library 7)` |
| franka_ros2 | **v2.3.0** | 与 libfranka 0.20.4 配套 |
| franka_description | **1.6.1** | franka_ros2 v2.3.0 的 dependency.repos 指定 |
| MoveIt2 | **2.13.0**（源码编译） | 必须源码编译新版；Humble apt 自带的 2.5.9 是旧 `servo.h` 架构，跑不了本方案的新 `moveit_servo`（`servo.hpp` / `getNextJointState` / `TwistCommand`） |
| moveit_servo | 随 MoveIt2 2.13.0 | 新 C++ 库 API |
| random_numbers | `ros2` 分支 | `geometric_shapes` 编译依赖，需单独源码克隆 |

---

## 2. 目录结构

```
franka_quest_teleop/
├── docker_launch_files/      # Docker 构建配方（Dockerfile + docker-compose + install 脚本）
│   ├── franka_ros2/          # ← 我们用这个（franka_ros2 分支）
│   └── franka_ros/           # ROS1 版，本方案没用到
├── setup_env.sh              # 容器内一键 source 所有工作区
├── ws_franka_vr/src/         # ★ 核心：本项目自研/改造的 franka_vr 包
├── ws_moveit2/src/           # MoveIt2 2.13.0 源码（6 个包）
└── ros2_ws/src/              # franka_ros2 v2.3.0 + franka_description 1.6.1
```

三个工作区在容器内挂载到 `/docker_volume/` 下，与宿主机双向同步。

---

## 3. 重建步骤（新机器）

### 3.1 起 Docker 容器
```bash
# 宿主机装好 Docker + Docker Compose v2
cd docker_launch_files
docker compose run franka_ros2        # 构建并进入容器
# 需要图形界面(RViz)时，宿主机先执行： xhost +local:docker
```
> 把本仓库三个工作区放到容器挂载的 `/docker_volume/` 下（或调整 compose 的挂载路径指向本仓库）。

### 3.2 编译顺序（**顺序不能乱**，后者依赖前者）
```bash
source /opt/ros/humble/setup.bash

# (1) franka_ros2 v2.3.0
cd /docker_volume/ros2_ws
colcon build
source install/setup.bash

# (2) MoveIt2 2.13.0（最重，双核约需数小时，可挂夜里跑）
cd /docker_volume/ws_moveit2
#   若缺 random_numbers：
#   git clone https://github.com/moveit/random_numbers -b ros2 src/random_numbers
#   （并清掉 build/geometric_shapes 缓存后重编）
colcon build --parallel-workers 2      # 内存小就降到 2 线程
source install/setup.bash

# (3) franka_vr（本项目）
cd /docker_volume/ws_franka_vr
colcon build --packages-select franka_vr
source install/setup.bash
```

### 3.3 曾经踩过的坑（新机器如复现，照此修）
- **`franka_semantic_components` 找不到 `controller_interface/helpers.hpp`**
  → 在 `franka_semantic_components/CMakeLists.txt` 的 `THIS_PACKAGE_INCLUDE_DEPENDS`
    和 `package.xml` 里加上 `controller_interface`。
    （franka_ros2 v2.3.0 本身可能已包含，先直接编译，报错再补。）
- **`geometric_shapes` 需要 `random_numbers::random_numbers` target**
  → 见 3.2 (2) 里克隆 `random_numbers -b ros2`。
- **Eigen 三元表达式 `?:` 类型报错**（`franka_vr_vel.cpp`）→ 已改成 `if` 赋值，本快照已修好。

---

## 4. 运行（真机遥操）

```bash
# 每个终端都先：
source /docker_volume/setup_env.sh

# 终端1：拉起机器人 + MoveIt Servo + 控制器 + 夹爪（遥操右臂 172.16.0.3）
ros2 launch franka_vr franka_twist.launch.py \
     robot_ip:=172.16.0.3 use_fake_hardware:=false load_gripper:=true

# 终端2：Quest 桥接节点
cd /docker_volume/ws_franka_vr/src/franka_vr/oculus_reader
python3 oculus_reader/start_franka_vr.py
```

### Quest 3S 准备
1. `adb` 连上头显（USB），首次需在头显里点 **允许调试授权**。
2. 安装遥操 APK：`adb install ws_franka_vr/src/franka_vr/oculus_reader/oculus_reader/APK/teleop-debug.apk`
3. `start_franka_vr.py` 通过 adb 从 APK 读手柄位姿+按键。

### 操作
- **右扳机 (rightTrig) > 0.1**：进入遥操跟随（松开即停）。
- **右握把 (rightGrip)**：> 0.6 关夹爪，< 0.4 开夹爪。

---

## 5. 本方案的遥操控制逻辑（相对原开源版的改造）

- **位置**：绝对映射——手柄在 `fr3_link0` 系下的绝对位置 = 夹爪目标位置。
- **姿态**：相对增量——按扳机瞬间记录手柄+夹爪姿态基准，之后
  `夹爪目标姿态 = (手柄相对转动) ⊗ 夹爪基准`（按下瞬间不翻）。
- **全程限速**（`franka_vr_vel.cpp`）：线速度 `min(3.0×误差, 0.08 m/s)`、
  角速度 `min(1.0×角误差, 0.3 rad/s)`——根治"按下猛冲触发 reflex"。
  > 标定对齐好后可把 `v_lin_max`（当前 0.08）往上调，提升响应速度。
- **跳变/追踪保护**：单帧位移 > 0.08m 丢弃；右手柄旋转矩阵 `det≈1` 才发布。
- **待办**：`oculus_base` 静态变换（x/y/z/rpy）与 scale 的精细标定，
  目标是"手柄碰桌面 = 夹爪碰桌面"的绝对对齐。绝对映射对**追踪稳定性**极敏感，
  标定前务必先保证头显佩戴稳定、手柄始终在头显相机视野内。

---

## 6. 组件来源（致谢 / 可追溯）

| 组件 | 来源 |
|---|---|
| Docker 环境骨架 | fork 自 `Fjakob/libfranka-docker`（经 `ZorAttC/libfranka-docker`） |
| franka_vr 包 + Quest APK + oculus_reader | 原开源项目 `franka_vr`（本仓库在其基础上做了兼容性移植与安全化改造） |
| libfranka / franka_ros2 / franka_description | Franka Robotics 官方 |
| MoveIt2 / moveit_servo | MoveIt 官方 |
