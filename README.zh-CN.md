# franka_quest_teleop

[English](README.md) · **简体中文**

用 **Meta Quest 3S** 手柄遥操 **Franka FR3** 双臂——一套完整、可复现的 ROS 2 环境。
clone 下来、编一次、就能跑。

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-required-2496ED?logo=docker&logoColor=white)
![Robot](https://img.shields.io/badge/Robot-Franka%20FR3-000000)
![Status](https://img.shields.io/badge/status-真机已验证-success)

已在真机双臂 FR3 上跑通：绝对位姿跟随、接入误差恒为 0、全程限速，
并用 Smith 预测器消掉了延迟诱发的转腕共振。

```
Quest 3S 手柄 ──adb logcat──> 桥接节点 ──set_target_pose──> 外环速度控制
   ──MoveIt Servo (TWIST)──> ros2_control ──libfranka──> FR3 双臂
```

---

## 目录

- [这个仓库里有什么](#这个仓库里有什么)
- [系统结构](#系统结构)
- [前置条件](#前置条件)
- [快速开始](#快速开始)
- [使用](#使用)
- [仓库结构](#仓库结构)
- [版本对照表](#版本对照表)
- [两个最容易误会的点](#两个最容易误会的点)
- [文档](#文档)
- [排障](#排障)
- [致谢](#致谢)
- [许可](#许可)

---

## 这个仓库里有什么

复现这条通路所需的一切，都在这一个仓库里：

| | |
|---|---|
| **源码** | 遥操包本体——外环笛卡尔控制节点、双臂 launch、全部配置、Quest 桥接及其 APK——加上全部第三方工作区，版本锁到具体 commit 并随仓库分发 |
| **环境** | 一份 Dockerfile 构建出完整运行时：ROS 2 Humble、libfranka 0.20.4、MoveIt 2 的编译依赖、adb，以及桥接脚本的 Python 依赖 |
| **知识** | 带逐段验证命令的结构说明、真机上实际踩过的排障清单，以及一份完整的控制调优复盘 |

**能复现出什么**

- 用 Quest 手柄绝对位姿跟随双臂，`Grip` 键控夹爪
- 无冲击接入：按 `Enter` 时把目标重锚到当前末端位姿，接入误差恒为 0，因此不存在标定文件
- 一套经得起真机的控制环：速度前馈、消掉延迟诱发的 ~2 Hz 转腕共振的 Smith 预测器、
  最短弧四元数误差、反馈闭在真机实测状态上、全程限速限加速度
- 同一套 launch 也能跑单臂（`active_arm:=left|right|both`）
- mock 硬件模式，没有机器人也能开发

**它不是什么** —— 不是通用 VR 遥操框架。它针对的是服务器 version 10 的 FR3 + Quest 手柄，
调参也只对这个组合成立。

仓库根目录在容器里被挂载为 `/docker_volume`，所以文档、脚本、launch 文件里写的容器内路径，
和仓库里的相对路径是一一对应的。

## 系统结构

```
 ┌─ Quest 3S 头显 ────────────────────────────────────────────────┐
 │  teleop-debug.apk —— 以 tag=wE9ryARX 往 logcat 打              │
 │  手柄位姿 4x4 矩阵 + 按键                                       │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ① USB / adb logcat -T 0 -s wE9ryARX:I   (~70 Hz)
                ▼
 ┌─ 桥接节点 start_franka_vr_dual.py ─────────────────────────────┐
 │  解析 → ×SCALE → 转到 ROS 坐标系                                │
 │  广播 TF: world → oculus_base(静态) → oculus_{left,right}       │
 │  接入即重锚: target ≡ 当前末端位姿, 接入误差恒 0                 │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ② 服务 /{left,right}/set_target_pose  (70 Hz)
                │    旁路诊断话题 /{left,right}/debug_target
                ▼
 ┌─ 外环 demo_franka_vr_vel (每臂一个) ───────────────────────────┐
 │  200 Hz: 位姿误差 → 笛卡尔速度 twist                            │
 │  速度前馈 + PD + Smith 预测器 (T = 0.09 s)                      │
 │  最短弧四元数 + 2·vec 对数映射 + 限速限加速度                    │
 │  反馈闭在【真机实测状态】上, 不是内部模型                        │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ③ MoveIt Servo (TWIST) → 雅可比伪逆 → 关节
                ▼
 ┌─ ros2_control —— JointTrajectoryController, effort 接口 ───────┐
 │  每关节 PD (基座关节硬 p=600, 腕关节软 p=50)                     │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ④ franka_hardware → libfranka 0.20.4 → 1 kHz 实时环
                ▼
              FR3 双臂 (左 172.16.0.2 / 右 172.16.0.3)
                │ ⑤ 关节状态回读 (30 Hz) ──► 回到外环当反馈
```

每一段的验证命令见 [docs/architecture.md](docs/architecture.md)。

## 前置条件

| | 要求 |
|---|---|
| **机器人** | Franka FR3（单臂或双臂），服务器 **version 10** |
| **VR** | Meta Quest 3S + 手柄，一根**能传数据**的 USB 线 |
| **宿主系统** | Ubuntu 22.04，Docker + Docker Compose v2 |
| **网络** | 宿主机能连到机械臂（默认 `172.16.0.2` / `172.16.0.3`） |
| **内存** | 最低 16 GB —— MoveIt 2 要源码编译 |
| **磁盘** | 镜像加编译产物约 25 GB |

## 快速开始

```bash
git clone https://github.com/YEKESONG/franka_quest_teleop.git
cd franka_quest_teleop

# 1 —— 构建镜像并起一个常驻容器
xhost +local:docker                                              # RViz 要显示就得先执行
docker compose -f docker_launch_files/docker-compose.yml build
docker compose -f docker_launch_files/docker-compose.yml up -d
docker exec -it franka_dev bash                                  # 要几个终端就 exec 几次

# 2 —— 编译四个工作区（第一次要几小时，MoveIt 2 占大头）
bash /docker_volume/scripts/build_all.sh -j 2                    # 瓶颈是内存，-j 别调大

# 3 —— 准备头显（宿主机侧，一次性）
bash scripts/setup_quest_adb.sh                                  # 戴上头显，点【一律允许】
```

> **容器名**：compose 默认把容器命名为 `franka_dev`。这个名字被占用时改名：
> `FRANKA_CONTAINER_NAME=franka_dev2 docker compose ... up -d`。

## 使用

两个终端，都 `docker exec` 进**同一个**容器。

```bash
# 终端 1 —— 机械臂 + MoveIt Servo + 夹爪 + RViz
bash /docker_volume/scripts/run_arm_stack.sh --real              # 双臂
bash /docker_volume/scripts/run_arm_stack.sh --real --arm right  # 只起右臂

# 终端 2 —— Quest 桥接（必须前台跑在交互终端里，脚本要读单键）
bash /docker_volume/scripts/run_vr_bridge.sh --arm right
```

去掉 `--real` 就是 mock 硬件（不碰真机）。

### 操作

| 输入 | 作用 |
|---|---|
| `Enter` | 遥操开 / 关。接入瞬间自动重锚：目标 ≡ 当前末端位姿，接入误差恒为 0 |
| `1` / `2` | 遥操进行中就地重新对零 左 / 右臂，不用断开 |
| 手柄 `Grip` | `> 0.6` 关夹爪，`< 0.4` 开夹爪 |

### 常用参数

| 参数 | 默认 | 含义 |
|---|---|---|
| `--arm` / `active_arm` | `both` | `left` / `right` / `both` |
| `--real` | 关 | 用真机而不是 mock 硬件 |
| `base_sep` | `1.05` | 两台机器人底座实际间距（米），**必须和真实摆位对上** |
| `control_tip` | `hand` | Servo 控制点：`hand` 或 `link8` |
| `--no-rviz`、`--no-gripper` | 关 | 不起 RViz / 夹爪节点 |
| `SCALE`（脚本内常量） | `2.0` | 手柄位移 → 机械臂位移 |

脚本只是薄封装，等价的原始命令是：

```bash
ros2 launch franka_vr dual_franka_teleop.launch.py \
    active_arm:=right use_fake_hardware:=false robot_ip_right:=172.16.0.3
```

## 仓库结构

| 路径 | 内容 |
|---|---|
| `ws_franka_vr/src/franka_vr/` | **核心自研包** —— 外环控制节点、双臂 launch、全部配置、Quest 桥接 + APK |
| `ros2_ws/src/` | franka_ros2 **v2.3.0** + franka_description **1.6.1** |
| `ws_moveit2/src/` | MoveIt 2 **2.13.0** 源码，含 `moveit_servo` |
| `ws_ik_plugins/src/` | pick_ik **1.1.2** —— 必须源码编译，原因见下 |
| `docker_launch_files/` | Dockerfile、compose、entrypoint |
| `scripts/` | 一键编译 / 起机械臂栈 / 起桥接 / 头显连线自检 |
| `docs/` | 结构说明、排障速查、控制调优复盘 |
| `tools/`、`datasets/` | 数据集回放工具与样例轨迹（周边工具，不是通路本身） |
| `setup_env.sh` | 按依赖顺序 source 四个工作区 |
| `udp_only.xml` | 强制 FastDDS 只走 UDP —— 共享内存跨不过容器 IPC 命名空间 |

本仓库是**源码快照**：`build/`、`install/`、`log/` 和诊断录制产物不入库，`colcon build` 会重新生成。

**故意没收进来**的：两份过时的 libfranka 源码（0.9.2 与 0.13.3，对 FR3 服务器 v10 都是错版本）、
一份旧的 franka_ros2 v0.1.15、以及一个 ROS 1 工作区。收进来只会诱导别人编错版本。

## 版本对照表

| 组件 | 版本 | 为什么是它 |
|---|---|---|
| libfranka | **0.20.4**（apt `ros-humble-libfranka`） | FR3 服务器 v10 要求 ≥ 0.20；0.13.x 会报 `Incompatible library version (server 10, library 7)` |
| franka_ros2 | **v2.3.0** | 与 libfranka 0.20.4 配套 |
| franka_description | **1.6.1** | franka_ros2 v2.3.0 指定 |
| MoveIt 2 | **2.13.0**，源码编译 | Humble 的 apt 版是 2.5.9，旧 `servo.h` 架构里没有本方案用的 `servo.hpp` / `getNextJointState` / `TwistCommand` |
| pick_ik | **1.1.2**，源码编译 | apt 版会拉 apt 版 `moveit_core`，与源码版并存导致加载时 ABI 冲突 |

## 两个最容易误会的点

**pick_ik 装着，但实时遥操不调用它。** 控制节点走 MoveIt Servo 的 `TWIST` 模式，
twist → 关节由**雅可比伪逆**完成，也没有启动 `move_group`。
所以 `kinematics_{left,right}.yaml` 里的权重参数**对遥操全部不生效**——别在那儿调参治遥操问题。
但那两个文件必须保留：launch 会无条件读取它们。
限位实际由 `fr3_servo_*.yaml` 里的 `joint_limit_margins` 兜底。

**遥操没有标定文件。** 按 `Enter` 时用实时 TF 重锚，令目标等于当前末端位姿，
接入误差结构上恒为 0，改 `SCALE` 也不需要重新标定。
早期版本把标定存成 JSON，出过"文件 0 字节 → 目标差几米 → 一启动满速猛冲触发 reflex 报红"。

## 文档

| 文档 | 什么时候看 |
|---|---|
| [MIGRATION.md](MIGRATION.md) | 换新机器重建：版本对齐、编译顺序、运行命令 |
| [docs/architecture.md](docs/architecture.md) | 要查 topic / frame 名字，或逐段验证是哪一段断了 |
| [docs/troubleshooting.md](docs/troubleshooting.md) | 编不过、连不上、行为不对 |
| [docs/control_tuning_retrospective.md](docs/control_tuning_retrospective.md) | **改任何控制参数之前必读** —— 几十轮真机迭代，包括那些验证过无效甚至有害的改法 |
| [tools/README.md](tools/README.md) | 用数据集回放工具 |

## 排障

最常见的三个：

| 症状 | 根因 |
|---|---|
| 话题列得出来，`echo` 一条都收不到 | FastDDS 共享内存跨不过容器 IPC 命名空间 —— 用 `udp_only.xml` |
| 机械臂呈 **~5 秒周期**抽搐 | 别的进程占着 adb server，它的轮询打断了 logcat 长连接 |
| 一连真机就报 `Incompatible library version` | libfranka 版本不是 0.20.4 |

完整清单见 [docs/troubleshooting.md](docs/troubleshooting.md)。

## 致谢

| 组件 | 来源 |
|---|---|
| Docker 骨架 | fork 自 [`Fjakob/libfranka-docker`](https://github.com/Fjakob/libfranka-docker)（经 `ZorAttC/libfranka-docker`），已按真机验证过的环境重写 |
| franka_vr 包、Quest APK、oculus_reader | 原开源项目 `franka_vr`；本仓库做了双臂化并重写了控制环 |
| libfranka / franka_ros2 / franka_description | [Franka Robotics](https://github.com/frankaemika) |
| MoveIt 2 / moveit_servo / pick_ik | [MoveIt](https://github.com/moveit) |

## 许可

本仓库未声明统一许可证。随仓库分发的第三方组件各自保留原许可证——见
`ros2_ws/src/`、`ws_moveit2/src/`、`ws_ik_plugins/src/`、
`ws_franka_vr/src/franka_vr/oculus_reader/` 下的 `LICENSE` 文件。再分发前请遵守它们。
