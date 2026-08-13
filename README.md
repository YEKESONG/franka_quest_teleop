# franka_quest_teleop

Meta Quest 3S 手柄遥操 Franka **FR3 双臂**的**完整可运行环境**：
ROS 2 Humble + MoveIt Servo + Docker，clone 下来照 [MIGRATION.md](MIGRATION.md) 走一遍就能跑。

已在真机双臂 FR3 上跑通：**绝对位姿跟随、接入零误差、全程限速、Smith 预测消延迟致振**。

```
Quest 3S 手柄 ──adb logcat──> 桥接节点 ──set_target_pose──> 外环速度控制
   ──MoveIt Servo(TWIST)──> ros2_control ──libfranka──> FR3 双臂
```

---

## 快速开始

```bash
git clone https://github.com/YEKESONG/franka_quest_teleop.git
cd franka_quest_teleop

# 1. 起容器（仓库根会被挂到容器里的 /docker_volume）
xhost +local:docker                                             # RViz 要显示
docker compose -f docker_launch_files/docker-compose.yml build
docker compose -f docker_launch_files/docker-compose.yml up -d  # 后台常驻
docker exec -it franka_dev bash                                 # 要几个终端就 exec 几次

# 2. 容器内编译（第一次很久：MoveIt2 那步双核约数小时）
bash /docker_volume/scripts/build_all.sh -j 2     # 内存 <16G 就用 2，别用满核

# 3. 跑（两个终端，各自 docker exec 进来）
bash /docker_volume/scripts/run_arm_stack.sh --real     # 终端1 机械臂栈
bash /docker_volume/scripts/run_vr_bridge.sh            # 终端2 Quest 桥接
```

先在仿真里验证再上真机：把 `--real` 去掉即可（默认 `use_fake_hardware:=true`）。

**操作**：`Enter` 开/关遥操（接入瞬间自动对零，接入误差为 0）；`1`/`2` 遥操中手动重新对零左/右臂；
手柄 `Grip` 键控夹爪。

---

## 仓库结构

| 路径 | 内容 |
|---|---|
| `ws_franka_vr/src/franka_vr/` | ★ **核心自研包**：外环速度控制节点、双臂 launch、全部配置、Quest 桥接脚本 + APK |
| `ros2_ws/src/` | franka_ros2 **v2.3.0** + franka_description **1.6.1**（源码，含 mesh） |
| `ws_moveit2/src/` | MoveIt2 **2.13.0** 源码（6 个仓库，含 moveit_servo） |
| `ws_ik_plugins/src/` | pick_ik **1.1.2**（必须源码编，见下） |
| `docker_launch_files/` | Dockerfile + docker-compose + entrypoint |
| `scripts/` | 一键编译 / 起机械臂栈 / 起桥接 / Quest 连线自检 |
| `docs/` | 结构说明、排障速查、控制调优复盘 |
| `tools/` + `datasets/` | 数据集回放工具与样例数据（周边，非通路本身） |
| `setup_env.sh` | 容器内一键 source 四个工作区 |
| `udp_only.xml` | FastDDS 只走 UDP（容器 IPC 命名空间下 SHM 不通） |

四个工作区在容器内都在 `/docker_volume/` 下 —— **仓库根 = `/docker_volume`**，
所以文档、脚本、launch 里写的容器内路径和仓库里的相对路径是一一对应的。

> 本仓库是**源码快照**，不含 `build/ install/ log/`（换机器 `colcon build` 重新生成），
> 也不含 `diag_logs/`（诊断录制产物，几十 MB，每次跑都会重新生成）。

**故意没收进来的东西**（原开发机 `~/libfranka-docker/docker_volume/` 里还有，但这条通路用不到）：

| 没收 | 为什么 |
|---|---|
| `libfranka/`(0.9.2)、`libfranka_v0.13.2/`(0.13.3) 源码 | 都是**错版本**。FR3 服务器 v10 要 ≥0.20，现在统一用 apt 的 0.20.4。收进来只会诱导别人编错版本 |
| `franka_ros2_OLD_v0.1.15/` | 早期版本，已被 v2.3.0 取代 |
| `catkin_ws/`（ROS1 franka_ros） | ROS1 分支，本方案没用到 |

---

## 文档

| 文档 | 什么时候看 |
|---|---|
| [MIGRATION.md](MIGRATION.md) | **换新设备重建**：版本对齐、编译顺序、运行命令 |
| [docs/architecture.md](docs/architecture.md) | 想知道数据怎么从手柄流到关节、topic/frame 叫什么、怎么逐段验证 |
| [docs/troubleshooting.md](docs/troubleshooting.md) | 出问题了：编不过、收不到数据、抽搐、猛冲、限位…… |
| [docs/control_tuning_retrospective.md](docs/control_tuning_retrospective.md) | 想改控制参数前**必读**：几十轮真机迭代的推理过程与失败记录 |
| [tools/README.md](tools/README.md) | 用数据集回放工具 |

---

## 关键版本（对不上会不兼容）

| 组件 | 版本 | 为什么是这个 |
|---|---|---|
| libfranka | **0.20.4**（apt `ros-humble-libfranka`） | FR3 服务器 version 10 要求 ≥0.20；0.13.x 直接报 `Incompatible library version` |
| franka_ros2 | **v2.3.0** | 与 libfranka 0.20.4 配套 |
| franka_description | **1.6.1** | franka_ros2 v2.3.0 的 dependency.repos 指定 |
| MoveIt2 | **2.13.0**（源码编译） | Humble apt 版是 2.5.9 的旧 `servo.h` 架构，跑不了本方案用的 `servo.hpp`/`getNextJointState`/`TwistCommand` |
| pick_ik | **1.1.2**（源码编译） | apt 版会拉 apt 版 moveit_core，与源码版并存导致 ABI 冲突 |
| ROS 2 | Humble (Ubuntu 22.04) | — |

---

## 两个必须知道的事实

**pick_ik 装着，但实时遥操不调用它。** 当前控制节点走 MoveIt Servo 的 TWIST 模式，
twist→关节由雅可比伪逆完成，`kinematics_{left,right}.yaml` 里的权重**对遥操不生效**
（那两个 yaml 仍被 launch 无条件读取，删了会崩）。别在那儿调参治遥操问题。

**遥操没有标定文件。** 按 Enter 接入的瞬间用实时 TF 重锚，令目标 ≡ 当前末端位姿，
接入误差恒为 0；改 SCALE 之类参数不需要重新标定。

---

## 组件来源

| 组件 | 来源 |
|---|---|
| Docker 环境骨架 | fork 自 `Fjakob/libfranka-docker`（经 `ZorAttC/libfranka-docker`），已按真实环境重写 |
| franka_vr 包 + Quest APK + oculus_reader | 原开源项目 `franka_vr`，本仓库做了双臂化、安全化与控制重构 |
| libfranka / franka_ros2 / franka_description | Franka Robotics 官方 |
| MoveIt2 / moveit_servo / pick_ik | MoveIt 官方 |
