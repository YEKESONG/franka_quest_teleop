# Quest 3S 遥操 Franka FR3 —— 环境重建手册

从零把这套遥操跑起来。每一步都标了"为什么"，踩过的坑写在原地，不用再踩一遍。

> 本仓库是**源码快照**：不含 `build/ install/ log/`、不含各依赖的嵌套 `.git`。
> 换机器 `colcon build` 重新生成即可。

---

## 0. 硬件与前提

| 项 | 值 |
|---|---|
| 机器人 | Franka **FR3 双臂**（左 `172.16.0.2`，右 `172.16.0.3`） |
| 机器人服务器版本 | **version 10** ← 这一条决定了 libfranka 必须 ≥ 0.20 |
| VR 设备 | Meta **Quest 3S**（双手柄） |
| 宿主系统 | Ubuntu 22.04 + Docker + Docker Compose v2 |
| 容器基础 | ROS 2 **Humble** |
| 建议内存 | ≥16 G（编 MoveIt2 用；小内存要降并行度） |
| 网络 | 宿主机能 ping 通 172.16.0.2/3；容器用 host 网络 |

单臂也能跑：所有 launch / 脚本都支持 `active_arm:=left|right|both`。

---

## 1. 版本对齐（**先看这张表**，版本错了后面全白搭）

| 组件 | 版本 | 装法 | 错了会怎样 |
|---|---|---|---|
| libfranka | **0.20.4** | apt `ros-humble-libfranka=0.20.4-*`（Dockerfile 已带） | 装 0.13.x → 真机报 `Incompatible library version (server 10, library 7)` |
| franka_ros2 | **v2.3.0** (`7ed0458`) | 本仓库 `ros2_ws/src/` 已带 | 与 libfranka 版本不配套，编译或运行时报接口错 |
| franka_description | **1.6.1** (`2c4610f`) | 同上 | xacro 不支持 `arm_prefix` / `connected_to`，双臂前缀化起不来 |
| MoveIt2 | **2.13.0** (`bb2eb75`) | 本仓库 `ws_moveit2/src/` 源码编译 | apt 的 2.5.9 是旧 `servo.h` 架构，本方案的 `servo.hpp`/`getNextJointState`/`TwistCommand` 全不存在 |
| moveit_msgs / geometric_shapes / srdfdom / random_numbers / moveit_resources | MoveIt2 2.13.0 配套的 ros2 分支 | 同上 | `geometric_shapes` 缺 `random_numbers::random_numbers` target |
| pick_ik | **1.1.2** (`7e1794a`) | 本仓库 `ws_ik_plugins/src/` 源码编译 | **不能 apt 装**：apt 版会拉 apt 版 `moveit_core`，和源码版并存 → ABI 冲突 → 加载即崩 |

> 旧配方里"从源码编 `libfranka_v0.13.2` 再 `dpkg -i`"那条路**已废弃**——它装出来的是 0.13.3，
> 对 FR3 服务器 v10 是错版本。现在统一用 apt 的 0.20.4。

---

## 2. 起容器

```bash
cd <仓库根>
xhost +local:docker          # 每次宿主机重启后都要执行一次，否则 RViz 起不来

docker compose -f docker_launch_files/docker-compose.yml build      # 首次约 10-20 分钟
docker compose -f docker_launch_files/docker-compose.yml run --rm franka_dev
```

再开终端接进同一个容器：

```bash
docker exec -it franka_dev bash
```

> **这台开发机上要注意**：已经有一个在跑的老容器也叫 `franka_dev`（镜像 tag
> `libfranka:franka_ros2`）。本仓库的 compose 用的是**另一个** tag
> `franka_quest_teleop:humble`，build 不会抢走老 tag、不影响正在跑的通路。
> 但容器名默认相同，要两个并存就改名：
> `FRANKA_CONTAINER_NAME=franka_dev2 docker compose -f docker_launch_files/docker-compose.yml up -d`

compose 做了这几件事，缺一不可：

| 配置 | 为什么 |
|---|---|
| `..:/docker_volume` | **仓库根 = 容器里的 `/docker_volume`**，编译产物落在宿主机，容器可随时重建 |
| `network_mode: host` | 机械臂在 172.16.0.x，且要和宿主机上的 ROS 节点互通 |
| `privileged` + `/dev:/dev` | adb 直接看到 USB 上的 Quest；实时设备访问 |
| `cap_add: SYS_NICE` + `rtprio: 99` | FR3 的 1 kHz 实时控制回路要提实时优先级 |
| `/tmp/.X11-unix` + `DISPLAY` | RViz |

> 容器入口**不会**自动编译（旧的 `install_franka_ros2.sh` 会，编几小时且失败就进不去容器）。
> 编译是显式的一步，见下。

---

## 3. 编译（顺序不能乱，后者依赖前者）

```bash
bash /docker_volume/scripts/build_all.sh          # 全编
bash /docker_volume/scripts/build_all.sh -j 2     # 内存 <16G / 双核机器
bash /docker_volume/scripts/build_all.sh --from 2 # 编挂了从第 2 步接着编
bash /docker_volume/scripts/build_all.sh --only 4 # 只改了 franka_vr 时
```

脚本干的就是下面这五步，手动等价于：

```bash
source /opt/ros/humble/setup.bash

# (1) franka_ros2 v2.3.0 + franka_description 1.6.1
cd /docker_volume/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# (2) MoveIt2 2.13.0 —— 最重的一步，双核约数小时，可挂夜里跑
cd /docker_volume/ws_moveit2 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# (3) pick_ik（必须针对上一步的源码版 moveit_core 编）
cd /docker_volume/ws_ik_plugins && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# (4) franka_vr 本体
cd /docker_volume/ws_franka_vr && colcon build --packages-select franka_vr --symlink-install
source install/setup.bash

# (5) 桥接脚本的 python 包（reader.py 用绝对包名 import，不装会 ImportError）
pip3 install -e /docker_volume/ws_franka_vr/src/franka_vr/oculus_reader
```

**第 2 步失败绝大多数是 OOM**（进程被杀 / 机器卡死）→ 降到 `-j 2`，再 `--from 2` 续编。

编完每个新终端只需：

```bash
source /docker_volume/setup_env.sh
```

它按 franka_ros2 → MoveIt2 → pick_ik → franka_vr 的顺序 source（overlay 顺序不能反），
并自动 `export FASTRTPS_DEFAULT_PROFILES_FILE=<仓库根>/udp_only.xml`。

---

## 4. 跑起来

### 4.1 Quest 准备（一次性）

```bash
bash /docker_volume/scripts/setup_quest_adb.sh
```

它会：验 adb → 列设备 → 检查有没有别的进程抢 adb → 装/拉起 APK。
首次连线要**戴上头显**点"允许 USB 调试 → 一律允许"。

APK 在 `ws_franka_vr/src/franka_vr/oculus_reader/oculus_reader/APK/teleop-debug.apk`
（包名 `com.rail.oculus.teleop`），桥接节点起来时也会自己检查并安装。

### 4.2 仿真先跑一遍（不碰真机）

```bash
source /docker_volume/setup_env.sh
bash /docker_volume/scripts/run_arm_stack.sh                    # 双臂仿真 + RViz
bash /docker_volume/scripts/run_vr_bridge.sh                    # 另一个终端
```

RViz 里能看到双臂、按 Enter 后模型跟手动 —— 这一步过了再上真机。

### 4.3 真机

```bash
# 终端1：控制栈 + MoveIt Servo + 夹爪 + RViz
bash /docker_volume/scripts/run_arm_stack.sh --real
#   等价于 ros2 launch franka_vr dual_franka_teleop.launch.py \
#            active_arm:=both use_fake_hardware:=false \
#            robot_ip_left:=172.16.0.2 robot_ip_right:=172.16.0.3

# 终端2：Quest 桥接（必须前台跑在交互终端，脚本要读单键）
bash /docker_volume/scripts/run_vr_bridge.sh

# 终端3（可选）：抖动诊断录制
cd /docker_volume/ws_franka_vr/src/franka_vr/oculus_reader
python3 oculus_reader/diag_record.py --side left
```

上真机前确认：**两台 FR3 已解抱闸、外部急停在手边、工作空间无人**。

### 4.4 操作

| 键 / 手柄 | 作用 |
|---|---|
| `Enter` | 遥操 开 / 关。**接入瞬间自动重锚**：目标 ≡ 当前末端位姿，接入误差恒 0 |
| `1` / `2` | 遥操进行中，手动重新对零 左 / 右臂 |
| 手柄 `Grip` | > 0.6 关夹爪，< 0.4 开夹爪 |

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `active_arm` / `--arm` | `both` | `left` / `right` / `both` |
| `base_sep` | `1.05` | 两台机器人底座实际间距(米)，**要和真机摆位对上** |
| `control_tip` | `hand` | Servo 控制点：`hand` 或 `link8` |
| `load_gripper` | `true` | 关掉则不起夹爪节点、URDF 也不带 hand |
| `SCALE`（脚本内常量） | `2.0` | 手柄位移 → 机械臂位移 |

---

## 5. 遥操控制逻辑（相对原开源版改了什么）

- **位置/姿态都是绝对映射 + 重锚偏置**：`target = 手柄位置×SCALE + cal_offset`、
  `q_target = q_手柄 ⊗ ori_offset`；两个 offset 在按 Enter 接入的瞬间由实时 TF 算出，
  使接入误差恒为 0（bumpless）。**不再有标定文件**——历史上标定文件为 0 字节导致过
  "目标差几米、一启动猛冲报红"。
- **外环闭在真机实测状态上**（不是内部积分模型），根治来回平移的累积漂移。
- **速度前馈 + Smith 预测器**：前馈补主体运动（开环，不吃稳定裕度），
  Smith 预测器把环内延迟从稳定性方程里消掉，根治转腕 2 Hz 共振。
- **姿态误差用最短弧 + `2·vec` 对数映射**：修掉四元数双覆盖导致的"持续朝一个方向转"，
  以及 `axis×angle` 在小角度处轴方向乱跳放大成多关节抖。
- **全程限速限加速度 + 软启动 + 抗积分饱和 + 跳变丢弃**。

完整参数表和"为什么是这些值"见
[docs/control_tuning_retrospective.md](docs/control_tuning_retrospective.md)。
**改控制参数前先读它**——里面记了大量已经验证无效甚至有害的改法
（比如这套系统里"抖动加 kd"是反的）。

---

## 6. 遇到问题

先按 [docs/architecture.md](docs/architecture.md) 的"逐段验证"定位是哪一段断了，
再查 [docs/troubleshooting.md](docs/troubleshooting.md)。最常见的三个：

| 症状 | 去看 |
|---|---|
| 宿主机看得见话题但收不到数据 | FastDDS SHM 跨不过容器 IPC 命名空间 → `udp_only.xml` |
| 机械臂 ~5 秒周期抽搐 | 别的进程抢 adb，打断了 logcat 长连接 |
| 真机一连就报 `Incompatible library version` | libfranka 版本不是 0.20.4 |
