# 排障速查

按"症状 → 根因 → 怎么修"排的。每条都是这套系统上**真的踩过**的，不是通用建议。
控制层面的抖动/跟手问题在 [control_tuning_retrospective.md](control_tuning_retrospective.md) 里有完整推理过程。

---

## 环境 / 编译

### `Incompatible library version (server 10, library 7)`
FR3 机器人服务器是 version 10，要求 libfranka ≥ 0.20。装成 0.13.x 就报这个。
→ 本仓库 Dockerfile 装的是 apt 的 `ros-humble-libfranka=0.20.4-*`。
如果是老容器，`dpkg -l | grep libfranka` 确认版本，别再用旧配方里"源码编 libfranka_v0.13.2"那条路。

### `franka_semantic_components` 找不到 `controller_interface/helpers.hpp`
→ 在该包的 `CMakeLists.txt` 的 `THIS_PACKAGE_INCLUDE_DEPENDS` 和 `package.xml` 里补上
`controller_interface`。（franka_ros2 v2.3.0 一般已包含，先编，报错再补。）

### `geometric_shapes` 找不到 `random_numbers::random_numbers`
→ `ws_moveit2/src/random_numbers` 必须在（本仓库已带，tag 2.0.5 / ros2 分支）。
清掉 `ws_moveit2/build/geometric_shapes` 再重编。

### MoveIt2 编到一半进程被杀 / 机器卡死
基本都是 OOM。→ `bash scripts/build_all.sh -j 2`（内存 <16G 就用 2）。
编挂了不用从头来：`--from 2` 接着编，colcon 会跳过已完成的包。

### `moveit_servo` 报 `servo.h` 之类的旧 API 找不到
apt 装的 MoveIt 2.5.9 是旧架构，本方案要的是 2.13.0 的 `servo.hpp` / `getNextJointState` /
`TwistCommand`。→ 必须源码编 `ws_moveit2`，且 `setup_env.sh` 里它要在 apt 版之后 source。

### 改了 `kinematics_*.yaml` / `fr3_servo_*.yaml` 不生效
`ws_ik_plugins` / `ros2_ws` 的 install **不是** symlink 安装，config 改了要重编（或手动 cp 进 install）。
`ws_franka_vr` 用了 `--symlink-install`，改 yaml/launch 立即生效，改 .cpp 仍要重编。

---

## 通信 / DDS

### 宿主机 `ros2 topic list` 看得见话题，`echo` 一条都收不到；TF 树里连 `left_fr3_link0` 都没有
`franka_dev` 是 `IpcMode=private`，FastDDS 的共享内存段跨不过 IPC 命名空间。
同厂商 + 同主机名时 FastDDS 优先选 SHM → "发现能通、数据收不到"。
→ 用仓库根的 `udp_only.xml` 强制只走 UDPv4：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=<仓库根>/udp_only.xml    # setup_env.sh 已自动 export
```

容器内**每个**起节点的终端都要有这个环境变量。

### 宿主机和容器互相看不见节点
两边 `ROS_DOMAIN_ID` 要一致（本仓库 `setup_env.sh` 默认 0），且容器必须是 `network_mode: host`。

---

## Quest / adb

### 机械臂呈 **~5 秒周期**抽搐
adb server 是全机单例（127.0.0.1:5037），谁先起谁拥有。
`wuji-hand-teleop` 容器里的 `adb_watchdog.sh` 每 5 秒跑一次 `adb devices` + `adb reverse --list`，
它不认设备型号，插着 Quest 也照打——每 5 秒打断一次本项目的 `logcat` 长连接
→ 手柄位姿成批到达 + 跳变 → 下游 `use_smoothing:false` + `max_expected_latency:0.03`
直接把跳变变成速度尖峰 → 抽搐。**周期恰好 ~5 秒就是这个根因的特征。**

```bash
docker exec wuji-hand-teleop pkill -9 -f adb_watchdog.sh
docker exec wuji-hand-teleop pkill -9 -f RoboticsServiceProcess
adb reverse --remove-all; adb kill-server
```
（那个 watchdog `trap '' INT TERM`，Ctrl+C 杀不掉，必须 `kill -9`。
换 adb server 属主后头显会重新弹 USB 调试授权——密钥变了——记得点"一律允许"。）

### `adb devices` 显示 `unauthorized`
头显里没点允许，或换过 adb server 属主。→ **戴上头显**，重新点"一律允许"。

### `adb devices` 显示 `no permissions`
宿主机缺 udev 规则。→ 在容器里跑（compose 已 `privileged` + 挂 `/dev`），或宿主机加 udev 规则。

### 一个 CPU 核被 python 跑满
`reader.py` 的 logcat 必须带 tag 过滤 `-s wE9ryARX:I`。不带过滤时整台 Quest 的日志
（每秒上万行）全灌过来，逐行 grep 就是忙等。本仓库版本已带过滤（~70 行/秒）。

### 手柄位姿漂 / 跳
绝对映射对头显追踪极敏感：头显要**戴在头上**（摘下来放桌上追踪就废了），手柄始终在头显相机视野内。
单帧位移 > 0.08 m 的帧会被丢弃并打 `跳变忽略` 日志——日志刷屏就说明追踪在丢。

---

## 遥操行为

### 按 Enter 接入瞬间机械臂猛冲 / 报红
现版本结构上不该出现——接入即重锚令接入误差恒为 0。若仍发生，看日志有没有
`重锚失败(查不到末端TF)`：查不到 TF 时脚本会**拒绝发目标**（安全设计），
说明 ④⑤ 段没起来（控制器没 spawn / joint_states 没发 / TF 断了），先修那里。

### 手不动，机械臂却朝一个方向持续转，反向也纠不回来
四元数双覆盖：姿态误差走了"长弧"（>180°）。现版本已在 `q_diff.w()<0` 时翻到最短弧。
若在改过的代码里复现，检查这段有没有被改掉。

### 来回平移越走越偏（漂移）
外环反馈闭在了内部模型（`robot_state` 只被自己的指令喂养、不回读真机）上，偏差逐周期累积。
现版本用 `getStateMonitor()->getCurrentState()` 闭真机。

### 转腕时 ~2 Hz 剧烈共振
环内延迟致振（抖动频率 ≈ π/T）。治法是**消延迟**，不是加低通、也不是降 kp：
- `fr3_servo_*.yaml` 的 `max_expected_latency` 已从 0.1 降到 0.03（它不是"测量值"，
  是"你让每条指令晚多少执行"，servo 给指令打 `now + 此值` 的未来时间戳）。下限约 0.02–0.025。
- 外环 Smith 预测器（`T_PREDICT=0.09`）把残余延迟从稳定性方程里消掉。

⚠️ 这类抖动**加 kd 会更糟**（D 经延迟给谐振喂能量），抬内环腕关节 d 会引入 9.8~342 Hz 高频颤。

### 残留 ~9.8 Hz 细抖
指令端干净、只在实测里 → 产生于低通下游（内环 PD / 结构共振 / franka 本体），外环软件够不着。
→ 物理层解决：**加固底座**，零控制代价。

### 老顶关节限位
TWIST 模式走雅可比伪逆，**没有主动避限位**（pick_ik 那套权重不在这条路径上）。
现有手段是 `fr3_servo_*.yaml` 的 `joint_limit_margins` 硬缓冲；
`halt_all_joints_in_cartesian_mode: false` 让单关节顶限位时只停该关节、其余继续用冗余跟随。

---

## 真机安全

- 起真机前确认：两台 FR3 已解抱闸、外部急停在手边、工作空间无人。
- `scripts/run_arm_stack.sh --real` 会二次确认，别用 `yes |` 之类跳过。
- 回放工具与遥操**不能同时跑**——`tools/replay_wuji_vr_dataset.py` 会主动检测
  `start_franka_vr_dual.py` 是否在跑，在跑就拒绝执行，避免两路指令抢同一台机器人。
