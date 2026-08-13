# 通路结构：手柄怎么变成机械臂的动作

一条数据流从 Quest 手柄走到 FR3 关节力矩，中间经过 5 段。搞不清哪段出问题时，
按这张图从上往下逐段验证（每段都给了验证命令）。

```
 ┌─ Quest 3S 头显 ────────────────────────────────────────────────┐
 │  teleop-debug.apk (com.rail.oculus.teleop)                     │
 │  以 tag=wE9ryARX 往 logcat 打手柄位姿 4x4 矩阵 + 按键          │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ① USB / adb logcat -T 0 -s wE9ryARX:I   (~70 Hz)
                ▼
 ┌─ franka_dev 容器：start_franka_vr_dual.py ─────────────────────┐
 │  reader.py 解析 →  ×SCALE(2.0) → 转到 ROS 坐标系               │
 │  广播 TF:  world → oculus_base(静态) → oculus_{left,right}     │
 │  接入即重锚: target ≡ 当前末端位姿, 接入误差恒 0                │
 │  Grip 键 → 夹爪 action                                         │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ② 服务 /{left,right}/set_target_pose (franka_vr/srv/SetTargetPose, 70 Hz)
                │    旁路诊断话题 /{left,right}/debug_target (PoseStamped)
                ▼
 ┌─ franka_dev 容器：demo_franka_vr_vel (每臂一个, 命名空间 /left /right) ─┐
 │  外环 200 Hz：位姿误差 → 笛卡尔速度 twist                       │
 │    速度前馈(0.8) + PD(kp_lin 7.0 / kp_ang 4.0) + Smith 预测(0.09s)│
 │    + 最短弧四元数 + 2·vec 对数映射 + 限速限加速度 + 输出低通     │
 │  反馈闭在【真机实测状态】上, 不是内部模型                        │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ③ MoveIt Servo (CommandType::TWIST) → 雅可比伪逆 → 关节位置/速度
                ▼
 ┌─ ros2_control (每臂一个 ros2_control_node) ───────────────────┐
 │  /{ns}/{prefix}_fr3_arm_controller  JointTrajectoryController  │
 │  effort 接口 + 每关节 PD (j1-4 硬 p=600, j7 软 p=50)           │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ④ franka_hardware → libfranka 0.20.4 → 1 kHz 实时环
                ▼
            FR3 真机 (左 172.16.0.2 / 右 172.16.0.3)
                │ ⑤ 关节状态回读 /{ns}/franka/joint_states (30 Hz)
                └────────────► 回到外环当反馈 + robot_state_publisher 发 TF
```

---

## 命名约定（双臂靠前缀 + 命名空间隔开）

| 项 | 左臂 | 右臂 |
|---|---|---|
| 命名空间 | `/left` | `/right` |
| URDF 前缀 | `left_` | `right_` |
| 基座帧 | `left_fr3_link0` | `right_fr3_link0` |
| 末端帧 | `left_fr3_hand`（`control_tip:=link8` 时为 `left_fr3_link8`） | `right_fr3_hand` |
| 规划组 | `left_fr3_arm` | `right_fr3_arm` |
| 控制器 | `left_fr3_arm_controller` | `right_fr3_arm_controller` |
| 手柄帧 | `oculus_left` | `oculus_right` |
| 机器人 IP | 172.16.0.2 | 172.16.0.3 |

两条臂都挂在公共 `world` 下：`world → {left,right}_base`，间距由 launch 参数
`base_sep`（默认 1.05 m）给定，要和真机实际摆位对上，否则双臂坐标系是错的。

---

## 逐段验证

```bash
source <仓库根>/setup_env.sh

# ① Quest → PC：adb 能看到设备、APK 在跑
bash scripts/setup_quest_adb.sh

# ② 桥接 → 目标位姿：手柄一动，这里就该有数
ros2 topic hz  /right/debug_target
ros2 topic echo /right/debug_target --once
ros2 run tf2_ros tf2_echo right_fr3_link0 oculus_right     # 手柄在基座系下的位姿

# ③ 外环 → Servo：有关节轨迹发出来
ros2 topic hz /right/right_fr3_arm_controller/joint_trajectory   # 应≈200Hz

# ④ 控制器活着
ros2 control list_controllers -c /right/controller_manager

# ⑤ 真机状态回来了
ros2 topic hz /right/franka/joint_states                          # 应≈30Hz
ros2 run tf2_ros tf2_echo right_fr3_link0 right_fr3_hand
```

哪一步没数，问题就在它上面那一段。

---

## 三个容易误会的点

**1. pick_ik 装了，但实时遥操不调用它。**
`demo_franka_vr_vel` 走 Servo 的 `CommandType::TWIST`，twist→关节由**雅可比伪逆**
完成；`dual_franka_teleop.launch.py` 也没起 `move_group`。所以
`kinematics_{left,right}.yaml` 里的权重参数**对遥操全部不生效**，别在那儿调参治遥操。
（那份 yaml 仍被 launch 无条件读取，删了会崩，所以文件里加了防误解横幅。）
真正管限位的是 `fr3_servo_*.yaml` 里的 `joint_limit_margins`。

**2. 遥操里没有"标定文件"。**
早期版本存过标定 json，出过"文件 0 字节 → offset 全 0 → 目标差几米 → 启动猛冲报红"。
现在改成**接入即重锚**：按 Enter 那一刻用实时 TF 令 `target ≡ 当前末端位姿`，
接入误差恒为 0，结构上不存在猛冲的动力来源。改 SCALE 之类参数也不用重新标定。

**3. 位置是绝对映射，姿态也是绝对映射（都带重锚偏置）。**
`target = 手柄位置×SCALE + cal_offset`、`q_target = q_手柄 ⊗ ori_offset`。
绝对映射对**头显追踪稳定性**极敏感——头显要戴在头上、手柄始终在头显相机视野内，
否则位姿会漂。单帧位移 > `JUMP_MAX`(0.08 m) 的帧会被丢弃。
