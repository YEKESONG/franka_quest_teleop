> **收录说明**（2026-08-13）：本文原件在 `~/sy/VR遥操Franka_IK与控制调优复盘.md`，
> 逐字收进本仓库，正文未改。文中的容器路径 `/docker_volume/...` 就是**本仓库根目录**，
> 例如 `/docker_volume/ws_franka_vr/...` = 仓库里的 `ws_franka_vr/...`。
> 文末"第六部分"第 5 条提到的"活跃代码未纳入 git"——就是本次整合要解决的问题，现已解决。

# VR 遥操 Franka：IK 与控制环调优复盘

> 记录 Quest 3s 遥操双臂 Franka FR3 项目中，IK 算法与外环控制的全部尝试、踩过的坑，以及为什么最终落在当前这一版。
> 生成日期：2026-07-23

---

## 0. 一句话总览

问题从"IK 把关节顶到限位报红"开始，换 IK 插件（KDL → pick_ik）解决了限位锁死；但随后暴露出一连串**外环控制**问题（漂移、持续旋转、延迟致振、细抖），经过几十轮"改一处、真机测一次、看数据"的迭代，最终靠 **pick_ik + 速度前馈 + Smith 预测器 + 闭真机反馈** 这一组合达到当前最好状态。

**核心教训**：不要靠"手感"猜，要靠**录数据 + 频谱/雅可比分析**定位病因；很多"想当然"的经验规则（抖动加 kd、误差大加 kp）在带 100ms 延迟的这套系统里恰好反过来。

---

## 第一部分：IK 算法的演变

### 1.1 起点 —— KDL（MoveIt 默认）
- 原系统用 MoveIt Servo 的 TWIST（速度级）模式，IK 委托给 `kdl_kinematics_plugin/KDLKinematicsPlugin`。
- KDL 是 **牛顿-拉夫逊迭代求根**（`ChainIkSolverPos_NR_JL`），内部冗余解算是最小范数速度（贪心），`return_approximate_solution=true` 会返回"最接近的解"——常常贴着限位。
- **症状**：遥操到某些位姿时，某关节被顶到限位 → 报红锁死。7DOF 冗余完全没被用来避限位。

### 1.2 两次判断更正（诚实记录我的错误）
1. **误判为雅可比伪逆**：我最初以为 servo 用的是内部雅可比伪逆。读了 `moveit_servo/src/utils/command.cpp` 的 `jointDeltaFromIK` 才发现：**只要配了 kinematics 插件，就走插件（KDL），不是内部伪逆**。这个更正很关键——它意味着"换插件就能换 IK，零代码"。
2. **误判为"闭式一步解"**：我一度说它是闭式伪逆、不迭代。实际是 KDL 的**迭代**牛顿法。

> 教训：分析框架内部行为前，先读源码确认，别凭印象。

### 1.3 换插件方案评估（TRAC-IK / BioIK / pick_ik）
既然走插件，就换个"会用冗余避限位"的：
- **TRAC-IK**：SQP + 硬限位，`Distance` 模式最小位移。稳、成熟，但主要靠"更会解"避限位，不主动回中。需源码编（NLopt）。
- **BioIK**：全局优化，可叠加 `CenterJointsGoal`（回中）+ `MinimalDisplacementGoal`。最贴合"用好 7DOF"，但 memetic 有随机性（抖动风险）、维护弱、要调。
- **pick_ik**：MoveIt 官方新一代，梯度下降 + 代价函数含 `avoid_joint_limits_weight`/`center_joints_weight`/`minimal_displacement_weight`。Humble 有 apt 二进制、维护活跃。

**选定 pick_ik**：好装、官方维护、且有显式"避限位/回中/最小位移"三个代价项，正好覆盖需求。

### 1.4 pick_ik 安装（避坑点）
- **不能用 apt 装**：apt 版会拉 apt 版 `moveit_core`，而本项目 MoveIt2 是**源码编译**（`/docker_volume/ws_moveit2`）→ 两份 moveit_core 并存 → ABI 冲突，pick_ik 加载可能崩。
- **正解**：源码 clone 进独立 overlay 工作区 `/docker_volume/ws_ik_plugins`，针对源码版 moveit_core 编译；`setup_env.sh` 加一行 source 它。
- 插件类名：`pick_ik/PickIkPlugin`。

### 1.5 pick_ik 参数踩坑（"抖动/太钝"的第一次拉锯）
- **首版配错**：`avoid_joint_limits_weight=0.2` + `minimal_displacement_weight=0` → 优化器没锚定在当前构型，为避限位在关节空间**跳变** → 启动即抖、冲机报红。
  - 教训：**微分/伺服型 IK 里，minimal_displacement（连续性）必须占主导**，否则相邻周期解会跳。
- **过度矫正**：`minimal_displacement=1.0` → 解被死死钉住 → **欠跟手**（稳态跟随误差 ∝ 与位姿代价的权重比）。
- **落点**：`minimal_displacement_weight=0.3`、`avoid_joint_limits_weight=0.05`、`center_joints_weight=0`、`mode=local`、`rotation_scale=0.5`。
  - 注意 pick_ik 的目标权重**按平方计入**总代价（`weight²`），调参要按平方想。

---

## 第二部分：控制环的演变（IK 换完为什么还要大改）

pick_ik 解决了限位锁死，但"跟手 / 抖动 / 报红"的战场转到了外环控制节点 `franka_vr_vel.cpp`（速度解析控制：位姿误差 → 笛卡尔速度 twist → servo/IK → 关节）。

### 2.1 修掉的真 bug（这些是确凿的、必须修的）
1. **标定文件是 0 字节** → `json.load` 抛异常被吞 → `cal_offset=[0,0,0]` → 目标 = 手柄原始缩放坐标，和真机差几米 → 一启动满速猛冲报红。
   - **修**：废弃标定持久化，改**接入即重锚（bumpless clutch）**——按 Enter 那一刻用实时 TF 重算 offset，令 `target ≡ 当前末端位姿`，接入误差恒为 0。结构上不存在猛冲动力源。
2. **开环漂移** → 外环 `current_pose` 来自 `robot_state`（只被自己指令喂养、从不回读真机），模型与真机偏差逐周期累积 → 来回平移越走越偏。
   - **修**：外环误差改用**真机实测状态**（`getStateMonitor()->getCurrentState()`）+ 抗积分饱和回同步。
3. **四元数双覆盖 bug** → 姿态误差用 `getAngle()`∈[0,2π]，`w<0` 时走"长弧" → 手不动机械臂却持续朝一个方向转、反向也纠不回来。
   - **修**：`q_diff.w()<0` 时翻到 `w≥0`，始终走最短弧。
4. **axis×angle 数值不稳** → `getAxis()` 在小角度处除以趋 0 的数 → 轴方向乱跳 → 经旋转雅可比放大成多关节抖。
   - **修**：姿态误差改用**对数映射 `rot_error = 2·q_diff.vec()`**，单位四元数附近平滑无奇点。

### 2.2 PID 调参拉锯（大量无效尝试，值得记录教训）
- **误以为"不跟手是速度限制"** → 实际速度上限（1.5m/s 等）只在误差>0.25m 才触发，正常遥操碰不到。真正的滞后是 **P 控制一阶滞后 τ=1/kp**。
- **误以为"抖动就加 kd"** → 在这套系统里两种抖都不吃 kd：
  - 2Hz 延迟致振：D 经延迟**给谐振喂能量**（kd_ang=0 稳、0.25 就炸）。
  - 9.8Hz 噪声细抖：微分把测量噪声放大 200 倍，**kd 是噪声放大器**。
- **误以为"误差大就该加 kp"** → 一阶系统收敛时间只由 kp 决定，与初始误差无关；跟随滞后由目标速度决定，与误差大小无关。
- **大量 kp_ang/kd_ang 来回横跳**（3.5→6→8→2.5→4，kd 0→0.25→0.4→1→0）——**基本都在错误的层面上试**，因为真正的病根不在外环增益。

> 教训：调参前先分清"抖动是哪一类"（真实低频振荡 / 延迟致振 / 测量噪声），三类的解法完全不同甚至相反。

### 2.3 速度前馈（第一个大突破）
- 纯反馈 `v=kp·e` 的死穴：**必须先有误差才有速度**，跟移动目标必有滞后 `e_ss=ṙ/kp`。
- 加前馈 `v = FF·ṙ + kp·e + kd·ė`：主体运动由前馈（**开环、不进反馈环、不消耗稳定裕度**）提供，kp 只修残差 → 稳态误差降到 `(1-FF)·ṙ/kp`。
- **前馈不是"预测未来"**，是"读出目标当前速度并提前给量"（相邻两帧目标差分）。
- 三重防护：限幅（防丢帧假尖峰）+ 低通（差分放大噪声）+ 长间断清零。
- 效果：仿真里跟随滞后 115ms → ~30ms。

### 2.4 姿态抖动的数据诊断（排除法，全部靠录数据）
写了诊断脚本 `diag_record.py`，录 7 关节角/速度 + 下发指令 + 目标位姿。逐个假设用数据否定：

| 假设 | 数据结果 | 结论 |
|---|---|---|
| pick_ik 不收敛/跳解 | cmd 单步 1~7mrad、跳变≈0 | ❌ 指令平滑，不是跳解 |
| 内环软腕谐振 | 对齐后 \|cmd-q\|<0.04rad | ❌ 内环跟踪正常 |
| 腕奇异 | 姿态雅可比条件数≈1.6（极良态），与抖动零相关 | ❌ 不是奇异 |
| 零空间冗余乱动 | 抖动 0% 零空间 / 100% 任务空间 | ❌ 是末端本身振 |
| Quest 目标噪声 | hold 段目标干净（5~20mrad），末端却放大 2.5× | ❌ 目标干净 |
| 前馈放大噪声 | 关掉 FF 仍抖 | ❌ 不是前馈 |
| **延迟诱发的反馈环共振** | 抖动上游产生、7 关节相干于 **5.3Hz = π/T（T≈94ms）** | ✅ **确诊** |

关键工具：把关节速度抖动用雅可比分解成"任务空间 vs 零空间"，以及算姿态雅可比条件数（用 TF 实测末端位置校验过 DH 正确）。

### 2.5 失败的修法（都验证了无效或有害）
- **输出低通**（2Hz）：把 5.3Hz 极限环压到 2.3Hz，但**没杀掉**，只是加相位滞后把频率挪低。
- **抬内环腕关节阻尼 d**（j5/6/7 的 d 加大）：→ 引入 **9.8~342Hz 高频颤振**（把 franka 本就带毛刺的测速信号放大了）。回退。
- **降外环 kp_ang**（4→2.5）：对这个"任务空间但 kp 无关"的振荡**无效**——因为驱动源在延迟，不在 DC 增益。

### 2.6 Smith 预测器（制胜一招）
- 病根：反馈用的**实测位姿滞后了 ~90ms**（指令→真机动→被测到），这个环内延迟让姿态反馈在 ~2Hz 共振。
- **Smith 预测器**：反馈不用滞后的实测位姿，而用 **"实测位姿 + 最近 T 秒已下发指令的积分"** 预测当前真实位姿，再算误差 → **闭环里等效没有延迟 → 不共振**。
- 对纯延迟对象该预测精确（等于用"指令位姿"闭环）；有滞后时是很好的近似。
- 实现：环形缓冲存最近 `T_PREDICT+0.02s` 的 twist；接入 / 抗饱和卡死时清历史防跑飞。
- **效果**：转腕 2Hz 剧烈共振消失，不再易报红。**这是当前版本能成立的核心。**

### 2.7 遗留的 9.8Hz 细抖（软件到头了）
- 特征：~9.8Hz、全臂相干、幅度小、**指令端干净、只在实测里**。
- 结论：产生于**低通下游（内环 PD / 结构共振 / franka 自身）**，外环软件结构上够不着。
- 试过的软件解都有代价：降 kd（伤跟手，你否掉）、Butterworth 平滑（跟手影响小但你要最干净基线）。
- **剩下最该做的是物理层**：加固底座（工作台硬性固定），零控制代价。

---

## 第三部分：所有踩过的坑（速查表）

| # | 坑 | 后果 | 教训 |
|---|---|---|---|
| 1 | 误判 servo 用伪逆（实为 KDL 插件） | 一度想自写零空间/QP | 读源码确认框架内部行为 |
| 2 | pick_ik apt 安装 | 与源码版 moveit_core ABI 冲突 | 源码版 MoveIt 必须源码编插件 |
| 3 | pick_ik `minimal_displacement=0` | 关节跳变、启动报红 | 伺服 IK 连续性项必须主导 |
| 4 | pick_ik `minimal_displacement=1.0` | 欠跟手 | 连续性与跟手要平衡 |
| 5 | 标定文件 0 字节 | 目标差几米、猛冲报红 | 弃持久化，改接入即重锚 |
| 6 | 外环闭在内部模型 | 来回平移漂移 | 反馈必须闭真机 |
| 7 | 四元数双覆盖 | 持续旋转、反向纠不回 | 姿态误差走最短弧 |
| 8 | axis×angle | 小角度轴乱跳、多关节抖 | 用 2·vec 对数映射 |
| 9 | 以为不跟手=速度限制 | 白改 | 滞后来自 τ=1/kp，不是限速 |
| 10 | 以为抖动加 kd | 延迟致振被 D 喂能量、噪声被 D 放大 | 分清抖动类型 |
| 11 | kp_ang/kd_ang 反复横跳 | 大量无效试错 | 病根不在外环增益时，别在外环打转 |
| 12 | 抬内环腕 d 压振 | 引入高频颤振 | 高 d 放大测速噪声 |
| 13 | 输出低通治延迟致振 | 只挪频率不杀振 | 延迟致振要消延迟（Smith），不是低通 |
| 14 | Butterworth coeff 想设 0.5 | 会让 servo 起不来 | 该系数硬约束 >1.0 |

---

## 第四部分：为什么是当前这一版（设计理由）

当前版本 = 一串"针对确诊病因的对症解"的叠加，每一层都有数据支撑：

| 组件 | 解决的问题 | 为什么用它 |
|---|---|---|
| **pick_ik**（min_disp 主导 + 轻避限位） | 关节限位锁死 | 用 7DOF 冗余避限位；连续性保平滑 |
| **接入即重锚（bumpless）** | 启动猛冲报红 | 接入误差恒 0，结构上无猛冲源 |
| **闭真机反馈 + 抗饱和** | 来回平移漂移 | 误差基于真机，不累积 |
| **四元数最短弧 + 2·vec 对数映射** | 持续旋转 / 姿态抖 | 数值稳定、走近路 |
| **速度前馈（FF=0.8）** | 跟随滞后 | 开环补主运动，不吃稳定裕度 |
| **Smith 预测器（T=0.09）** | 2Hz 转腕共振 | 消除环内延迟，从根上不共振 |
| **输出低通 + 保守增益** | 稳定裕度 | 留余量、不冒进 |

**没有采用的**（及原因）：
- Ki 积分项：删了——移动目标上加相位滞后、windup 报红风险。
- kd_ang（姿态 D）：设 0——延迟致振下 D 是喂能量。
- 内环加 d：回退——放大测速噪声成高频颤。
- servo 平滑 Butterworth：回退——要最干净基线（但它是"压细抖不太伤跟手"的备选，随时可加回）。

---

## 第五部分：当前最好版本的完整参数

### 外环 `franka_vr_vel.cpp`
```
位置通道:  kp_lin=7.0   kd_lin=0.5   FF_GAIN=0.8
姿态通道:  kp_ang=4.0   kd_ang=0.0   FF_GAIN_ANG=0.3
D 项低通:  D_LPF=0.8
Smith 预测: T_PREDICT=0.09
输出低通:  beta_ang=0.06(≈2Hz)   beta_lin=0.11(≈4Hz)
前馈:      FF_LPF=0.7   FF_V_MAX=2.0   FF_W_MAX=3.0
限幅:      v_lin_max=1.5  a_lin_max=10  w_ang_max=2.0  a_ang_max=10
保护:      SOFT_START_SEC=0.5  ENGAGE_GAP_SEC=0.3  CMD_RESYNC_RAD=0.5
```

### 中环 servo `fr3_servo_{left,right}.yaml`
```
max_expected_latency: 0.03      # 纯调度延迟，越小越跟手（有下限）
use_smoothing: false
publish_period: 0.005 (200Hz)
```

### IK `kinematics_{left,right}.yaml`（pick_ik）
```
kinematics_solver: pick_ik/PickIkPlugin
mode: local
position_scale: 1.0    rotation_scale: 0.5
minimal_displacement_weight: 0.3   # 连续性主导
avoid_joint_limits_weight: 0.05    # 轻微避限位
center_joints_weight: 0.0
```

### 内环 `fr3_ros_controllers_{left,right}.yaml`（每关节 PD，力矩）
```
joint1-4: p=600 d=30      (基座，硬)
joint5:   p=250 d=10
joint6:   p=150 d=10
joint7:   p=50  d=5       (腕，软)
i=0 全部(无积分)
```

### 桥接 `start_franka_vr_dual.py`
```
SCALE = 2.0            # 手柄位移 -> franka 位移
对零: 按 Enter 接入时自动重锚(无需预先标定); 键 1/2 = 遥操中手动重新对零
```

---

## 第五点五部分：延迟预算(之前 vs 现在)

> ⚠️ 诚实标注：整条通路里**只有 `max_expected_latency` 是确切已知的配置值**，其余多为估计或从共振频率反推，**ADB 那段从未实测**。数字看趋势，不看精度。

### 通路里的延迟项
```
【参考路径】手在动 → 目标位姿送到控制器  (不进反馈环，只影响"手感滞后")
  Quest手柄 →(a)ADB logcat传输→ PC →(b)桥接采样70Hz →(c)服务调用 →(d)外环200Hz
【反馈路径】命令 → 真机动 → 被测到  (★这段的延迟导致了 2Hz 共振)
  →(e)pick_ik解算 →(f)max_expected_latency时间戳 →(g)控制器 →(h)真机+内环响应 →(i)关节状态回读
```

### 各延迟项对比
| 项 | 之前 | 现在 | 数值来源 |
|---|---|---|---|
| **(f) max_expected_latency** | **100ms** | **30ms** | ✅ **配置值，确切** |
| (a) ADB logcat 传输 | ~30–60ms | ~30–60ms(没动) | ⚠️ 估计，从未实测 |
| (b) 桥接采样 70Hz | ~7ms | ~7ms | 估计(半周期) |
| (d) 外环采样 200Hz | ~2.5ms | ~2.5ms | 估计(半周期) |
| (c)服务 + (e)IK解算 | ~1–3ms | ~1–3ms | 估计 |
| (g)(h)(i) 控制器+真机+回读 | ~15–35ms | ~15–35ms | 估计 |
| **反馈环延迟(致振那段)** | **~115–135ms** | **~45–65ms** | 反推/估计 |
| **端到端"手→臂"总滞后** | **~150–180ms** | **~85–115ms** | 反推/估计 |

### 三个关键认识
1. **唯一确切砍下来的是 `max_expected_latency`：100ms → 30ms，省 70ms。** 其余项没动。
2. **`max_expected_latency` 不是"系统的延迟"，是"你让每条指令晚多少执行"**——servo 把指令打上 `now + 此值` 的未来时间戳，本意是给控制器留插值提前量(拿延迟换平滑)，但对闭环遥操是纯亏。下限约 0.02–0.025(需 ≥3–5 个 publish_period，太小会顿挫/丢点)。
3. **Smith 预测器没有减少物理延迟**，是把延迟从"反馈环的稳定性方程"里消掉(等效无延迟)→ 共振死；但跟手滞后的物理下限仍在。**参考路径 ADB(最大的一块)完全没碰**，这才是"想根本降延迟"该动的地方(→ UDP)。

### 复盘反思：为什么没有一开始就调它
详见"第七部分"。简言之：**当时不知道"抖动是延迟致振"、也不知道"这个参数=人为加延迟"，是靠录数据、算 π/T、读源码一步步才定位到延迟这个根，以及这个参数的真实作用。诊断在前，才知道该动它。**

---

## 第六部分：遗留问题与后续方向

1. **9.8Hz 细抖** → 大概率结构共振。软件已到边际，建议**加固底座**（物理层，不伤跟手）。
2. **整体延迟**（~90-100ms，大头是 Quest→PC 的 ADB logcat 通路）→ 若要根本性提升跟手 + 让 Smith 预测更准，改 **UDP 直传**（工程量大，需改 APK/Unity）。
3. **T_PREDICT 精调**：现在 0.09 是估的，可扫 0.07~0.11 找最平滑点。
4. **代码隐患**：`target_pose` 全局在服务回调（写）与 pose_tracker 线程（读）间存在数据竞争（既有、影响极小，未修）。
5. **版本管理**：活跃代码在 docker 卷 `ws_franka_vr`，**未纳入 git**——建议尽快 `git init`，这版是几十轮试出来的，值得固化。

---

## 附：关键文件与路径

| 内容 | 路径（容器内 `/docker_volume` = 主机 `/home/wang/libfranka-docker/docker_volume`） |
|---|---|
| 外环控制节点 | `ws_franka_vr/src/franka_vr/src/franka_vr_vel.cpp` |
| IK 配置 | `ws_franka_vr/src/franka_vr/config/kinematics_{left,right}.yaml` |
| servo 配置 | `ws_franka_vr/src/franka_vr/config/fr3_servo_{left,right}.yaml` |
| 内环增益 | `ws_franka_vr/src/franka_vr/config/fr3_ros_controllers_{left,right}.yaml` |
| Quest 桥接 | `ws_franka_vr/src/franka_vr/oculus_reader/oculus_reader/start_franka_vr_dual.py` |
| 诊断录制脚本 | `ws_franka_vr/src/franka_vr/oculus_reader/oculus_reader/diag_record.py` |
| pick_ik 源码/安装 | `ws_ik_plugins/src/pick_ik`、`ws_ik_plugins/install/pick_ik` |
| MoveIt2 源码 | `ws_moveit2/src/moveit2`（含 moveit_servo `jointDeltaFromIK`） |
| 环境脚本 | `/docker_volume/setup_env.sh` |
| 运行容器 | `franka_dev`（机械臂栈）、`wuji-hand-teleop`（手套→灵巧手，独立） |

### 启动（真机，三终端）
```bash
# 终端1 机械臂+IK栈
source /docker_volume/setup_env.sh; export DISPLAY=:0
ros2 launch franka_vr dual_franka_teleop.launch.py \
    use_fake_hardware:=false robot_ip_left:=172.16.0.2 robot_ip_right:=172.16.0.3
# 终端2 Quest桥接
source /docker_volume/setup_env.sh
cd /docker_volume/ws_franka_vr/src/franka_vr/oculus_reader
python3 oculus_reader/start_franka_vr_dual.py
# 终端3(可选) 诊断录制
python3 oculus_reader/diag_record.py --side left
```
操作：按 **Enter** 接入(自动对零)；键 **1/2** 遥操中手动重新对零；手柄 **Grip** 控夹爪。

---

## 第七部分：复盘反思 —— "是不是早点调小 max_expected_latency 就少走弯路了？"

**答案：把它调小确实是关键一步，但"早点调"这个前提不成立——因为动它之前，必须先知道三件当时都不知道的事。**

### 动它需要的三个前提（当时都不具备）
1. **不知道抖动的病因是"延迟"**。一开始以为是 IK、是奇异、是内环谐振、是 pick_ik 参数……是靠"录数据 → 算出抖动频率 ≈ π/T → 反推出回路有 ~100ms 延迟"才第一次把矛头指向"延迟"。**没有这个诊断，根本不会去想延迟这条线。**
2. **不知道这个参数=人为加延迟**。它名字叫 "max_expected_latency"（预期最大延迟），字面像是"系统测量值/安全余量"，不像"你主动加的延迟"。是**读了 servo 源码**（`time_stamp = now + max_expected_latency`）才发现它把指令打上未来时间戳、真机就是晚这么多执行。**没读源码，不会知道它可调、更不会知道调它有用。**
3. **它只是延迟大盘里的一块，不是全部**。调它把反馈环延迟从 ~115ms 降到 ~45ms，缓解了共振**倾向**，但 ~45ms 的残余延迟仍会共振——**真正让共振消失的是 Smith 预测器**（把残余延迟也从环里消掉）。所以"只调它"并不够。

### 那些弯路是不是白走的？
不全是。很多弯路**排除了错误方向**（不是奇异、不是零空间、不是 pick_ik 跳解、不是 Quest 噪声），这些"排除"本身就是定位病因的必要步骤——正因为一个个排掉了，才反推到"延迟"这个唯一剩下的解释。**弯路里真正冤枉的是"反复横跳 kp_ang/kd_ang"那一段**（在错误的层面上试），如果更早坚持"先录数据定性、再动手"，那一段能省掉。

### 真正的教训（可迁移）
- **先诊断、后动手**：靠数据（频谱、雅可比、任务/零空间分解）定病因，而不是靠"手感"猜着调参。这是全程唯一真正高效的方法。
- **可疑的默认参数要读源码搞懂它到底在干嘛**：`max_expected_latency` 这种"名字有误导性"的参数，不读源码就永远想不到去调它。
- **一次只改一个变量**：横跳 kp/kd + 同时改多项，导致无法归因，是弯路的主要来源。
- **延迟是遥操的头号敌人，且要分清"环内延迟(致振)"和"参考延迟(手感滞后)"**：前者用 Smith/降 latency 治，后者只能靠降传输(ADB→UDP)。
