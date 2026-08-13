#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <moveit/utils/logger.hpp>
#include "franka_vr/srv/set_target_pose.hpp"
#include <Eigen/Geometry>

using namespace moveit_servo;

namespace
{
constexpr double POSITION_TOLERANCE = 0.002;     // 位置容差（米）
constexpr double ORIENTATION_TOLERANCE = 0.05;  // 角度容差（弧度）
}  // namespace

static geometry_msgs::msg::Pose target_pose;
static bool target_set = false;
// 软启动: 记录最近一次收到目标的时刻。pose_tracker 用它检测"遥操重新接入"——
// 接入瞬间把速度记忆清零并让速度上限从 0 渐入, 根治"标定完启动就猛冲触发 reflex 报红"。
static std::atomic<int64_t> last_target_ns{ 0 };

// ==== 速度前馈：目标速度估计 ====
// 必须在【服务回调】里差分——那才是目标真正更新的时刻(70Hz)。
// 若放到 200Hz 控制循环里差分，会有 2/3 的周期"目标没变" -> 得到 0,0,尖峰 的锯齿，不可用。
// 前馈是开环项(只依赖参考、不依赖测量)，故不进入闭环特征方程 -> 提升跟随但不消耗稳定裕度。
static geometry_msgs::msg::Pose prev_target_pose;
static rclcpp::Time prev_target_time;
static bool ff_primed = false;
static Eigen::Vector3d ff_v_filt = Eigen::Vector3d::Zero();
static Eigen::Vector3d ff_w_filt = Eigen::Vector3d::Zero();
static std::atomic<double> ff_vx{ 0.0 }, ff_vy{ 0.0 }, ff_vz{ 0.0 };
static std::atomic<double> ff_wx{ 0.0 }, ff_wy{ 0.0 }, ff_wz{ 0.0 };
namespace
{
constexpr double FF_LPF = 0.7;      // 目标速度低通(τ≈33ms)：差分必然放大噪声，必须滤
constexpr double FF_V_MAX = 2.0;    // 目标线速度限幅[m/s]：防丢帧/跳变产生的假尖峰被前馈放大送出
constexpr double FF_W_MAX = 3.0;    // 目标角速度限幅[rad/s]
}  // namespace

void set_target_pose_callback(const rclcpp::Node::SharedPtr& node,
                              const std::shared_ptr<franka_vr::srv::SetTargetPose::Request> request,
                              std::shared_ptr<franka_vr::srv::SetTargetPose::Response> response)
{
  target_pose = request->target_pose;
  target_set = true;
  const rclcpp::Time now = node->now();
  last_target_ns.store(now.nanoseconds());   // 软启动: 打时间戳供接入检测

  // ---- 速度前馈：用【相邻两次目标】和它们的真实时间间隔求目标速度 ----
  const double dt = ff_primed ? (now - prev_target_time).seconds() : 0.0;
  if (ff_primed && dt > 1e-4 && dt < 0.3)
  {
    Eigen::Vector3d v((target_pose.position.x - prev_target_pose.position.x) / dt,
                      (target_pose.position.y - prev_target_pose.position.y) / dt,
                      (target_pose.position.z - prev_target_pose.position.z) / dt);

    // 角速度: ω ≈ 2·(q_new ⊗ q_old⁻¹).vec / dt，同样必须做最短弧(双覆盖)修正
    tf2::Quaternion q_new, q_old;
    tf2::fromMsg(target_pose.orientation, q_new);
    tf2::fromMsg(prev_target_pose.orientation, q_old);
    tf2::Quaternion dq = q_new * q_old.inverse();
    dq.normalize();
    if (dq.w() < 0.0)
      dq = tf2::Quaternion(-dq.x(), -dq.y(), -dq.z(), -dq.w());
    Eigen::Vector3d w(2.0 * dq.x() / dt, 2.0 * dq.y() / dt, 2.0 * dq.z() / dt);

    if (v.norm() > FF_V_MAX) v = v.normalized() * FF_V_MAX;   // 限幅防假尖峰
    if (w.norm() > FF_W_MAX) w = w.normalized() * FF_W_MAX;
    ff_v_filt = FF_LPF * ff_v_filt + (1.0 - FF_LPF) * v;      // 低通
    ff_w_filt = FF_LPF * ff_w_filt + (1.0 - FF_LPF) * w;
  }
  else
  {
    ff_v_filt.setZero();   // 首帧 / 长间断(遥操中断后重新接入) -> 清零重新起步
    ff_w_filt.setZero();
  }
  ff_vx.store(ff_v_filt.x()); ff_vy.store(ff_v_filt.y()); ff_vz.store(ff_v_filt.z());
  ff_wx.store(ff_w_filt.x()); ff_wy.store(ff_w_filt.y()); ff_wz.store(ff_w_filt.z());
  prev_target_pose = target_pose;
  prev_target_time = now;
  ff_primed = true;

  response->success = true;
  response->message = "Target pose set successfully";
  // 70Hz×2臂 全量打日志会拖累实时性，改为限流
  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                       "Target pose: [%.3f, %.3f, %.3f] ff_v=%.2f m/s", target_pose.position.x,
                       target_pose.position.y, target_pose.position.z, ff_v_filt.norm());
}

Eigen::Vector3d compute_position_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  return Eigen::Vector3d(target.position.x - current.position.x, target.position.y - current.position.y,
                         target.position.z - current.position.z);
}

double compute_orientation_error(const geometry_msgs::msg::Pose& current, const geometry_msgs::msg::Pose& target)
{
  tf2::Quaternion q_current, q_target;
  tf2::fromMsg(current.orientation, q_current);
  tf2::fromMsg(target.orientation, q_target);
  return q_current.angleShortestPath(q_target);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");
  moveit::setNodeLoggerName(demo_node->get_name());

  // 基座/末端帧改为 ROS 参数（双臂: base=left_fr3_link0, tip=left_fr3_hand）。默认为单臂值，向后兼容。
  demo_node->declare_parameter<std::string>("base_frame", "fr3_link0");
  demo_node->declare_parameter<std::string>("tip_frame", "fr3_hand");
  const std::string base_frame = demo_node->get_parameter("base_frame").as_string();
  const std::string tip_frame = demo_node->get_parameter("tip_frame").as_string();
  RCLCPP_INFO(demo_node->get_logger(), "Servo frames: base=%s tip=%s", base_frame.c_str(), tip_frame.c_str());

  auto target_pose_service = demo_node->create_service<franka_vr::srv::SetTargetPose>(
      "set_target_pose", std::bind(&set_target_pose_callback, demo_node, std::placeholders::_1, std::placeholders::_2));

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  auto trajectory_outgoing_cmd_pub = demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params.command_out_topic, rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  const auto get_current_pose = [](const std::string& target_frame, const moveit::core::RobotStatePtr& robot_state) {
    return robot_state->getGlobalLinkTransform(target_frame);
  };

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      robot_state->getJointModelGroup(servo_params.move_group_name);

  servo.setCommandType(CommandType::TWIST);

  std::mutex pose_guard;
  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo.getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window, servo_params.max_expected_latency, demo_node->now());

    // 加速度限幅用：上一周期的指令速度 + 控制周期
    Eigen::Vector3d prev_lin_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_ang_vel = Eigen::Vector3d::Zero();
    const double dt_cmd = servo_params.publish_period;

    // ==== D 项状态：误差的低通微分 ====
    // ė = d(target)/dt − d(measured)/dt。手不动时 ė ≈ −末端实际速度，即纯阻尼，正是抑振所需。
    // 必须低通：原始微分会把测量噪声放大 1/dt=200 倍。
    Eigen::Vector3d prev_pos_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d d_err_filt = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_rot_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d d_rot_filt = Eigen::Vector3d::Zero();
    const double D_LPF = 0.8;            // 一阶低通系数(τ≈22ms, 截止≈7Hz)：滤噪但保留~2.5Hz振荡
    bool d_state_primed = false;         // 首个周期不算微分(否则从0跳变出巨大尖峰)

    // ==== 输出速度低通：掐掉 100ms 延迟造成的 5.3Hz(=π/T) 控制环极限环 ====
    // 诊断实测：转腕抖动是"上游"产生(指令本身抖)、7关节相干于 5.3Hz -> 延迟诱发的极限环。
    // 在 5.3Hz 处把回路增益滚降到 1 以下即可杀掉，且保留人手带宽(~1-2Hz)。
    // 一阶低通系数 β = dt/(dt+1/(2π·fc))。fc=2Hz, dt=0.005 -> β≈0.059。
    Eigen::Vector3d lp_lin_out = Eigen::Vector3d::Zero();
    Eigen::Vector3d lp_ang_out = Eigen::Vector3d::Zero();

    // ==== Smith 预测器：消除反馈环内的延迟，根除延迟致振(转腕 2Hz 共振) ====
    // 反馈用的实测位姿滞后了 T_PREDICT(命令->真机运动->被测到)。用"实测位姿 + 这段时间内
    // 已下发指令的积分"预测当前真实位姿，再用它算误差 -> 闭环里等效没有延迟 -> 不共振。
    // 纯延迟对象时该预测是精确的；有滞后时是很好的近似。T 过大会自激，从观测共振周期 π/5.3Hz≈0.09s 起调。
    struct TwistSample { rclcpp::Time t; Eigen::Vector3d v; Eigen::Vector3d w; };
    std::deque<TwistSample> twist_hist;
    const double T_PREDICT = 0.09;   // 反馈延迟估计[s]；抖动没压住可略增，出现新的自激则减小

    // ==== 软启动参数：遥操"重新接入"时速度上限由 0 渐入，防止启动瞬间猛冲报红 ====
    const double SOFT_START_SEC = 0.5;   // 渐入时长[s]（嫌起步慢调小；仍报红调大）
    const double ENGAGE_GAP_SEC = 0.3;   // 超过此间隔没有新目标 -> 判定为重新接入
    int64_t prev_seen_ns = 0;            // 上一周期看到的目标时间戳
    rclcpp::Time engage_time = demo_node->now();

    // 抗积分饱和阈值：指令态与真机的最大单关节偏差超过它就强制回同步
    // 注意: 正常快速运动时指令态本就领先真机一个跟踪滞后量(速度×滞后, 可达 0.3rad),
    // 阈值定太紧会在正常运动中反复触发回同步 -> 锯齿式振荡。只在"真被卡住"时才该触发。
    const double CMD_RESYNC_RAD = 0.50;  // ≈29°

    while (rclcpp::ok())
    {
      if (target_set)
      {
        std::lock_guard<std::mutex> pguard(pose_guard);

        // ---- 软启动：检测遥操重新接入(首次/中断后恢复)，复位速度记忆并重新渐入 ----
        const int64_t cur_ns = last_target_ns.load();
        const double gap_s = static_cast<double>(cur_ns - prev_seen_ns) * 1e-9;
        if (prev_seen_ns == 0 || gap_s > ENGAGE_GAP_SEC)
        {
          prev_lin_vel.setZero();   // 关键：清掉上次遗留速度，否则接入瞬间从旧速度起步=猛冲
          prev_ang_vel.setZero();
          d_err_filt.setZero();     // D 项状态一并复位，防止用上一次的陈旧微分
          d_rot_filt.setZero();
          d_state_primed = false;
          lp_lin_out.setZero();     // 输出低通状态复位
          lp_ang_out.setZero();
          twist_hist.clear();       // Smith 预测器历史复位
          engage_time = demo_node->now();
          RCLCPP_INFO(demo_node->get_logger(), "软启动：遥操接入，速度在 %.2fs 内由 0 渐入", SOFT_START_SEC);
        }
        prev_seen_ns = cur_ns;
        // soft: 0->1 的渐入系数，直接缩放本周期的速度上限
        const double soft =
            std::min(1.0, std::max(0.0, (demo_node->now() - engage_time).seconds() / SOFT_START_SEC));

        // ==== 闭环关键：外环误差用【真机实测状态】算，而不是内部积分模型 ====
        // 原来 current_pose 来自 robot_state（只被自己的指令喂养、从不回读真机），
        // 模型与真机的偏差会逐周期累积 -> 来回平移越走越偏。改用实测状态后偏差不再累积。
        const auto measured_state = planning_scene_monitor->getStateMonitor()->getCurrentState();

        // 抗积分饱和：真机被限位/reflex/碰撞卡住时，指令态会继续积分跑飞，
        // 解除阻塞瞬间就是一次大跳。偏差过大则把指令态强制拉回真机。
        {
          std::vector<double> cmd_q, meas_q;
          robot_state->copyJointGroupPositions(joint_model_group, cmd_q);
          measured_state->copyJointGroupPositions(joint_model_group, meas_q);
          double dev = 0.0;
          for (size_t i = 0; i < cmd_q.size() && i < meas_q.size(); ++i)
            dev = std::max(dev, std::abs(cmd_q[i] - meas_q[i]));
          if (dev > CMD_RESYNC_RAD)
          {
            robot_state->setJointGroupPositions(joint_model_group, meas_q);
            robot_state->update();
            twist_hist.clear();   // 真机被卡住时清预测历史，防止解除瞬间预测跑飞
            RCLCPP_WARN(demo_node->get_logger(), "指令态偏离真机 %.3f rad，已回同步(抗饱和)", dev);
          }
        }

        // 保存未经预测器修改的真机实测位姿。预测位姿可以用于补偿反馈延迟，
        // 但绝不能用于“已经到达”的判定：预测器可能比真机领先约 20 mm，
        // 若此时清掉 target_set，机械臂会提前停止且真实残差再也无法消除。
        const geometry_msgs::msg::Pose measured_pose =
            tf2::toMsg(get_current_pose(tip_frame, measured_state));
        geometry_msgs::msg::Pose current_pose = measured_pose;

        // ==== Smith 预测：current_pose(实测,滞后T) += 最近 T_PREDICT 秒的指令积分 -> 预测当下真实位姿 ====
        const rclcpp::Time t_now = demo_node->now();
        {
          Eigen::Vector3d dpos = Eigen::Vector3d::Zero();
          tf2::Quaternion dq(0, 0, 0, 1);
          for (const auto& s : twist_hist)
          {
            if ((t_now - s.t).seconds() > T_PREDICT)
              continue;                       // 只累加延迟窗口内的指令
            dpos += s.v * dt_cmd;
            const double wn = s.w.norm();
            if (wn > 1e-9)                     // 基座系角速度左乘累积
              dq = tf2::Quaternion(tf2::Vector3(s.w.x() / wn, s.w.y() / wn, s.w.z() / wn), wn * dt_cmd) * dq;
          }
          current_pose.position.x += dpos.x();
          current_pose.position.y += dpos.y();
          current_pose.position.z += dpos.z();
          tf2::Quaternion mq;
          tf2::fromMsg(current_pose.orientation, mq);
          mq = dq * mq;
          mq.normalize();
          current_pose.orientation = tf2::toMsg(mq);
        }

        // 预测误差仅供闭环控制使用；姿态预测误差在下方由四元数计算。
        Eigen::Vector3d pos_error = compute_position_error(current_pose, target_pose);

        // 停止条件必须看真机实测位姿，而不是 Smith 预测位姿。
        const Eigen::Vector3d measured_pos_error =
            compute_position_error(measured_pose, target_pose);
        const double measured_ori_error =
            compute_orientation_error(measured_pose, target_pose);

        if (measured_pos_error.norm() < POSITION_TOLERANCE &&
            measured_ori_error < ORIENTATION_TOLERANCE)
        {
          RCLCPP_INFO(demo_node->get_logger(),
                      "Measured pose reached target within tolerance (%.1f mm, %.2f deg)",
                      measured_pos_error.norm() * 1000.0,
                      measured_ori_error * 57.29577951308232);
          target_set = false;
          continue;
        }

        // ==== 限速 P 控制 + 加速度限幅：速度顶到接近 Franka 上限，加速度受限故不报红 ====
        // Franka FR3 硬阈值(超即红): 平移 1.7 m/s / 旋转 2.5 rad/s; 平移加速度 ~13 m/s²
        // 稳定判据: 带纯延迟T的一阶系统需 kp·T < π/2≈1.57。T≈0.1s 时 kp 上限约 15.7。
        // kd 提供阻尼，可在同等 kp 下压住振荡。Td = kd/kp ≈ 0.04s (约 T/2.5)。
        const double kp_lin = 7.0;       // 线速度比例增益
        const double kd_lin = 0.5;       // 线速度微分增益（抑振；嫌钝调小，仍晃调大）
        const double FF_GAIN = 0.8;      // 速度前馈系数（仿真最优≈0.8；1.0 因前馈同样被延迟而略过冲）
        const double v_lin_max = 1.5;    // 最大线速度 [m/s]（<1.7 硬限；大误差时的安全网）
        const double a_lin_max = 10.0;   // 最大线加速度 [m/s²]（<13 硬限；换向时间 = Δv/a）

        // ---- D 项：低通微分 ----
        Eigen::Vector3d d_err = Eigen::Vector3d::Zero();
        if (d_state_primed)
            d_err = (pos_error - prev_pos_error) / dt_cmd;
        prev_pos_error = pos_error;
        d_err_filt = D_LPF * d_err_filt + (1.0 - D_LPF) * d_err;

        // 前馈 + 向量式 PD：主体运动由前馈提供，kp 只负责修残差
        //   稳态误差 e_ss = (1-ff)·目标速度/kp  ← ff=0 时退化为 目标速度/kp（纯反馈的固有滞后）
        const Eigen::Vector3d v_ff(ff_vx.load(), ff_vy.load(), ff_vz.load());
        Eigen::Vector3d linear_vel = FF_GAIN * v_ff + kp_lin * pos_error + kd_lin * d_err_filt;
        const double v_cap = v_lin_max * soft;   // soft: 软启动渐入
        if (linear_vel.norm() > v_cap)
            linear_vel = linear_vel.normalized() * v_cap;
        {   // 加速度斜坡：本周期速度相对上周期的变化 ≤ a_max*dt
            Eigen::Vector3d dv = linear_vel - prev_lin_vel;
            double dv_max = a_lin_max * dt_cmd;
            if (dv.norm() > dv_max)
                dv = dv.normalized() * dv_max;
            linear_vel = prev_lin_vel + dv;
        }
        prev_lin_vel = linear_vel;

        // 计算角速度（同样限速 + 限加速度）
        tf2::Quaternion q_current, q_target;
        tf2::fromMsg(current_pose.orientation, q_current);
        tf2::fromMsg(target_pose.orientation, q_target);
        tf2::Quaternion q_diff = q_target * q_current.inverse();
        q_diff.normalize();
        // ★四元数双覆盖修正：q 与 -q 表示同一旋转，但 tf2 的 getAngle()=2*acos(w) ∈[0,2π]，
        //   w<0 时会返回"长弧"(>180°)，导致机械臂绕远路朝反方向持续旋转，且反向转手也纠正不回来
        //   （容差判断用的 angleShortestPath 却是最短弧，两者矛盾）。
        //   统一翻到 w>=0 分支，保证始终走最短弧 [0,π]。
        if (q_diff.w() < 0.0)
          q_diff = tf2::Quaternion(-q_diff.x(), -q_diff.y(), -q_diff.z(), -q_diff.w());
        const double kp_ang = 4.0;       // 角速度比例增益（回退到"较不坏"基线）
        const double kd_ang = 0.0;       // 角速度微分增益（关闭：D 经延迟对腕振是喂能量，且放大噪声）
        const double w_ang_max = 2.0;    // 最大角速度 [rad/s]（<2.5 硬限）
        const double a_ang_max = 10.0;   // 最大角加速度 [rad/s²]

        // 姿态误差用【对数映射】rot_error = 2·q_diff.vec()，取代 axis×angle。
        //   原因: tf2 的 getAxis() 在小角度(平滑跟踪常态)处是 vec/√(1-w²) 除以趋0的数，
        //   轴方向乱跳；这个方向抖动经旋转雅可比放大到多关节 -> "参与关节越多越抖"。
        //   2·vec 在单位四元数附近平滑、无奇点，且与前馈(也用 2·vec)口径一致。
        Eigen::Vector3d rot_error(2.0 * q_diff.x(), 2.0 * q_diff.y(), 2.0 * q_diff.z());
        Eigen::Vector3d d_rot = Eigen::Vector3d::Zero();
        if (d_state_primed)
            d_rot = (rot_error - prev_rot_error) / dt_cmd;
        prev_rot_error = rot_error;
        d_rot_filt = D_LPF * d_rot_filt + (1.0 - D_LPF) * d_rot;

        // 姿态前馈单独降权：w_ff 由四元数微分得来，比位置前馈噪声大，且只在转动时激活
        //   —— 和"转才抖"高度吻合。先降到 0.3 试；若转腕抖动明显减轻，说明前馈噪声是主凶之一。
        const double FF_GAIN_ANG = 0.3;   // 恢复一点姿态前馈(已证实不是抖的原因)，找回转动跟手
        const Eigen::Vector3d w_ff(ff_wx.load(), ff_wy.load(), ff_wz.load());
        Eigen::Vector3d angular_vel = FF_GAIN_ANG * w_ff + kp_ang * rot_error + kd_ang * d_rot_filt;
        const double w_cap = w_ang_max * soft;
        if (angular_vel.norm() > w_cap)
            angular_vel = angular_vel.normalized() * w_cap;
        d_state_primed = true;           // 微分状态已就绪（首周期跳过，避免 0->e 的假尖峰）
        {   // 角加速度斜坡
            Eigen::Vector3d dw = angular_vel - prev_ang_vel;
            double dw_max = a_ang_max * dt_cmd;
            if (dw.norm() > dw_max)
                dw = dw.normalized() * dw_max;
            angular_vel = prev_ang_vel + dw;
        }
        prev_ang_vel = angular_vel;

        // ==== 输出速度低通(对症 5.3Hz 极限环) ====
        //   角速度: fc≈2Hz(极限环在姿态通道, 强滤); 线速度: fc≈4Hz(平移本就稳, 弱滤少加滞后)
        const double beta_ang = 0.06;    // fc≈2.0Hz  @ dt=0.005
        const double beta_lin = 0.11;    // fc≈4.0Hz
        lp_ang_out = lp_ang_out + beta_ang * (angular_vel - lp_ang_out);
        lp_lin_out = lp_lin_out + beta_lin * (linear_vel  - lp_lin_out);
        angular_vel = lp_ang_out;
        linear_vel  = lp_lin_out;

        // 把本周期最终下发的 twist 存入历史，供 Smith 预测器积分；丢弃窗口外的旧样本
        twist_hist.push_back({ t_now, linear_vel, angular_vel });
        while (!twist_hist.empty() && (t_now - twist_hist.front().t).seconds() > T_PREDICT + 0.02)
          twist_hist.pop_front();

        TwistCommand target_twist{ base_frame,
                                   { linear_vel.x(), linear_vel.y(), linear_vel.z(), angular_vel.x(), angular_vel.y(),
                                     angular_vel.z() } };

        joint_state = servo.getNextJointState(robot_state, target_twist);
        StatusCode status = servo.getStatus();

        if (status != StatusCode::INVALID)
        {
          updateSlidingWindow(joint_state, joint_cmd_rolling_window, servo_params.max_expected_latency,
                              demo_node->now());
          if (const auto msg = composeTrajectoryMessage(servo_params, joint_cmd_rolling_window))
          {
            trajectory_outgoing_cmd_pub->publish(msg.value());
          }
          if (!joint_cmd_rolling_window.empty())
          {
            robot_state->setJointGroupPositions(joint_model_group, joint_cmd_rolling_window.back().positions);
            robot_state->setJointGroupVelocities(joint_model_group, joint_cmd_rolling_window.back().velocities);
          }
        }
      }
      tracking_rate.sleep();
    }
  };

  std::thread tracker_thread(pose_tracker);
  tracker_thread.detach();

  rclcpp::WallRate command_rate(70);
  RCLCPP_INFO_STREAM(demo_node->get_logger(), servo.getStatusMessage());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(demo_node);
    command_rate.sleep();
  }

  if (tracker_thread.joinable())
    tracker_thread.join();

  rclcpp::shutdown();
  return 0;
}
