#!/usr/bin/env python3
# 单/双臂 Quest 遥操桥接（Stage 4），由 ROS 参数 active_arm 选择。
#   左手柄(oculus_left) -> /left/set_target_pose ,  leftGrip 控左夹爪
#   右手柄(oculus_right)-> /right/set_target_pose,  rightGrip控右夹爪
# 映射：绝对位置 + 绝对姿态 + SCALE 倍缩放。
# 对零方式：按 Enter 接入遥操时【自动重锚】(target ≡ 当前末端位姿，接入误差恒为0)，
#           无需事先标定；键 1/2 = 遥操进行中【手动就地重新对零】(可选)。不再有标定存盘。
# 夹爪 action 客户端非阻塞（仿真无 action 服务也不卡）。

import sys
import select
import termios
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import (TransformBroadcaster, StaticTransformBroadcaster,
                     Buffer, TransformListener)
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from tf_transformations import (quaternion_from_euler, quaternion_multiply,
                                quaternion_inverse)
from franka_msgs.action import Homing, Grasp
from franka_vr.srv import SetTargetPose

SCALE = 2.0            # 手柄位移 -> franka 位移 (1:2)
JUMP_MAX = 0.08        # 单帧跳变丢弃阈值(米)
GRIP_CLOSE = 0.6
GRIP_OPEN = 0.4


class KeyboardListener:
    """在当前终端非阻塞读单键(Enter/数字)，回调 on_key。需在交互 TTY 里前台运行。"""

    def __init__(self, on_key, logger):
        self.on_key = on_key
        self._stop = False
        self._ok = False
        try:
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            new = termios.tcgetattr(self.fd)
            new[3] &= ~(termios.ICANON | termios.ECHO)   # 关行缓冲 + 关回显
            termios.tcsetattr(self.fd, termios.TCSANOW, new)
            self._ok = True
            threading.Thread(target=self._run, daemon=True).start()
        except Exception as e:
            logger.warn(f"键盘监听不可用(需在交互终端里前台运行): {e}")

    def _run(self):
        while not self._stop:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    ch = sys.stdin.read(1)
                except Exception:
                    break
                if ch:
                    self.on_key(ch)

    def restore(self):
        self._stop = True
        if self._ok:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            except Exception:
                pass


class ArmBridge:
    """封装一条臂：TF 帧 / set_target_pose 服务 / 夹爪 / 重锚 offset / 跳变防护。"""

    def __init__(self, node: Node, side: str):
        self.node = node
        self.side = side                       # 'left' / 'right'
        self.ns = side                          # 命名空间 /left /right
        self.base_frame = f'{side}_fr3_link0'
        self.tip_frame = f'{side}_fr3_hand'
        self.ctrl_frame = f'oculus_{side}'
        self.grip_key = 'rightGrip' if side == 'right' else 'leftGrip'

        self.cli = node.create_client(SetTargetPose, f'/{self.ns}/set_target_pose')
        self.dbg_tgt = node.create_publisher(PoseStamped, f'/{self.ns}/debug_target', 10)  # 诊断:发下去的目标位姿
        self.grasp_cli = ActionClient(node, Grasp, f'/{self.ns}/franka_gripper/grasp')
        self.homing_cli = ActionClient(node, Homing, f'/{self.ns}/franka_gripper/homing')

        self.clutch = False
        self.prev_ctrl_pos = None
        self.cal_offset = np.zeros(3)
        self.ori_offset = np.array([0.0, 0.0, 0.0, 1.0])   # 姿态偏置(单位四元数 xyzw)
        self.anchored = False              # 未成功重锚前，绝不发送目标
        self.gripper_closed = False
        self.request_calib = False

    # ---------- 重锚：令 target ≡ 当前末端位姿（bumpless，接入误差恒为 0）----------
    # 不再做标定持久化：每次接入都用实时 TF 重算，既免除"存盘失效/为空"这类故障，
    # 也让 SCALE 等参数改动后无需重新标定。
    def _anchor(self, tf_buffer, p_ctrl, q_ctrl):
        """重算位置/姿态偏置，使此刻目标正好等于机械臂当前末端位姿。
        成功 True；失败必须 False —— 调用方据此拒绝发送目标（安全关键）。"""
        try:
            e = tf_buffer.lookup_transform(self.base_frame, self.tip_frame, rclpy.time.Time())
        except Exception as ex:
            self.node.get_logger().error(
                f"[{self.side}] 重锚失败(查不到末端TF): {ex} —— 拒绝发送目标")
            return False
        p_ee = np.array([e.transform.translation.x, e.transform.translation.y,
                         e.transform.translation.z])
        q_ee = np.array([e.transform.rotation.x, e.transform.rotation.y,
                         e.transform.rotation.z, e.transform.rotation.w])
        # target = p_ctrl + cal_offset  => 取 cal_offset = p_ee - p_ctrl，则此刻 target == p_ee
        self.cal_offset = p_ee - p_ctrl
        # q_target = q_ctrl ⊗ ori_offset => 取 ori_offset = q_ctrl⁻¹ ⊗ q_ee，则此刻 q_target == q_ee
        self.ori_offset = quaternion_multiply(quaternion_inverse(q_ctrl), q_ee)
        self.anchored = True
        self.node.get_logger().info(
            f"[{self.side}] 重锚完成 锚点=({p_ee[0]:.3f},{p_ee[1]:.3f},{p_ee[2]:.3f}) 接入误差=0")
        return True

    # ---------- 手动重锚(键 1/2)：不关遥操也能就地重新对零 ----------
    def handle_calibration(self, tf_buffer):
        if not self.request_calib:
            return
        self.request_calib = False
        try:
            c = tf_buffer.lookup_transform(self.base_frame, self.ctrl_frame, rclpy.time.Time())
        except Exception as ex:
            self.node.get_logger().error(f"[{self.side}] 手动重锚失败(查不到手柄TF): {ex}")
            return
        q = c.transform.rotation
        self._anchor(tf_buffer,
                     np.array([c.transform.translation.x, c.transform.translation.y,
                               c.transform.translation.z]),
                     np.array([q.x, q.y, q.z, q.w]))

    # ---------- 遥操主循环(由 Enter 开关驱动) ----------
    def handle_teleop(self, enabled, tf_buffer):
        if not enabled:
            self.clutch = False
            return
        try:
            c = tf_buffer.lookup_transform(self.base_frame, self.ctrl_frame, rclpy.time.Time())
        except Exception:
            return
        ctrl_pos = np.array([c.transform.translation.x, c.transform.translation.y,
                             c.transform.translation.z])
        q = c.transform.rotation
        ctrl_quat = np.array([q.x, q.y, q.z, q.w])

        if not self.clutch:
            # ★接入即重锚：令 target ≡ 当前末端位姿，接入误差恒为 0
            #   -> 结构上不存在"猛冲"的动力来源，根治标定失效导致的启动报红。
            if not self._anchor(tf_buffer, ctrl_pos, ctrl_quat):
                return                       # 重锚失败：本帧不发目标，下帧自动重试
            self.prev_ctrl_pos = ctrl_pos.copy()
            self.clutch = True
            self.node.get_logger().info(f"[{self.side}] 遥操接入(已重锚)")

        if not self.anchored:                # 双保险：未重锚绝不发目标
            return

        step = np.linalg.norm(ctrl_pos - self.prev_ctrl_pos)
        if step > JUMP_MAX:
            self.node.get_logger().warn(f"[{self.side}] 跳变忽略 {step:.3f}m")
            self.prev_ctrl_pos = ctrl_pos.copy()
            return
        self.prev_ctrl_pos = ctrl_pos.copy()

        target = ctrl_pos + self.cal_offset        # 绝对位置 + 重锚偏置
        q_target = quaternion_multiply(ctrl_quat, self.ori_offset)   # 姿态: 手柄 ⊗ 重锚偏置
        q_target = q_target / np.linalg.norm(q_target)

        pose = Pose()
        pose.position.x = float(target[0])
        pose.position.y = float(target[1])
        pose.position.z = float(target[2])
        pose.orientation.x = float(q_target[0])
        pose.orientation.y = float(q_target[1])
        pose.orientation.z = float(q_target[2])
        pose.orientation.w = float(q_target[3])
        req = SetTargetPose.Request()
        req.target_pose = pose
        self.cli.call_async(req)
        ps = PoseStamped()                       # 诊断:同一目标位姿发到话题供录制
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.header.frame_id = self.base_frame
        ps.pose = pose
        self.dbg_tgt.publish(ps)

    # ---------- 夹爪(非阻塞) ----------
    def handle_gripper(self, buttons):
        if self.grip_key not in buttons:
            return
        v = buttons[self.grip_key][0]
        if v > GRIP_CLOSE and not self.gripper_closed:
            self.gripper_closed = True
            if self.grasp_cli.server_is_ready():
                g = Grasp.Goal()
                g.width = 0.01
                g.speed = 0.10
                g.force = 20.0
                g.epsilon.inner = 0.1
                g.epsilon.outer = 0.2
                self.grasp_cli.send_goal_async(g)
        elif v < GRIP_OPEN and self.gripper_closed:
            self.gripper_closed = False
            if self.homing_cli.server_is_ready():
                self.homing_cli.send_goal_async(Homing.Goal())


class OculusDualPublisher(Node):
    def __init__(self):
        super().__init__('oculus_reader_dual')
        self.declare_parameter('active_arm', 'both')
        active_arm = self.get_parameter('active_arm').value.lower()
        if active_arm not in ('left', 'right', 'both'):
            raise ValueError(
                f"active_arm must be 'left', 'right', or 'both', got: {active_arm!r}")
        self.active_sides = (
            ('left', 'right') if active_arm == 'both' else (active_arm,)
        )

        from reader import OculusReader
        self.oculus_reader = OculusReader()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # world -> oculus_base 静态变换(与单臂一致)
        for name, default in (('x', 0.05), ('y', 0.0), ('z', 0.45),
                              ('roll', 3.14 / 2.0), ('pitch', 0.0), ('yaw', -3.14 / 2)):
            self.declare_parameter(f'oculus_base.{name}', default)
        self._publish_static()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_left = None
        self.last_right = None

        self.arms = {
            side: ArmBridge(self, side)
            for side in self.active_sides
        }

        self.teleop_enabled = False                       # 键盘 Enter 切换
        self.kbd = KeyboardListener(self._on_key, self.get_logger())

        self.timer = self.create_timer(1.0 / 70.0, self.timer_callback)
        arm_label = {'left': '02 左臂', 'right': '03 右臂', 'both': '02/03 双臂'}[active_arm]
        self.get_logger().info(
            f"{arm_label}遥操桥接就绪 | 键盘: [Enter]遥操开关(接入时自动对零) "
            "[1]/[2]手动重新对零 左/右臂 | 当前遥操: OFF")

    def _on_key(self, ch):
        if ch in ('\r', '\n'):
            self.teleop_enabled = not self.teleop_enabled
            for arm in self.arms.values():
                arm.clutch = False
            self.get_logger().info(
                f"[KEY] 遥操 {'ON ▶ 开始跟随' if self.teleop_enabled else 'OFF ■ 停止'}")
        elif ch == '1':
            if 'left' in self.arms:
                self.arms['left'].request_calib = True
                self.get_logger().info("[KEY] 手动重新对零 左臂")
        elif ch == '2':
            if 'right' in self.arms:
                self.arms['right'].request_calib = True
                self.get_logger().info("[KEY] 手动重新对零 右臂")

    def _publish_static(self):
        gp = lambda n: self.get_parameter(f'oculus_base.{n}').get_parameter_value().double_value
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'oculus_base'
        t.transform.translation.x = float(gp('x'))
        t.transform.translation.y = float(gp('y'))
        t.transform.translation.z = float(gp('z'))
        quat = quaternion_from_euler(gp('roll'), gp('pitch'), gp('yaw'))
        t.transform.rotation.x, t.transform.rotation.y = float(quat[0]), float(quat[1])
        t.transform.rotation.z, t.transform.rotation.w = float(quat[2]), float(quat[3])
        self.static_tf_broadcaster.sendTransform(t)

    def _publish_ctrl(self, transform, name, last_attr):
        rot_matrix = transform[:3, :3]
        if not np.isclose(np.linalg.det(rot_matrix), 1.0, atol=1e-3):
            return False
        translation = transform[:3, 3]
        last = getattr(self, last_attr)
        if last is not None and np.linalg.norm(translation - last) > 0.1:
            setattr(self, last_attr, translation)
            return False
        setattr(self, last_attr, translation)

        from tf_transformations import quaternion_from_matrix
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'oculus_base'
        t.child_frame_id = name
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        quat = quaternion_from_matrix(transform)
        t.transform.rotation.x, t.transform.rotation.y = float(quat[0]), float(quat[1])
        t.transform.rotation.z, t.transform.rotation.w = float(quat[2]), float(quat[3])
        self.tf_broadcaster.sendTransform(t)
        return True

    def timer_callback(self):
        transformations, buttons = self.oculus_reader.get_transformations_and_buttons()

        rot_z = np.array([[0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        ya = np.deg2rad(60)
        rot_y = np.array([[np.cos(ya), 0.0, np.sin(ya), 0.0], [0.0, 1.0, 0.0, 0.0],
                          [-np.sin(ya), 0.0, np.cos(ya), 0.0], [0.0, 0.0, 0.0, 1.0]])

        if 'right' in self.arms and 'r' in transformations:
            rp = transformations['r'].copy()
            rp[0:3, 3] *= SCALE
            rp = np.dot(np.dot(rp, rot_z), rot_y)
            self._publish_ctrl(rp, 'oculus_right', 'last_right')
        if 'left' in self.arms and 'l' in transformations:
            lp = transformations['l'].copy()
            lp[0:3, 3] *= SCALE
            lp = np.dot(np.dot(lp, rot_z), rot_y)
            self._publish_ctrl(lp, 'oculus_left', 'last_left')

        for arm in self.arms.values():
            arm.handle_calibration(self.tf_buffer)
            arm.handle_teleop(self.teleop_enabled, self.tf_buffer)
            arm.handle_gripper(buttons)


def main():
    rclpy.init()
    node = OculusDualPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kbd.restore()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
