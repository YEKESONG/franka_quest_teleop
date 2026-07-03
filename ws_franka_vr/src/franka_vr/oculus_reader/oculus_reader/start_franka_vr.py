from reader import OculusReader
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, Pose
from tf_transformations import (quaternion_from_matrix, quaternion_from_euler,
                                quaternion_multiply, quaternion_inverse)
import numpy as np
from franka_msgs.action import Move,Homing, Grasp
from franka_vr.srv import SetTargetPose
from termcolor import cprint
class OculusPublisher(Node):
    def __init__(self):
        super().__init__('oculus_reader')
        self.oculus_reader = OculusReader()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 70.0, self.timer_callback)

        # 声明 oculus_base 的参数
        self.declare_parameter('oculus_base.x', 0.05)
        self.declare_parameter('oculus_base.y', 0.0)
        self.declare_parameter('oculus_base.z', 0.45)
        self.declare_parameter('oculus_base.roll', 3.14 / 2.0)
        self.declare_parameter('oculus_base.pitch', 0.0)
        self.declare_parameter('oculus_base.yaw', -3.14 / 2)

        # 初始化并发布静态变换
        self.publish_static_transform()

        # 初始化 TF2 Buffer 和 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建服务客户端，用于末端控制
        self.cli = self.create_client(SetTargetPose, 'set_target_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service set_target_pose...')

        # 创建 Action 客户端，用于夹爪控制
        self.action_client = ActionClient(self, Move, '/franka_gripper/move')
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server /franka_gripper/move...')
        # 创建 Homing Action 客户端
        self.homing_client = ActionClient(self, Homing, '/franka_gripper/homing')
        while not self.homing_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server /franka_gripper/homing...')

        # 创建 Grasp Action 客户端
        self.grasp_client = ActionClient(self, Grasp, '/franka_gripper/grasp')
        while not self.grasp_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for action server /franka_gripper/grasp...')

        # 夹爪状态跟踪
        self.gripper_state = False  # None: 未初始化, True: 关闭, False: 打开
        self.scale = 1.0  # translation缩放因子(1.0 = 1:1, 手动多少 franka 动多少)
    

        self.last_gripper_send_time = self.get_clock().now()
        self.gripper_send_interval = 0.2  # 5 Hz

        self.scale = 1.0  # translation缩放因子(1.0 = 1:1, 手动多少 franka 动多少)

        #过滤跳变
        self.last_transform = None

        # ==== 离合(clutch) + 增量控制状态 ====
        self.clutch_engaged = False   # 扳机是否已按下并记录了基准
        self.ref_ctrl_pos = None      # 按下瞬间手柄位置 (world 系)
        self.ref_ee_pos = None        # 按下瞬间 franka 末端位置 (fr3_link0 系)
        self.ref_ee_ori = None        # 按下瞬间 franka 末端姿态 (先锁定不跟随)
        self.pos_scale = 1.0          # 手柄位移 -> franka 位移 比例
        self.prev_ctrl_pos = None     # 上一帧手柄位置(逐帧跳变检测)
        self.right_valid = False      # 本帧右手柄追踪是否有效
        self.right_pos = None         # 本帧右手柄位置(oculus_base 系, 仅调试用)
    def publish_static_transform(self):
        """发布从 world 到 oculus_base 的静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'oculus_base'

        x = self.get_parameter('oculus_base.x').get_parameter_value().double_value
        y = self.get_parameter('oculus_base.y').get_parameter_value().double_value
        z = self.get_parameter('oculus_base.z').get_parameter_value().double_value
        roll = self.get_parameter('oculus_base.roll').get_parameter_value().double_value
        pitch = self.get_parameter('oculus_base.pitch').get_parameter_value().double_value
        yaw = self.get_parameter('oculus_base.yaw').get_parameter_value().double_value

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        quat = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static transform from world to oculus_base: "
                              f"position=[{x}, {y}, {z}], euler=[{roll}, {pitch}, {yaw}]")

    def publish_transform(self, transform, name):
        """发布动态变换"""
        try:
            if transform.shape != (4, 4):
                self.get_logger().warning(f"Invalid transform shape for {name}: {transform.shape}")
                return

            rotation_matrix = transform[:3, :3]
            det = np.linalg.det(rotation_matrix)
            if not np.isclose(det, 1.0, atol=1e-5):
                self.get_logger().warning(f"Invalid rotation matrix for {name}: determinant = {det}")
                return

            translation = transform[:3, 3]
            if name == 'oculus_right':
                if self.last_transform is not None:
                    diff = np.linalg.norm(translation - self.last_transform)
                    if diff > 0.1:  # 相邻帧跳变阈值(米)，只滤真正的瞬跳
                        cprint(f"Translation difference for {name}: {diff}", 'red')
                        self.last_transform = translation
                        return
                self.last_transform = translation  # 正常帧也滚动更新基准
            
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'oculus_base'
            t.child_frame_id = name
            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])

            quat = quaternion_from_matrix(transform)
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.tf_broadcaster.sendTransform(t)
            # self.get_logger().info(f"Published transform for {name}")

        except Exception as e:
            self.get_logger().warning(f"Error publishing transform for {name}: {str(e)}")

    def send_target_pose(self, transform_stamped):
        """发送末端目标位姿给 set_target_pose 服务"""
        pose = Pose()
        pose.position.x = transform_stamped.transform.translation.x
        pose.position.y = transform_stamped.transform.translation.y
        pose.position.z = transform_stamped.transform.translation.z
        pose.orientation = transform_stamped.transform.rotation

        req = SetTargetPose.Request()
        req.target_pose = pose

        self.cli.call_async(req)

    def send_homing_goal(self):
        """发送夹爪打开命令 (Homing)"""
        goal_msg = Homing.Goal()
        self.homing_client.send_goal_async(goal_msg)
        self.get_logger().info("Sending gripper homing (open) command")
        self.gripper_state = False

    def send_grasp_goal(self):
        """发送夹爪关闭命令 (Grasp)"""
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.01    # 完全闭合
        goal_msg.speed = 0.10   # 速度 0.03 m/s
        goal_msg.force = 20.0   # 抓取力 50 N
        goal_msg.epsilon.inner = 0.1
        goal_msg.epsilon.outer = 0.2
        self.grasp_client.send_goal_async(goal_msg)
        self.get_logger().info("Sending gripper grasp (close) command")
        self.gripper_state = True

    def send_gripper_goal(self, width, action):
        """发送夹爪移动目标，不等待完成"""
        if action == 'grasp':
            self.gripper_state = True
            width = 0.01
        elif action == 'homing':
            self.gripper_state = False
            width = 0.08
        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = 0.05 # 设置一个合理的速度（m/s）

        self.get_logger().info(f"Sending gripper goal: width={width}, speed={goal_msg.speed}")
        self.action_client.send_goal_async(goal_msg)  # 只发送，不处理反馈或结果
        

    def timer_callback(self):
        transformations, buttons = self.oculus_reader.get_transformations_and_buttons()

        # 定义绕 Z 轴 -90 度的旋转矩阵
        rot_z_minus_90 = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # 设置旋转角度（90度转换为弧度）
        y_angle = np.deg2rad(60)  # 或者直接用 np.pi/2

        # 绕 Y 轴旋转 90° 的变换矩阵
        rot_y_90 = np.array([
            [np.cos(y_angle), 0.0, np.sin(y_angle), 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-np.sin(y_angle), 0.0, np.cos(y_angle), 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # ============ 发布手柄 TF (RViz 可视化 + 提供 world->oculus_right) ============
        # 右控制器：坐标变换与原作者一致(rot_z-90, rot_y60)，方向对齐靠这套 + world->oculus_base
        self.right_valid = False
        if 'r' in transformations:
            rp = transformations['r'].copy()
            if np.isclose(np.linalg.det(rp[:3, :3]), 1.0, atol=1e-3):  # 追踪质量: 行列式≈1
                rp[0:3, 3] *= self.scale
                rp = np.dot(rp, rot_z_minus_90)
                rp = np.dot(rp, rot_y_90)
                self.publish_transform(rp, 'oculus_right')
                self.right_valid = True
        if 'l' in transformations:
            lp = transformations['l'].copy()
            lp[0:3, 3] *= self.scale
            lp = np.dot(lp, rot_z_minus_90)
            self.publish_transform(lp, 'oculus_left')

        # ============ rightTrig: 绝对位姿映射 (像作者视频: 手柄绝对位置=夹爪绝对位置) ============
        # 目标 = fr3_link0 系下手柄的绝对位置。标定 oculus_base 让"手柄空间"与 franka 空间物理重合。
        trig = ('rightTrig' in buttons and buttons['rightTrig'][0] > 0.1)

        if not trig:
            if self.clutch_engaged:
                self.get_logger().info("Trigger RELEASED")
            self.clutch_engaged = False
            self.handle_gripper(buttons)
            return

        if not self.right_valid:      # 本帧追踪无效 → 不动
            self.handle_gripper(buttons)
            return

        try:
            # 手柄在 franka 基座坐标系里的绝对位姿(经 world->oculus_base 标定对齐)
            c = self.tf_buffer.lookup_transform('fr3_link0', 'oculus_right', rclpy.time.Time())
            ctrl_pos = np.array([c.transform.translation.x,
                                 c.transform.translation.y,
                                 c.transform.translation.z])
            q = c.transform.rotation
            ctrl_quat = np.array([q.x, q.y, q.z, q.w])   # 手柄当前姿态(franka系)
        except Exception:
            self.handle_gripper(buttons)
            return

        # 首次按下: 记录 手柄姿态基准 + 夹爪姿态基准 (姿态做相对增量, 按下瞬间不翻)
        if not self.clutch_engaged:
            try:
                ee = self.tf_buffer.lookup_transform('fr3_link0', 'fr3_hand', rclpy.time.Time())
                eo = ee.transform.rotation
            except Exception:
                self.handle_gripper(buttons)
                return
            self.ref_ctrl_quat = ctrl_quat.copy()                       # 手柄姿态基准
            self.ref_ee_quat = np.array([eo.x, eo.y, eo.z, eo.w])       # 夹爪姿态基准
            self.prev_ctrl_pos = ctrl_pos.copy()
            self.clutch_engaged = True
            self.get_logger().info("Trigger ENGAGED (6D 绝对映射)")

        # 跳变防护: 单帧手柄瞬跳过大 → 忽略本帧
        frame_step = np.linalg.norm(ctrl_pos - self.prev_ctrl_pos)
        if frame_step > 0.08:
            self.get_logger().warn(f"跳变忽略: {frame_step:.3f}m")
            self.prev_ctrl_pos = ctrl_pos.copy()
            self.handle_gripper(buttons)
            return
        self.prev_ctrl_pos = ctrl_pos.copy()

        # 姿态: 夹爪目标 = 夹爪基准 ⊕ 手柄相对旋转
        #   q_delta = ctrl_quat * ref_ctrl_quat^-1  (手柄从按下到现在转了多少)
        #   q_target = q_delta * ref_ee_quat        (把这个旋转叠加到夹爪基准上)
        q_delta = quaternion_multiply(ctrl_quat, quaternion_inverse(self.ref_ctrl_quat))
        q_target = quaternion_multiply(q_delta, self.ref_ee_quat)
        q_target = q_target / np.linalg.norm(q_target)   # 归一化

        pose = Pose()
        pose.position.x = float(ctrl_pos[0])
        pose.position.y = float(ctrl_pos[1])
        pose.position.z = float(ctrl_pos[2])
        pose.orientation.x = float(q_target[0])
        pose.orientation.y = float(q_target[1])
        pose.orientation.z = float(q_target[2])
        pose.orientation.w = float(q_target[3])
        req = SetTargetPose.Request()
        req.target_pose = pose
        self.cli.call_async(req)
        self.get_logger().info(
            f"[SEND] pos=({ctrl_pos[0]:.3f},{ctrl_pos[1]:.3f},{ctrl_pos[2]:.3f})")

        self.handle_gripper(buttons)
        return

    def handle_gripper(self, buttons):
        """rightGrip 控制夹爪: >0.6 关, <0.4 开"""
        if 'rightGrip' not in buttons:
            return
        grip_value = buttons['rightGrip'][0]
        if grip_value > 0.6 and self.gripper_state == False:
            self.send_grasp_goal()
        elif grip_value < 0.4 and self.gripper_state == True:
            self.send_homing_goal()

def main():
    rclpy.init()
    node = OculusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()