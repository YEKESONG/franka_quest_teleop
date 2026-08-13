# 单/双臂遥操栈（Stage 2）：按 active_arm 启动控制栈、MoveIt Servo 和夹爪。
# 左臂 servo 在 /left（组 left_fr3_arm、帧 left_fr3_link0/left_fr3_hand、服务 /left/set_target_pose）；
# 右臂对称。夹爪 / 双手柄桥接在后续阶段加。
#
#   ros2 launch franka_vr dual_franka_teleop.launch.py                    # 双臂仿真
#   ros2 launch franka_vr dual_franka_teleop.launch.py active_arm:=right  # 仅 03 右臂

import os

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def servo_node_for(ns, prefix, use_fake_hardware, robot_ip, load_gripper, control_tip):
    """为一条臂造一个命名空间 servo 节点（手动组装 moveit 参数，全部前缀化）。"""
    fv = get_package_share_directory('franka_vr')
    fd = get_package_share_directory('franka_description')
    arm_id = 'fr3'

    robot_description = xacro.process_file(
        os.path.join(fd, 'robots', arm_id, arm_id + '.urdf.xacro'),
        mappings={
            'ros2_control': 'true', 'robot_type': arm_id, 'robot_ip': robot_ip,
            'hand': load_gripper, 'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': 'false', 'arm_prefix': prefix,
            'connected_to': f'{prefix}_base',
        }).toprettyxml(indent='  ')

    robot_description_semantic = xacro.process_file(
        os.path.join(fd, 'robots', arm_id, arm_id + '.srdf.xacro'),
        mappings={'arm_prefix': prefix, 'hand': load_gripper, 'robot_type': arm_id}
    ).toprettyxml(indent='  ')

    kinematics = _load_yaml(os.path.join(fv, 'config', f'kinematics_{prefix}.yaml'))
    joint_limits = _load_yaml(os.path.join(fv, 'config', f'joint_limits_{prefix}.yaml'))
    servo_yaml = _load_yaml(os.path.join(fv, 'config', f'fr3_servo_{prefix}.yaml'))

    return Node(
        package='franka_vr', executable='demo_franka_vr_vel',
        namespace=ns, output='screen',
        parameters=[
            {'moveit_servo': servo_yaml},
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': joint_limits},
            {'base_frame': f'{prefix}_fr3_link0'},   # C++ 读的参数
            {'tip_frame': f'{prefix}_fr3_{control_tip}'},
            {'update_period': 0.01},
            {'planning_group_name': f'{prefix}_fr3_arm'},
            {'use_sim_time': False},
        ],
    )


def gripper_include(ns, robot_ip, use_fake_hardware):
    """一条臂的夹爪：真机=franka_gripper_node(有 action)；仿真=fake 状态发布(仅关节)。"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
        launch_arguments={'namespace': ns, 'robot_ip': robot_ip,
                          'use_fake_hardware': use_fake_hardware, 'robot_type': 'fr3'}.items(),
    )


def launch_setup(context: LaunchContext):
    active_arm = LaunchConfiguration('active_arm').perform(context).lower()
    if active_arm not in ('left', 'right', 'both'):
        raise RuntimeError(
            f"active_arm must be 'left', 'right', or 'both', got: {active_arm!r}")

    ufh = LaunchConfiguration('use_fake_hardware').perform(context)
    ipl = LaunchConfiguration('robot_ip_left').perform(context)
    ipr = LaunchConfiguration('robot_ip_right').perform(context)
    lg = LaunchConfiguration('load_gripper').perform(context)
    control_tip = LaunchConfiguration('control_tip').perform(context).lower()
    if control_tip not in ('hand', 'link8'):
        raise RuntimeError(
            f"control_tip must be 'hand' or 'link8', got: {control_tip!r}")

    enabled_arms = []
    if active_arm in ('left', 'both'):
        enabled_arms.append(('left', ipl))
    if active_arm in ('right', 'both'):
        enabled_arms.append(('right', ipr))

    entities = [
        servo_node_for(side, side, ufh, robot_ip, lg, control_tip)
        for side, robot_ip in enabled_arms
    ]
    if lg.lower() == 'true':
        entities.extend(
            gripper_include(side, robot_ip, ufh)
            for side, robot_ip in enabled_arms
        )
    return entities


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare('franka_vr'), 'launch', 'dual_franka_bringup.launch.py'])]),
        launch_arguments={
            'active_arm': LaunchConfiguration('active_arm'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'robot_ip_left': LaunchConfiguration('robot_ip_left'),
            'robot_ip_right': LaunchConfiguration('robot_ip_right'),
            'base_sep': LaunchConfiguration('base_sep'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('active_arm', default_value='both',
                              description='启动哪侧机械臂: left, right, both'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        DeclareLaunchArgument('robot_ip_left', default_value='172.16.0.2'),
        DeclareLaunchArgument('robot_ip_right', default_value='172.16.0.3'),
        DeclareLaunchArgument('load_gripper', default_value='true'),
        DeclareLaunchArgument(
            'control_tip', default_value='hand',
            description='Servo 控制点: hand 或 link8；默认 hand 保持已有启动行为'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_sep', default_value='1.05',
                              description='两台机器人底座实际间距(米，中心到中心)'),
        bringup,
        OpaqueFunction(function=launch_setup),
    ])
