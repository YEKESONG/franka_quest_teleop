# 单/双臂 bringup（Stage 1）：按 active_arm 起前缀化 FR3 控制栈。
# 仿真优先：默认 use_fake_hardware:=true。此阶段只做“两条臂能起来、能被控制器驱动、TF 连到同一 world”，
# 还不接 MoveIt Servo / 遥操 / 夹爪（后续阶段加）。
#
#   ros2 launch franka_vr dual_franka_bringup.launch.py                     # 双臂仿真, 带 RViz
#   ros2 launch franka_vr dual_franka_bringup.launch.py active_arm:=right   # 仅 03 右臂

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def arm_nodes(ns, prefix, base_xyz, use_fake_hardware, robot_ip, load_gripper):
    """为一条臂生成：robot_description + 命名空间控制栈 + world->{prefix}_base 静态 TF。"""
    arm_id = 'fr3'
    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id, arm_id + '.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'ros2_control': 'true',
            'robot_type': arm_id,
            'robot_ip': robot_ip,
            'hand': load_gripper,                 # 只影响 URDF 是否带 hand 链接
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': 'false',
            'arm_prefix': prefix,                 # -> 链接 {prefix}_fr3_link0 等
            'connected_to': f'{prefix}_base',     # 根链接 = {prefix}_base (URDF 会创建)
        }).toprettyxml(indent='  ')

    controllers = os.path.join(
        get_package_share_directory('franka_vr'),
        'config', f'fr3_ros_controllers_{prefix}.yaml')

    has_gripper = str(load_gripper).lower() == 'true'
    nodes = [
        # 有原装夹爪时订阅聚合后的 joint_states；无夹爪(Wuji)时直接订阅有效的臂状态，
        # 避免缺失 gripper source 让聚合器持续发出非法 JointState、导致 TF 停更。
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=ns, output='screen',
             parameters=[{
                 'robot_description': robot_description,
                 # 默认 20Hz 会把上游 30Hz joint_states 每两帧放行一帧，
                 # 使动态 TF 实际退化为约 15Hz。设为 60Hz 只解除限流；
                 # 上游仍是 30Hz，因此 /tf 最终仍约为 30Hz。
                 'publish_frequency': 60.0,
             }],
             remappings=[] if has_gripper else [('joint_states', 'franka/joint_states')]),
        # ros2_control（joint_state_broadcaster 发出的 joint_states 被重映射到 franka/joint_states）
        Node(package='controller_manager', executable='ros2_control_node',
             namespace=ns,
             parameters=[controllers, {'robot_description': robot_description}],
             remappings=[('joint_states', 'franka/joint_states')],
             output='screen'),
        # 命名空间下的 spawner（不带 -c，自动找 /{ns}/controller_manager）
        Node(package='controller_manager', executable='spawner',
             namespace=ns, arguments=['joint_state_broadcaster'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             namespace=ns, arguments=[f'{prefix}_fr3_arm_controller'], output='screen'),
        # 把这条臂挂到公共 world（左臂 +y、右臂 -y，先给个默认摆位，之后仿真里再调）
        Node(package='tf2_ros', executable='static_transform_publisher',
             name=f'static_world_{prefix}',
             arguments=[base_xyz[0], base_xyz[1], base_xyz[2],
                        '0', '0', '0', '1', 'world', f'{prefix}_base'],
             output='screen'),
    ]
    if has_gripper:
        nodes.insert(2, Node(
            package='joint_state_publisher', executable='joint_state_publisher',
            namespace=ns, name='joint_state_publisher',
            parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                         'rate': 30, 'use_robot_description': False}],
            output='screen'))
    return nodes


def launch_setup(context: LaunchContext):
    active_arm = LaunchConfiguration('active_arm').perform(context).lower()
    if active_arm not in ('left', 'right', 'both'):
        raise RuntimeError(
            f"active_arm must be 'left', 'right', or 'both', got: {active_arm!r}")

    ufh = LaunchConfiguration('use_fake_hardware').perform(context)
    ipl = LaunchConfiguration('robot_ip_left').perform(context)
    ipr = LaunchConfiguration('robot_ip_right').perform(context)
    lg = LaunchConfiguration('load_gripper').perform(context)

    sep = float(LaunchConfiguration('base_sep').perform(context))   # 两底座间距(米)
    hl, hr = str(sep / 2.0), str(-sep / 2.0)
    nodes = []
    # 单臂模式仍保留原双臂坐标系中的底座位置，避免已有相机/VR 外参失效。
    if active_arm in ('left', 'both'):
        nodes += arm_nodes('left', 'left', ['0', hl, '0'], ufh, ipl, lg)
    if active_arm in ('right', 'both'):
        nodes += arm_nodes('right', 'right', ['0', hr, '0'], ufh, ipr, lg)

    # 根据启用侧选择 RViz 配置，单臂模式不显示已停用侧的缺失模型。
    rviz_file = (
        'dual_franka.rviz'
        if active_arm == 'both'
        else f'single_franka_{active_arm}.rviz'
    )
    rviz_cfg = os.path.join(
        get_package_share_directory('franka_vr'), 'config', rviz_file)
    nodes.append(
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('use_rviz'))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('active_arm', default_value='both',
                              description='启动哪侧机械臂: left, right, both'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true',
                              description='仿真=true；真机=false'),
        DeclareLaunchArgument('robot_ip_left', default_value='172.16.0.2'),
        DeclareLaunchArgument('robot_ip_right', default_value='172.16.0.3'),
        DeclareLaunchArgument('load_gripper', default_value='true',
                              description='true 时 URDF 带 hand 链接（此阶段不起夹爪节点）'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_sep', default_value='1.05',
                              description='两台机器人底座的实际间距(米)，调到 RViz 对上真机'),
        OpaqueFunction(function=launch_setup),
    ])
