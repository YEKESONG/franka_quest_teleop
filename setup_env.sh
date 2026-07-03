# 容器内一键 source 所有 ROS 工作区
# 用法：source /docker_volume/setup_env.sh
source /opt/ros/humble/setup.bash
[ -f /docker_volume/ros2_ws/install/setup.bash ]     && source /docker_volume/ros2_ws/install/setup.bash      # franka_ros2
[ -f /docker_volume/ws_moveit2/install/setup.bash ]  && source /docker_volume/ws_moveit2/install/setup.bash   # MoveIt2 (新版, 含 moveit_servo)
[ -f /docker_volume/ws_franka_vr/install/setup.bash ] && source /docker_volume/ws_franka_vr/install/setup.bash # franka_vr
