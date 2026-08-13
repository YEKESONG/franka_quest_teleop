# 容器内一键 source 所有 ROS 工作区
#   用法：source /docker_volume/setup_env.sh
#
# 本仓库根目录 = 容器里的 /docker_volume（docker/docker-compose.yml 就是这么挂的）。
# 下面用脚本自身位置推断仓库根，所以放在别处挂载也照样能 source。
# source 顺序 = 编译顺序：franka_ros2 → MoveIt2 → IK 插件 → franka_vr，
# 后者的 overlay 必须后 source，否则会被底层的同名包盖掉。

_SE_SRC="${BASH_SOURCE[0]:-$0}"
FRANKA_VR_ROOT="$(cd "$(dirname "$_SE_SRC")" && pwd)"
unset _SE_SRC

source /opt/ros/humble/setup.bash
[ -f "$FRANKA_VR_ROOT/ros2_ws/install/setup.bash" ]      && source "$FRANKA_VR_ROOT/ros2_ws/install/setup.bash"      # franka_ros2 v2.3.0
[ -f "$FRANKA_VR_ROOT/ws_moveit2/install/setup.bash" ]   && source "$FRANKA_VR_ROOT/ws_moveit2/install/setup.bash"   # MoveIt2 2.13.0 (含 moveit_servo)
[ -f "$FRANKA_VR_ROOT/ws_ik_plugins/install/setup.bash" ] && source "$FRANKA_VR_ROOT/ws_ik_plugins/install/setup.bash" # IK 插件 (pick_ik)
[ -f "$FRANKA_VR_ROOT/ws_franka_vr/install/setup.bash" ] && source "$FRANKA_VR_ROOT/ws_franka_vr/install/setup.bash" # franka_vr (本项目)

# FastDDS 只走 UDP：franka_dev 容器是 IpcMode=private，FastDDS 的共享内存段跨不过
# IPC 命名空间，会出现"宿主机能看见话题、echo 收不到数据"。详见 udp_only.xml 注释。
[ -f "$FRANKA_VR_ROOT/udp_only.xml" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$FRANKA_VR_ROOT/udp_only.xml"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
