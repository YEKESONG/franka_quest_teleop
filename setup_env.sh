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

# ROS 的 setup.bash 内部引用了未定义的 AMENT_TRACE_SETUP_FILES 等变量，
# 在 set -u（严格模式脚本里很常见）下 source 它会直接 "unbound variable" 退出。
# 这里记住调用方的 -u 状态、source 期间关掉、结束时原样恢复，
# 这样从普通交互 shell 还是从 `set -euo pipefail` 的脚本里 source 都不会炸。
case $- in *u*) _SE_HAD_U=1 ;; *) _SE_HAD_U=0 ;; esac
set +u

source /opt/ros/humble/setup.bash
[ -f "$FRANKA_VR_ROOT/ros2_ws/install/setup.bash" ]      && source "$FRANKA_VR_ROOT/ros2_ws/install/setup.bash"      # franka_ros2 v2.3.0
[ -f "$FRANKA_VR_ROOT/ws_moveit2/install/setup.bash" ]   && source "$FRANKA_VR_ROOT/ws_moveit2/install/setup.bash"   # MoveIt2 2.13.0 (含 moveit_servo)
[ -f "$FRANKA_VR_ROOT/ws_ik_plugins/install/setup.bash" ] && source "$FRANKA_VR_ROOT/ws_ik_plugins/install/setup.bash" # IK 插件 (pick_ik)
[ -f "$FRANKA_VR_ROOT/ws_franka_vr/install/setup.bash" ] && source "$FRANKA_VR_ROOT/ws_franka_vr/install/setup.bash" # franka_vr (本项目)

# FastDDS 只走 UDP：franka_dev 容器是 IpcMode=private，FastDDS 的共享内存段跨不过
# IPC 命名空间，会出现"宿主机能看见话题、echo 收不到数据"。详见 udp_only.xml 注释。
[ -f "$FRANKA_VR_ROOT/udp_only.xml" ] && export FASTRTPS_DEFAULT_PROFILES_FILE="$FRANKA_VR_ROOT/udp_only.xml"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# oculus_reader 走 PYTHONPATH 而不是只靠 pip install -e：
# reader.py 用的是绝对包名 (from oculus_reader.FPS_counter import ...)，不在路径里就 ImportError。
# 而 pip -e 装在容器的 site-packages 里，`docker compose run --rm` 一退出就没了，
# 换个容器又得重装。PYTHONPATH 写在这个文件里，跟着仓库走，任何容器 source 完就能用。
export PYTHONPATH="$FRANKA_VR_ROOT/ws_franka_vr/src/franka_vr/oculus_reader${PYTHONPATH:+:$PYTHONPATH}"

[ "$_SE_HAD_U" = 1 ] && set -u        # 恢复调用方原来的 -u 状态
unset _SE_HAD_U
true                                   # 上一行 unset 的返回值不该成为 source 的结果
