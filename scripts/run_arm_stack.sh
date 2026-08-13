#!/bin/bash
# 终端1：拉起机械臂控制栈（ros2_control + MoveIt Servo + 夹爪 + RViz）。
#
#   bash scripts/run_arm_stack.sh                 # 仿真双臂(默认, 不碰真机)
#   bash scripts/run_arm_stack.sh --real          # 真机双臂 172.16.0.2 / 172.16.0.3
#   bash scripts/run_arm_stack.sh --real --arm right   # 只起右臂(03)
#   bash scripts/run_arm_stack.sh --no-rviz
#
# 直接等价于：
#   ros2 launch franka_vr dual_franka_teleop.launch.py \
#        active_arm:=both use_fake_hardware:=false \
#        robot_ip_left:=172.16.0.2 robot_ip_right:=172.16.0.3
set -euo pipefail

# ROS 的 setup.bash 内部引用了未定义的 AMENT_TRACE_SETUP_FILES 等变量，
# 在 set -u 下会直接以 "unbound variable" 退出。source 期间临时关掉 -u。
ros_source() {
    [[ -f "$1" ]] || return 0
    set +u
    # shellcheck disable=SC1090
    source "$1"
    set -u
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ARM=both
FAKE=true
RVIZ=true
GRIPPER=true
IP_L=172.16.0.2
IP_R=172.16.0.3

while [[ $# -gt 0 ]]; do
    case "$1" in
        --real)        FAKE=false; shift ;;
        --sim)         FAKE=true; shift ;;
        --arm)         ARM="$2"; shift 2 ;;
        --no-rviz)     RVIZ=false; shift ;;
        --no-gripper)  GRIPPER=false; shift ;;
        --ip-left)     IP_L="$2"; shift 2 ;;
        --ip-right)    IP_R="$2"; shift 2 ;;
        -h|--help)     sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

case "$ARM" in left|right|both) ;; *) echo "--arm 只能是 left/right/both"; exit 1 ;; esac

ros_source "$ROOT/setup_env.sh"
export DISPLAY="${DISPLAY:-:0}"

if [[ "$FAKE" == "false" ]]; then
    echo "⚠ 真机模式：左臂 $IP_L / 右臂 $IP_R （active_arm=$ARM）"
    echo "  确认 FR3 已解抱闸、外部急停在手边、工作空间无人。"
fi

exec ros2 launch franka_vr dual_franka_teleop.launch.py \
    active_arm:="$ARM" \
    use_fake_hardware:="$FAKE" \
    load_gripper:="$GRIPPER" \
    use_rviz:="$RVIZ" \
    robot_ip_left:="$IP_L" \
    robot_ip_right:="$IP_R"
