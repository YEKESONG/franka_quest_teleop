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

source "$ROOT/setup_env.sh"
export DISPLAY="${DISPLAY:-:0}"

if [[ "$FAKE" == "false" ]]; then
    echo "⚠ 真机模式：确认两台 FR3 已解抱闸、外部急停在手边、周围无人。"
    echo "  左臂 $IP_L / 右臂 $IP_R （active_arm=$ARM）"
    echo -n "  按 Enter 继续，Ctrl-C 取消 ... "; read -r _
fi

exec ros2 launch franka_vr dual_franka_teleop.launch.py \
    active_arm:="$ARM" \
    use_fake_hardware:="$FAKE" \
    load_gripper:="$GRIPPER" \
    use_rviz:="$RVIZ" \
    robot_ip_left:="$IP_L" \
    robot_ip_right:="$IP_R"
