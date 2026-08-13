#!/bin/bash
# 终端2：Quest 手柄 → ROS 桥接节点。
#
#   bash scripts/run_vr_bridge.sh              # 双手柄 → 双臂
#   bash scripts/run_vr_bridge.sh --arm right  # 只用右手柄控右臂
#
# 键盘（必须前台跑在交互终端里，脚本要读单键）：
#   Enter  遥操 开/关（接入瞬间自动重锚：目标 ≡ 当前末端位姿，接入误差恒 0）
#   1 / 2  遥操进行中手动重新对零 左 / 右臂
#   手柄 Grip  > 0.6 关夹爪，< 0.4 开夹爪
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
ARM="${ACTIVE_ARM:-both}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --arm)     ARM="$2"; shift 2 ;;
        -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

case "$ARM" in left|right|both) ;; *) echo "--arm 只能是 left/right/both"; exit 1 ;; esac

ros_source "$ROOT/setup_env.sh"

# adb 必须先能看到头显，否则 reader 会一直空转
if ! adb devices | grep -qE "device$"; then
    echo "✗ adb 没看到 Quest。先跑： bash $ROOT/scripts/setup_quest_adb.sh" >&2
    exit 1
fi

cd "$ROOT/ws_franka_vr/src/franka_vr/oculus_reader"
exec python3 oculus_reader/start_franka_vr_dual.py --ros-args -p active_arm:="$ARM"
