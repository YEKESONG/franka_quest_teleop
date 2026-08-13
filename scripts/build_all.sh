#!/bin/bash
# 容器内一键编译四个工作区（顺序不能乱，后者依赖前者）。
#
#   bash /docker_volume/scripts/build_all.sh              # 全编
#   bash /docker_volume/scripts/build_all.sh -j 2         # 限制并行度(内存小/双核机器)
#   bash /docker_volume/scripts/build_all.sh --from 3     # 从第 3 步接着编(前面已编好)
#   bash /docker_volume/scripts/build_all.sh --only 4     # 只编第 4 步(改了 franka_vr 时最常用)
#
# 编译顺序与依赖：
#   1) ros2_ws       franka_ros2 v2.3.0 + franka_description 1.6.1  (依赖 apt 的 libfranka 0.20.4)
#   2) ws_moveit2    MoveIt2 2.13.0 源码                            (最重, 双核约数小时)
#   3) ws_ik_plugins pick_ik                                        (必须针对源码版 moveit_core 编)
#   4) ws_franka_vr  franka_vr 本体                                 (依赖 1+2)
#   5) pip -e oculus_reader                                         (可选: setup_env.sh 已用
#                                                                    PYTHONPATH 覆盖同样的作用。
#                                                                    pip 装在容器 site-packages 里,
#                                                                    run --rm 退出即失效)
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
JOBS=""
FROM=1
ONLY=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -j|--jobs)  JOBS="$2"; shift 2 ;;
        --from)     FROM="$2"; shift 2 ;;
        --only)     ONLY="$2"; shift 2 ;;
        -h|--help)  sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

PAR=()
if [[ -n "$JOBS" ]]; then
    PAR=(--parallel-workers "$JOBS")
    # 只限 colcon 的"同时编几个包"是不够的：每个包内部的 make 默认还会按 CPU 核数
    # 再开一层并行。20 核 15G 内存的机器上，这一层能把内存瞬间打满 → 编译进程被 OOM
    # killer 干掉，表现为"编到一半莫名失败/整机卡死"。MAKEFLAGS 把内层也一起摁住。
    export MAKEFLAGS="-j${JOBS}"
fi

step_enabled() {   # $1 = 步骤号
    [[ -n "$ONLY" ]] && { [[ "$1" == "$ONLY" ]]; return; }
    [[ "$1" -ge "$FROM" ]]
}

banner() { echo; echo "=============== [$1/5] $2 ==============="; }

if [[ ! -d /opt/ros/humble ]]; then
    echo "✗ 找不到 /opt/ros/humble —— 这个脚本要在 franka_dev 容器里跑。" >&2
    exit 1
fi
ros_source /opt/ros/humble/setup.bash

# ---- 1) franka_ros2 ----------------------------------------------------------
if step_enabled 1; then
    banner 1 "ros2_ws: franka_ros2 v2.3.0 + franka_description 1.6.1"
    cd "$ROOT/ros2_ws"
    colcon build "${PAR[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
ros_source "$ROOT/ros2_ws/install/setup.bash"

# ---- 2) MoveIt2 --------------------------------------------------------------
if step_enabled 2; then
    banner 2 "ws_moveit2: MoveIt2 2.13.0 源码（最重的一步）"
    echo "    提示: 内存 < 16G 建议 -j 2；这一步失败最常见的原因是 OOM 被杀。"
    cd "$ROOT/ws_moveit2"
    colcon build "${PAR[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
ros_source "$ROOT/ws_moveit2/install/setup.bash"

# ---- 3) pick_ik --------------------------------------------------------------
if step_enabled 3; then
    banner 3 "ws_ik_plugins: pick_ik"
    cd "$ROOT/ws_ik_plugins"
    colcon build "${PAR[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
ros_source "$ROOT/ws_ik_plugins/install/setup.bash"

# ---- 4) franka_vr ------------------------------------------------------------
if step_enabled 4; then
    banner 4 "ws_franka_vr: franka_vr"
    cd "$ROOT/ws_franka_vr"
    colcon build "${PAR[@]}" --packages-select franka_vr --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
ros_source "$ROOT/ws_franka_vr/install/setup.bash"

# ---- 5) oculus_reader (editable) ---------------------------------------------
if step_enabled 5; then
    banner 5 "pip -e oculus_reader"
    pip3 install -e "$ROOT/ws_franka_vr/src/franka_vr/oculus_reader"
fi

cat <<EOF

✅ 编译完成。每个新终端先执行：
     source $ROOT/setup_env.sh

   真机遥操(两个终端)：
     bash $ROOT/scripts/run_arm_stack.sh --real            # 终端1 机械臂+Servo+夹爪
     bash $ROOT/scripts/run_vr_bridge.sh                   # 终端2 Quest 桥接
EOF
