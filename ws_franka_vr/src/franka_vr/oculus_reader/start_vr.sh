#!/bin/bash

# 获取脚本所在的目录并切换到那里
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
cd "$SCRIPT_DIR" || exit 1

# ACTIVE_ARM 可选 left/right/both；默认 both 保持旧行为。
ACTIVE_ARM="${ACTIVE_ARM:-both}"
python3 oculus_reader/start_franka_vr_dual.py --ros-args -p active_arm:="$ACTIVE_ARM"
