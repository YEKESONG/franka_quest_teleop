#!/bin/bash
# 容器入口：只做 source + 提示，【不】自动编译。
#
# 旧配方的 install_franka_ros2.sh 会在每次进容器时判断 install/ 存不存在、
# 不存在就 colcon build。那样做的问题：编译要几小时、失败了容器直接起不来、
# 而且它编的是 libfranka 0.13.2（对 FR3 服务器 v10 是错版本）。
# 现在编译交给显式的 scripts/build_all.sh，入口只保证环境干净可用。

set -e

source /opt/ros/humble/setup.bash

if [ -f /docker_volume/setup_env.sh ]; then
    # setup_env.sh 里对未编译的工作区做了存在性判断，首次进容器不会报错
    source /docker_volume/setup_env.sh
fi

if [ ! -d /docker_volume/ws_franka_vr/install ]; then
    cat <<'EOF'

  ┌──────────────────────────────────────────────────────────────┐
  │ 工作区还没编译。首次进容器请先跑：                            │
  │     bash /docker_volume/scripts/build_all.sh                  │
  │ （MoveIt2 那一步最重，双核约数小时，建议挂夜里）              │
  └──────────────────────────────────────────────────────────────┘

EOF
fi

exec "$@"
