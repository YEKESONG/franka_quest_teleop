#!/bin/bash
# Quest 3S 连线自检 + 装遥操 APK。跑桥接前先跑这个。
#
#   bash scripts/setup_quest_adb.sh              # 自检 + 缺 APK 就装
#   bash scripts/setup_quest_adb.sh --reinstall  # 强制重装 APK
#
# 桥接节点(reader.py)是这么读手柄的：adb → 启动头显里的 APK →
# `logcat -T 0 -s wE9ryARX:I` 长连接流，逐行解析位姿+按键。
# 所以只要 adb 这条链路不干净，遥操就会"卡顿/抽搐/根本没数据"。
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APK="$ROOT/ws_franka_vr/src/franka_vr/oculus_reader/oculus_reader/APK/teleop-debug.apk"
PKG="com.rail.oculus.teleop"
REINSTALL=false
[[ "${1:-}" == "--reinstall" ]] && REINSTALL=true

command -v adb >/dev/null || { echo "✗ 没有 adb。容器里应该自带；宿主机上装： sudo apt install adb"; exit 1; }

echo "== 1. adb server =="
adb start-server >/dev/null 2>&1
echo "   OK (127.0.0.1:5037)"

echo "== 2. 设备 =="
DEV="$(adb devices | awk 'NR>1 && $2=="device" {print $1}')"
if [[ -z "$DEV" ]]; then
    adb devices
    cat <<'EOF'
✗ 没有可用设备。逐条排查：
   - USB 线插好，头显【戴上】后会弹"允许 USB 调试" → 选【一律允许】
   - 显示 "unauthorized"：头显里没点允许，或换过 adb server 属主(密钥变了)，重新点一次
   - 显示 "no permissions"：宿主机缺 udev 规则，或该在容器里跑(compose 已挂 /dev + privileged)
EOF
    exit 1
fi
echo "   设备: $DEV"

echo "== 3. adb 独占检查 =="
# 全机只有一个 adb server(127.0.0.1:5037)，谁先起谁拥有。
# 另一套遥操(wuji-hand-teleop 容器里的 adb_watchdog.sh)每 5 秒 `adb devices` 轮询，
# 会打断本项目的 logcat 长连接 → 手柄位姿成批到达+跳变 → 机械臂呈【~5 秒周期】抽搐。
# 这是 2026-07-29 实锤过的抽搐根因，不是玄学。
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q wuji-hand-teleop; then
    cat <<'EOF'
⚠ 检测到 wuji-hand-teleop 容器在跑 —— 它会抢同一个 adb server 和同一台 Quest。
  症状：遥操呈 ~5 秒周期抽搐。跑 VR 遥操前先清干净：
      docker exec wuji-hand-teleop pkill -9 -f adb_watchdog.sh
      docker exec wuji-hand-teleop pkill -9 -f RoboticsServiceProcess
      adb reverse --remove-all; adb kill-server
  （换 adb server 属主后头显会重新弹授权，记得点"一律允许"）
EOF
fi
REV="$(adb reverse --list 2>/dev/null)"
[[ -n "$REV" ]] && { echo "⚠ 这台设备上有别人建的 reverse 映射："; echo "$REV"; }

echo "== 4. APK =="
[[ -f "$APK" ]] || { echo "✗ 找不到 APK: $APK"; exit 1; }
if $REINSTALL; then
    echo "   强制重装 ..."
    adb install -r -t "$APK"
elif adb shell pm list packages 2>/dev/null | tr -d '\r' | grep -q "^package:$PKG$"; then
    echo "   已安装 $PKG"
else
    echo "   未安装，正在安装 ..."
    adb install -t "$APK"
fi

echo "== 5. 拉起 APK =="
adb shell am start -n "$PKG/$PKG.MainActivity" \
    -a android.intent.action.MAIN -c android.intent.category.LAUNCHER >/dev/null 2>&1
echo "   已启动（桥接节点起来时也会自己再拉一次）"

cat <<EOF

✅ 自检通过。接着：
     bash $ROOT/scripts/run_vr_bridge.sh
   头显要【戴在头上】且手柄在相机视野内，否则位姿会漂/丢。
EOF
