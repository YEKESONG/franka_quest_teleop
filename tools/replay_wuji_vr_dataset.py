#!/usr/bin/env python3
"""回放现有 Wuji VR LeRobot 数据集：右 FR3(link8) + 右 Wuji 手。

数据格式保持不变：
  observation.state = link8 位姿 7 + Wuji 实测 20
  action            = link8 目标增量 6 + Wuji 绝对关节目标 20

执行语义与 VLA 部署一致：每帧读取真机当前 link8 位姿，再应用 action 的 6D delta；
Wuji 的 20D action 原样发送。宿主机入口会自动准备两个现有 Docker 容器。
"""

import argparse
import glob
import json
import math
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


FRANKA_CONTAINER = "franka_dev"
WUJI_CONTAINER = "wuji-hand-teleop"
VOLUME_HOST = Path("/home/wang/libfranka-docker/docker_volume")
CONTAINER_SCRIPT = "/docker_volume/replay_wuji_vr_dataset.py"
FRANKA_SETUP = "/docker_volume/setup_env.sh"
WUJI_SETUP = "/home/wuji/ros2_ws/install/setup.bash"
FRANKA_PID = "/tmp/wuji_replay_franka.pid"
WUJI_PID = "/tmp/wuji_replay_hand.pid"
FRANKA_LOG = "/tmp/wuji_replay_franka.log"
WUJI_LOG = "/tmp/wuji_replay_hand.log"
HAND_CONFIG = Path(
    "/home/wang/sy/wuji-hand-teleop/src/output_devices/"
    "wujihand_output/config/wujihand_ik.yaml"
)
HAND_DOF = 20


def quat_normalize(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise RuntimeError("遇到零四元数")
    return q / n


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def axis_angle_to_quat(v):
    v = np.asarray(v, dtype=np.float64)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = v / angle
    s = math.sin(angle / 2.0)
    return np.r_[axis * s, math.cos(angle / 2.0)]


def quat_slerp(q0, q1, t):
    q0, q1 = quat_normalize(q0), quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))
    theta = math.acos(dot)
    return (
        math.sin((1.0 - t) * theta) / math.sin(theta) * q0
        + math.sin(t * theta) / math.sin(theta) * q1
    )


def quat_angle(q0, q1):
    return 2.0 * math.acos(float(np.clip(abs(np.dot(quat_normalize(q0), quat_normalize(q1))), 0, 1)))


def wait_period(start, period):
    remaining = period - (time.perf_counter() - start)
    if remaining > 0:
        time.sleep(remaining)


def _indices(names, expected, label):
    missing = [name for name in expected if name not in names]
    if missing:
        raise RuntimeError(f"{label}.names 缺少 {missing}")
    return [names.index(name) for name in expected]


def export_cache(dataset, episode, cache_path):
    """只生成容器传输缓存；不修改原数据集。"""
    import pandas as pd

    root = Path(dataset).expanduser().resolve()
    info_path = root / "meta/info.json"
    if not info_path.exists():
        raise RuntimeError(f"不是有效数据集，缺少 {info_path}")
    info = json.loads(info_path.read_text())
    features = info["features"]
    state_names = list(features["observation.state"]["names"])
    action_names = list(features["action"]["names"])
    state_dim = int(features["observation.state"]["shape"][0])
    action_dim = int(features["action"]["shape"][0])
    if (state_dim, action_dim) != (27, 26):
        raise RuntimeError(f"当前回放只接受右臂数据 state/action=27/26，实际 {state_dim}/{action_dim}")

    eef_state = _indices(
        state_names,
        [f"right_eef_{x}" for x in ("x", "y", "z", "qx", "qy", "qz", "qw")],
        "observation.state",
    )
    eef_action = _indices(
        action_names,
        [f"right_eef_{x}" for x in ("dx", "dy", "dz", "drx", "dry", "drz")],
        "action",
    )
    hand_action = _indices(
        action_names,
        [f"right_hand_cmd_finger{f}_joint{j}" for f in range(1, 6) for j in range(1, 5)],
        "action",
    )

    files = sorted(glob.glob(str(root / "data/chunk-*/*.parquet")))
    if not files:
        raise RuntimeError(f"{root}/data 下没有 parquet")
    df = pd.concat([pd.read_parquet(path) for path in files], ignore_index=True)
    rows = df[df["episode_index"] == episode].sort_values("frame_index")
    if rows.empty:
        available = sorted(df["episode_index"].unique().tolist())
        raise RuntimeError(f"没有 episode {episode}，可选 {available}")
    state = np.stack(rows["observation.state"]).astype(np.float64)
    action = np.stack(rows["action"]).astype(np.float64)
    state_pos = state[:, eef_state[:3]]
    state_quat = state[:, eef_state[3:7]]
    delta = action[:, eef_action]
    hand = action[:, hand_action]
    arrays = (state_pos, state_quat, delta, hand)
    if not all(np.all(np.isfinite(a)) for a in arrays):
        raise RuntimeError("episode 含 NaN/Inf")
    max_delta = float(np.linalg.norm(delta[:, :3], axis=1).max())
    if max_delta > 0.15:
        raise RuntimeError(f"机械臂 action delta 最大 {max_delta*1000:.1f}mm，超过安全阈值150mm")

    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        cache_path,
        fps=np.int64(info["fps"]),
        episode=np.int64(episode),
        state_pos=state_pos,
        state_quat=state_quat,
        delta=delta,
        hand=hand,
    )
    print(
        f"加载数据集 episode {episode}: 右臂+右手，{len(rows)}帧 @ {info['fps']}Hz，"
        f"state/action=27/26，delta最大 {max_delta*1000:.1f}mm"
    )


def load_cache(path, max_frames=0):
    with np.load(path, allow_pickle=False) as raw:
        data = {key: raw[key] for key in raw.files}
    required = {"fps", "state_pos", "state_quat", "delta", "hand"}
    if not required.issubset(data):
        raise RuntimeError(f"内部缓存缺少 {sorted(required - set(data))}")
    n = len(data["delta"])
    if max_frames > 0:
        n = min(n, max_frames)
    shapes = {
        "state_pos": (len(data["state_pos"]), 3),
        "state_quat": (len(data["state_quat"]), 4),
        "delta": (len(data["delta"]), 6),
        "hand": (len(data["hand"]), 20),
    }
    for key, expected in shapes.items():
        if data[key].shape != expected:
            raise RuntimeError(f"{key} shape={data[key].shape}，期望 {expected}")
        data[key] = data[key][:n]
    data["fps"] = float(data["fps"])
    return data


class Replayer:
    def __init__(self, data, execute):
        import rclpy
        import tf2_ros
        from franka_vr.srv import SetTargetPose
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.rclpy = rclpy
        self.SetTargetPose = SetTargetPose
        self.JointState = JointState
        self.data = data
        self.execute = execute
        self.hand_state = None
        rclpy.init()
        self.node = Node("wuji_vr_replayer")
        self.service = self.node.create_client(SetTargetPose, "/right/set_target_pose")
        self.hand_pub = self.node.create_publisher(
            JointState, "/right_hand/joint_commands", qos_profile_sensor_data
        )
        self.hand_sub = self.node.create_subscription(
            JointState, "/right_hand/joint_states", self._hand_cb, qos_profile_sensor_data
        )
        self.tf_node = Node("wuji_vr_replayer_tf")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.tf_executor = SingleThreadedExecutor()
        self.tf_executor.add_node(self.tf_node)
        import threading

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.tf_thread = threading.Thread(target=self.tf_executor.spin, daemon=True)
        self.spin_thread.start()
        self.tf_thread.start()

    def _hand_cb(self, msg):
        if len(msg.position) >= HAND_DOF:
            self.hand_state = np.asarray(msg.position[:HAND_DOF], dtype=np.float64)

    def measured(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "right_fr3_link0", "right_fr3_link8", self.rclpy.time.Time()
            )
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def wait_ready(self, timeout=75.0):
        deadline = time.monotonic() + timeout
        next_notice = 0.0
        while True:
            missing = []
            if not self.service.service_is_ready():
                missing.append("/right/set_target_pose")
            if self.measured() is None:
                missing.append("TF link0->link8")
            if self.hand_pub.get_subscription_count() == 0:
                missing.append("/right_hand/joint_commands订阅者")
            if self.hand_state is None:
                missing.append("/right_hand/joint_states")
            if not missing:
                print("[replay] 右臂 link8 和右 Wuji 手已就绪。")
                return
            now = time.monotonic()
            if now >= next_notice:
                print("[replay] 等待 " + ", ".join(missing) + "…")
                next_notice = now + 2.0
            if now > deadline:
                raise RuntimeError("等待超时：" + ", ".join(missing))
            time.sleep(0.1)

    def send_pose(self, pos, quat):
        if not self.execute:
            return
        from geometry_msgs.msg import Pose

        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = map(float, pos)
        q = quat_normalize(quat)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
        request = self.SetTargetPose.Request()
        request.target_pose = msg
        self.service.call_async(request)

    def send_hand(self, values):
        if not self.execute:
            return
        msg = self.JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.position = np.asarray(values, dtype=float).tolist()
        self.hand_pub.publish(msg)

    def goto_start(self):
        current = self.measured()
        if current is None:
            raise RuntimeError("起始对齐前 link8 TF 丢失")
        p0, q0 = current
        p1, q1 = self.data["state_pos"][0], self.data["state_quat"][0]
        distance = float(np.linalg.norm(p1 - p0))
        angle = quat_angle(q0, q1)
        steps = max(1, math.ceil(max(distance / 0.02, angle / 0.25) * 30.0))
        print(f"[replay] 对齐第一帧 state：距离 {distance*1000:.1f}mm，姿态 {math.degrees(angle):.1f}°")
        before_hand = self.hand_state.copy()
        for k in range(1, steps + 1):
            start = time.perf_counter()
            t = k / steps
            self.send_pose(p0 + t * (p1 - p0), quat_slerp(q0, q1, t))
            self.send_hand(self.data["hand"][0])
            wait_period(start, 1.0 / 30.0)

        # 与原版 replay 的 goto_start/join_motion 一样等待真正到位；只保留60秒故障上限。
        # 真机本次在15秒边界才报告到位，原来的15秒会产生“刚到就被判失败”的竞争。
        deadline = time.monotonic() + 60.0
        next_notice = 0.0
        while time.monotonic() < deadline:
            start = time.perf_counter()
            current = self.measured()
            if current is not None:
                pos_error = float(np.linalg.norm(p1 - current[0]))
                rot_error = quat_angle(q1, current[1])
                if pos_error < 0.01 and rot_error < 0.10:
                    print(
                        f"[replay] 起始对齐完成：位置残差 {pos_error*1000:.1f}mm，"
                        f"姿态残差 {math.degrees(rot_error):.1f}°"
                    )
                    break
                now = time.monotonic()
                if now >= next_notice:
                    print(
                        f"[replay] 等待起始到位：位置 {pos_error*1000:.1f}mm，"
                        f"姿态 {math.degrees(rot_error):.1f}°"
                    )
                    next_notice = now + 2.0
            self.send_pose(p1, q1)
            self.send_hand(self.data["hand"][0])
            wait_period(start, 1.0 / 30.0)
        else:
            raise RuntimeError(
                f"起始对齐失败：位置残差 {pos_error*1000:.1f}mm，"
                f"姿态残差 {math.degrees(rot_error):.1f}°；不执行轨迹"
            )

        requested = float(np.linalg.norm(self.data["hand"][0] - before_hand))
        moved = float(np.linalg.norm(self.hand_state - before_hand))
        print(f"[replay] Wuji 首帧：请求变化 {requested:.3f}rad，实测变化 {moved:.3f}rad")
        if requested > 0.05 and moved < 0.01:
            raise RuntimeError("Wuji 命令已发布但手没有响应；不执行轨迹")

    def run(self):
        fps = self.data["fps"]
        period = 1.0 / fps
        count = len(self.data["delta"])
        self.wait_ready()
        self.goto_start()
        print(f"[replay] 开始按 {fps:g}Hz 执行 {count} 帧 action。")
        checkpoint_state = self.hand_state.copy()
        checkpoint_target = self.data["hand"][0].copy()
        for i in range(count):
            start = time.perf_counter()
            current = self.measured()
            if current is None:
                raise RuntimeError(f"第{i}帧 link8 TF 丢失")
            delta = self.data["delta"][i]
            target_pos = current[0] + delta[:3]
            target_quat = quat_mul(axis_angle_to_quat(delta[3:]), current[1])
            self.send_pose(target_pos, target_quat)
            self.send_hand(self.data["hand"][i])

            if i and i % max(1, int(fps)) == 0:
                requested = float(np.linalg.norm(self.data["hand"][i] - checkpoint_target))
                moved = float(np.linalg.norm(self.hand_state - checkpoint_state))
                if requested > 0.05 and moved < 0.01:
                    raise RuntimeError(
                        f"第{i}帧：Wuji 过去1秒命令变化 {requested:.3f}rad，"
                        f"实测只变化 {moved:.3f}rad"
                    )
                checkpoint_target = self.data["hand"][i].copy()
                checkpoint_state = self.hand_state.copy()
                print(
                    f"  [{i}/{count}] delta={np.linalg.norm(delta[:3])*1000:.1f}mm，"
                    f"Wuji实测变化={moved:.3f}rad"
                )
            wait_period(start, period)

        # 把 Servo 目标归零到当前 link8，避免回放结束后保留最后一个误差目标。
        for _ in range(10):
            current = self.measured()
            if current is not None:
                self.send_pose(*current)
            time.sleep(0.02)
        print("[replay] 回放结束。")

    def shutdown(self):
        try:
            current = self.measured()
            if current is not None:
                self.send_pose(*current)
        except Exception:
            pass
        for executor in (self.executor, self.tf_executor):
            try:
                executor.shutdown()
            except Exception:
                pass
        self.spin_thread.join(timeout=1)
        self.tf_thread.join(timeout=1)
        for node in (self.tf_node, self.node):
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass


def ros_env():
    return (
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 "
        "ROS_LOCALHOST_ONLY=0 ROS2CLI_NO_DAEMON=1 "
        "FASTDDS_BUILTIN_TRANSPORTS=UDPv4; unset CYCLONEDDS_URI"
    )


def docker_bash(container, command, capture=False):
    return subprocess.run(
        ["docker", "exec", container, "bash", "-lc", command],
        text=True,
        stdout=subprocess.PIPE if capture else None,
        stderr=subprocess.STDOUT if capture else None,
    )


def ensure_container(name):
    check = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.Running}}", name],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )
    if check.returncode == 0 and check.stdout.strip() == "true":
        return False
    result = subprocess.run(
        ["docker", "start", name], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    if result.returncode:
        raise RuntimeError(f"启动容器 {name} 失败：{result.stdout}")
    print(f"[自动准备] 已启动容器 {name}")
    return True


def start_background(container, setup, launch, pidfile, logfile):
    inner = f"source {shlex.quote(setup)}; {ros_env()}; exec {launch}"
    command = (
        f"rm -f {pidfile}; nohup setsid bash -lc {shlex.quote(inner)} "
        f"> {logfile} 2>&1 < /dev/null & echo $! > {pidfile}"
    )
    result = docker_bash(container, command, capture=True)
    if result.returncode:
        raise RuntimeError(f"在 {container} 启动失败：{result.stdout}")


def stop_background(container, pidfile):
    command = (
        f"if test -s {pidfile}; then p=$(cat {pidfile}); "
        "kill -TERM -- -$p 2>/dev/null || kill -TERM $p 2>/dev/null || true; sleep 2; "
        "kill -KILL -- -$p 2>/dev/null || true; "
        f"rm -f {pidfile}; fi"
    )
    docker_bash(container, command, capture=True)


def right_serial():
    text = HAND_CONFIG.read_text()
    block = re.search(r"(?ms)^right_hand:\s*\n(?P<body>(?:^[ \t]+.*\n?)*)", text)
    match = re.search(r"serial_number:\s*[\"']?([^\"'\s#]+)", block.group("body") if block else "")
    if not match:
        raise RuntimeError(f"{HAND_CONFIG} 没有右手 serial_number")
    return match.group(1)


def process_exists(container, pattern):
    return docker_bash(container, f"pgrep -f {shlex.quote(pattern)} >/dev/null", capture=True).returncode == 0


def host_run(args):
    dataset = Path(args.dataset).expanduser().resolve()
    cache = VOLUME_HOST / f".{dataset.name}_ep{args.episode}_replay.npz"
    export_cache(dataset, args.episode, cache)
    if args.check or not args.execute:
        data = load_cache(cache, args.max_frames)
        print(
            f"[检查完成] {len(data['delta'])}帧；不会改数据集。"
            "真机回放请加 --execute。"
        )
        return

    started_containers = []
    started_arm = False
    started_hand = False
    try:
        for container in (FRANKA_CONTAINER, WUJI_CONTAINER):
            if ensure_container(container):
                started_containers.append(container)
        shutil.copy2(__file__, VOLUME_HOST / Path(__file__).name)

        if process_exists(FRANKA_CONTAINER, "[s]tart_franka_vr_dual.py"):
            raise RuntimeError("检测到 VR 遥操仍在运行；先停止遥操，避免抢控制")
        if process_exists(WUJI_CONTAINER, "[w]ujihand_controller"):
            raise RuntimeError("检测到 Wuji 遥操控制器仍在运行；先停止遥操，避免抢手指命令")

        arm_running = process_exists(FRANKA_CONTAINER, "[d]emo_franka_vr_vel")
        if arm_running:
            raise RuntimeError("已有机械臂 Servo 控制栈在运行；请先停止，确保 control_tip=link8")
        print("[自动准备] 启动右臂 Servo（control_tip=right_fr3_link8）…")
        start_background(
            FRANKA_CONTAINER,
            FRANKA_SETUP,
            "ros2 launch franka_vr dual_franka_teleop.launch.py "
            "active_arm:=right use_fake_hardware:=false load_gripper:=false "
            "control_tip:=link8 use_rviz:=false",
            FRANKA_PID,
            FRANKA_LOG,
        )
        started_arm = True

        if not process_exists(WUJI_CONTAINER, "[w]ujihand_driver_node"):
            serial = right_serial()
            print(f"[自动准备] 启动右 Wuji 手驱动（{serial}）…")
            start_background(
                WUJI_CONTAINER,
                WUJI_SETUP,
                "ros2 launch wujihand_bringup wujihand.launch.py "
                f"hand_name:=right_hand serial_number:={serial}",
                WUJI_PID,
                WUJI_LOG,
            )
            started_hand = True
        else:
            print("[自动准备] 复用已运行的右 Wuji 驱动。")

        inner_args = [
            "python3", "-u", CONTAINER_SCRIPT,
            "--cache", f"/docker_volume/{cache.name}",
            "--execute",
            "--max-frames", str(args.max_frames),
        ]
        command = (
            f"source {FRANKA_SETUP}; {ros_env()}; exec "
            + " ".join(shlex.quote(v) for v in inner_args)
        )
        try:
            result = subprocess.run(
                ["docker", "exec", "-i", FRANKA_CONTAINER, "bash", "-lc", command]
            )
        except KeyboardInterrupt:
            docker_bash(
                FRANKA_CONTAINER,
                "pkill -TERM -f '[r]eplay_wuji_vr_dataset.py --cache' || true",
                capture=True,
            )
            raise
        if result.returncode:
            arm_log = docker_bash(FRANKA_CONTAINER, f"tail -n 40 {FRANKA_LOG}", True).stdout
            hand_log = docker_bash(WUJI_CONTAINER, f"tail -n 40 {WUJI_LOG}", True).stdout
            raise RuntimeError(
                f"回放退出（{result.returncode}）\nFranka日志：\n{arm_log}\nWuji日志：\n{hand_log}"
            )
    finally:
        if started_hand:
            print("[自动收尾] 停止本次启动的 Wuji 驱动。")
            stop_background(WUJI_CONTAINER, WUJI_PID)
        if started_arm:
            print("[自动收尾] 停止本次启动的右臂 Servo。")
            stop_background(FRANKA_CONTAINER, FRANKA_PID)
        for container in reversed(started_containers):
            subprocess.run(
                ["docker", "stop", "--time", "5", container],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass
    parser = argparse.ArgumentParser(
        description="回放右FR3(link8)+右Wuji的27/26维数据集",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--dataset", default="", help="LeRobot 数据集根目录")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--execute", action="store_true", help="真机执行")
    parser.add_argument("--check", action="store_true", help="只检查数据，不连接真机")
    parser.add_argument("--max-frames", type=int, default=0, help="只执行前N帧，0=全部")
    parser.add_argument("--cache", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.execute and args.check:
        parser.error("--execute 与 --check 不能同时使用")
    if args.cache:
        data = load_cache(args.cache, args.max_frames)
        replay = Replayer(data, args.execute)
        try:
            replay.run()
        finally:
            replay.shutdown()
        return
    if not args.dataset:
        parser.error("请给 --dataset")
    host_run(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[replay] 已停止。")
        raise SystemExit(130)
    except RuntimeError as exc:
        print(f"\n[错误] {exc}", file=sys.stderr)
        raise SystemExit(1)
