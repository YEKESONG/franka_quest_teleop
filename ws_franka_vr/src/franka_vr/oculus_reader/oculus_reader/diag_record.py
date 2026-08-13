#!/usr/bin/env python3
"""
遥操抖动诊断录制器（独立节点，不影响遥操本身）。

同步订阅一条臂的：
  - 关节角/速度   /{side}/franka/joint_states
  - 下发给控制器的关节轨迹 /{side}/{side}_fr3_arm_controller/joint_trajectory
以 200Hz 缓冲最近 N 秒。检测到"抖动"（某关节速度的高频往复）时，
自动把该时刻前后各若干秒存成 CSV，供离线看波形。

也可手动触发：终端按 Enter 立即存一段。

用途：区分两种病因——
  A) 内环软腕谐振  -> 某几关节小幅高频同相振
  B) 奇异/IK跳解   -> 关节解跳变 / 某关节速度尖峰 / 特定姿态爆发

用法:
  python3 oculus_reader/diag_record.py --side left
"""
import argparse
import csv
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'diag_logs')


class DiagRecorder(Node):
    def __init__(self, side, win_sec, trig_vel_jerk):
        super().__init__(f'diag_recorder_{side}')
        self.side = side
        self.win = win_sec
        self.trig = trig_vel_jerk
        self.buf = deque(maxlen=int(win_sec * 2 * 220))   # 前后各 win 秒余量
        self.last_vel = None
        self.last_t = None
        self.last_save = 0.0
        os.makedirs(OUT_DIR, exist_ok=True)

        self.create_subscription(
            JointState, f'/{side}/franka/joint_states', self.on_js, 50)
        self.cmd_pos = [0.0] * 7
        self.create_subscription(
            JointTrajectory, f'/{side}/{side}_fr3_arm_controller/joint_trajectory',
            self.on_cmd, 50)
        self.tgt_quat = [0.0, 0.0, 0.0, 1.0]   # 目标姿态(诊断: 看目标本身是否在抖)
        self.create_subscription(
            PoseStamped, f'/{side}/debug_target', self.on_tgt, 50)
        self.get_logger().info(
            f'[{side}] 诊断录制中… 抖动自动存盘(阈值 jerk>{trig_vel_jerk}), '
            f'或按 Enter 手动存一段。CSV -> {OUT_DIR}')

    def on_cmd(self, msg: JointTrajectory):
        # trajectory 的关节顺序可能与 joint_states 不同 -> 按名字对齐到 j1..j7
        if msg.points:
            p = msg.points[-1].positions
            names = list(msg.joint_names)
            aligned = [0.0] * 7
            for k in range(1, 8):
                key = f'{self.side}_fr3_joint{k}'
                if key in names and len(p) > names.index(key):
                    aligned[k - 1] = p[names.index(key)]
            self.cmd_pos = aligned

    def on_tgt(self, msg: PoseStamped):
        o = msg.pose.orientation
        self.tgt_quat = [o.x, o.y, o.z, o.w]

    def on_js(self, msg: JointState):
        t = time.time()
        # franka/joint_states 只含 7 臂关节(无 finger)，按名字取前7
        name = list(msg.name)
        pos = list(msg.position)
        vel = list(msg.velocity) if msg.velocity else [0.0] * len(pos)
        # 按名字对齐到 j1..j7（与 cmd 一致），避免顺序错位
        qL, qdL = [], []
        for k in range(1, 8):
            key = f'{self.side}_fr3_joint{k}'
            if key not in name:
                return
            idx = name.index(key)
            qL.append(pos[idx])
            qdL.append(vel[idx] if idx < len(vel) else 0.0)
        q = np.array(qL)
        qd = np.array(qdL)

        # jerk 估计：关节加速度的绝对值（速度差分），高频往复会让它很大
        jerk = 0.0
        if self.last_vel is not None and self.last_t is not None:
            dt = max(1e-4, t - self.last_t)
            jerk = float(np.max(np.abs((qd - self.last_vel) / dt)))
        self.last_vel, self.last_t = qd, t

        row = [t] + q.tolist() + qd.tolist() + list(self.cmd_pos) + list(self.tgt_quat) + [jerk]
        self.buf.append(row)

        # 自动触发：jerk 超阈 + 距上次存盘>3s
        if jerk > self.trig and (t - self.last_save) > 3.0 and len(self.buf) > 100:
            self.save(f'auto_jerk{jerk:.0f}')
            self.last_save = t

    def save(self, tag):
        fn = os.path.join(OUT_DIR, f'{self.side}_{tag}_{int(time.time())}.csv')
        hdr = (['t'] + [f'q{i+1}' for i in range(7)] + [f'qd{i+1}' for i in range(7)]
               + [f'cmd{i+1}' for i in range(7)]
               + ['tqx', 'tqy', 'tqz', 'tqw'] + ['jerk'])
        with open(fn, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(list(self.buf))
        self.get_logger().info(f'[{self.side}] 已存 {len(self.buf)} 行 -> {fn}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', default='left', choices=['left', 'right'])
    ap.add_argument('--win', type=float, default=3.0, help='存盘窗口半宽[s]')
    ap.add_argument('--trig', type=float, default=40.0,
                    help='自动触发的 jerk 阈值[rad/s²]；太灵敏调大')
    a = ap.parse_args()

    rclpy.init()
    node = DiagRecorder(a.side, a.win, a.trig)

    # 后台线程：Enter 手动存盘
    import threading
    def kbd():
        for line in sys.stdin:
            node.save('manual')
    threading.Thread(target=kbd, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
