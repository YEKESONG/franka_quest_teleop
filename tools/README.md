# tools/ —— 周边工具

这些不是遥操通路本身，是围着它长出来的工具。**从新机器 clone 后不能直接跑**，
下面标出了每个工具需要改的硬编码路径 / 外部依赖。

---

## replay_wuji_vr_dataset.py —— 数据集回放

把 VR 遥操录下来的 LeRobot 数据集回放到真机（右 FR3 link8 + 右 Wuji 灵巧手），
执行语义与 VLA 部署一致：每帧读真机当前 link8 位姿，再应用 action 里的 6D delta；
手部 20D action 原样下发。

```bash
# 只检查数据，不连真机（安全，先跑这个）
python3 tools/replay_wuji_vr_dataset.py --dataset ~/lerobot_data/<数据集根目录> --check

# 真机回放
python3 tools/replay_wuji_vr_dataset.py --dataset ~/lerobot_data/<数据集根目录> --execute
```

它在**宿主机**上跑，自己去操作两个容器（会按需拉起、跑完收拾干净）：

| 容器 | 作用 | 来自 |
|---|---|---|
| `franka_dev` | 机械臂栈 | 本仓库 `docker_launch_files/` |
| `wuji-hand-teleop` | Wuji 灵巧手 | **另一个项目**，本仓库不含 |

脚本会先检查遥操是否还在跑（`start_franka_vr_dual.py` / `wujihand_controller`），
在跑就拒绝执行 —— 避免两路指令抢同一台机器人。

### 换机器要改的硬编码（脚本顶部常量，本仓库保持逐字节原样收录，没有改动）

| 常量 | 当前值 | 说明 |
|---|---|---|
| `VOLUME_HOST` | `/home/wang/libfranka-docker/docker_volume` | 宿主机上 `/docker_volume` 的实际路径。**改成本仓库根目录**即可 |
| `HAND_CONFIG` | `/home/wang/sy/wuji-hand-teleop/.../wujihand_ik.yaml` | 读右手 serial_number，属于 Wuji 项目 |
| `WUJI_SETUP` | `/home/wuji/ros2_ws/install/setup.bash` | `wuji-hand-teleop` 容器内路径 |

脚本运行时会把自己 `shutil.copy2` 进 `VOLUME_HOST`，所以它放在 `tools/` 下不影响使用。

> 没有 Wuji 灵巧手就用不了这个工具——它是"臂+手"的联合回放。
> 只回放手臂的话，`--check` 那条路径仍可用来看数据。

---

## 录制脚本在另一个项目里（本仓库不含）

`~/Desktop/gello_ros2/record_wuji_vr_lerobot.py` 是这条通路的**数据录制端**，
靠订阅桥接节点旁路发的 `/left/debug_target`、`/right/debug_target`（PoseStamped，
RELIABLE）+ TF `{side}_fr3_link0→{side}_fr3_hand` 拼 state/action。

没收进本仓库，是因为它同时依赖 Manus 手套、Wuji 20 DOF 灵巧手、三路 RealSense 和
lerobot——属于数采项目，搬过来也跑不起来。这里只记一句：
**`debug_target` 这个话题就是为它留的旁路**，改桥接节点时别把它删了。

---

## ../datasets/ —— 样例数据

| 文件 | 内容 |
|---|---|
| `wuji_vr_clean_desktop_10_right_ep0.npz` | 右臂单臂，493 帧：link8 位姿/目标、6D delta、手 20D |
| `wuji_vr_clean_desktop_6_ep0.npz` | 双臂，415 帧：左右 pos/quat + 左右手 20D |

都是从 LeRobot 数据集导出的缓存，纯轨迹数组（无图像），放在这里当**格式参考**——
新写解析/训练代码时对着它看字段维度，比翻文档快。
