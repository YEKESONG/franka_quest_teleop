# franka_quest_teleop

**English** · [简体中文](README.zh-CN.md)

Teleoperate a **Franka FR3** dual-arm setup with **Meta Quest 3S** controllers — a complete,
reproducible ROS 2 environment. Clone it, build it, run it.

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-required-2496ED?logo=docker&logoColor=white)
![Robot](https://img.shields.io/badge/Robot-Franka%20FR3-000000)
![Status](https://img.shields.io/badge/status-validated%20on%20hardware-success)

Validated on real dual-arm FR3 hardware: absolute pose following, zero engagement error,
rate-limited throughout, with a Smith predictor that removes the delay-induced wrist resonance.

```
Quest 3S controllers ──adb logcat──> bridge node ──set_target_pose──> outer-loop velocity control
   ──MoveIt Servo (TWIST)──> ros2_control ──libfranka──> FR3 arms
```

---

## Table of Contents

- [Why this repository exists](#why-this-repository-exists)
- [System architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Quick start](#quick-start)
- [Usage](#usage)
- [Repository layout](#repository-layout)
- [Version matrix](#version-matrix)
- [Two things that surprise everyone](#two-things-that-surprise-everyone)
- [Documentation](#documentation)
- [Troubleshooting](#troubleshooting)
- [Acknowledgements](#acknowledgements)
- [License](#license)

---

## Why this repository exists

The working teleoperation stack used to live in an unversioned Docker volume, its container
environment existed only as packages someone had `apt install`ed by hand, and the tuning
knowledge was spread across scattered notes. Dozens of rounds of on-hardware iteration were
one `rm -rf` away from being lost.

Everything needed to reproduce the pipeline now lives here:

- **Source** — the teleop package plus every third-party workspace, pinned and vendored
- **Environment** — a Dockerfile that reconstructs the exact container the robot ran on
- **Knowledge** — architecture, troubleshooting, and a full control-tuning retrospective

The repository root is mounted as `/docker_volume` inside the container, so paths in the docs,
scripts, and launch files map one-to-one onto paths in this repository.

## System architecture

```
 ┌─ Quest 3S headset ─────────────────────────────────────────────┐
 │  teleop-debug.apk — publishes controller poses + buttons       │
 │  to logcat under tag wE9ryARX                                  │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ① USB / adb logcat -T 0 -s wE9ryARX:I   (~70 Hz)
                ▼
 ┌─ bridge node: start_franka_vr_dual.py ─────────────────────────┐
 │  parse → ×SCALE → ROS frame conversion                         │
 │  TF: world → oculus_base (static) → oculus_{left,right}        │
 │  Re-anchor on engage: target ≡ current EE pose (bumpless)      │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ② service /{left,right}/set_target_pose  (70 Hz)
                │    side-channel topic /{left,right}/debug_target
                ▼
 ┌─ outer loop: demo_franka_vr_vel (one per arm) ─────────────────┐
 │  200 Hz: pose error → Cartesian twist                          │
 │  velocity feed-forward + PD + Smith predictor (T = 0.09 s)     │
 │  shortest-arc quaternion + 2·vec log map + rate/accel limits   │
 │  feedback closed on measured robot state, not an internal model│
 └──────────────┬─────────────────────────────────────────────────┘
                │ ③ MoveIt Servo (TWIST) → Jacobian pseudo-inverse
                ▼
 ┌─ ros2_control — JointTrajectoryController, effort interface ───┐
 │  per-joint PD (base joints stiff p=600, wrist soft p=50)       │
 └──────────────┬─────────────────────────────────────────────────┘
                │ ④ franka_hardware → libfranka 0.20.4 → 1 kHz loop
                ▼
              FR3 arms (left 172.16.0.2 / right 172.16.0.3)
                │ ⑤ joint states (30 Hz) ──► back to the outer loop
```

Per-stage verification commands live in [docs/architecture.md](docs/architecture.md).

## Prerequisites

| | Requirement |
|---|---|
| **Robot** | Franka FR3 (one or two arms), server **version 10** |
| **VR** | Meta Quest 3S with controllers, USB cable that carries data |
| **Host OS** | Ubuntu 22.04, Docker + Docker Compose v2 |
| **Network** | Host can reach the arms (default `172.16.0.2` / `172.16.0.3`) |
| **RAM** | 16 GB minimum — MoveIt 2 is built from source |
| **Disk** | ~25 GB for the image plus build artifacts |

## Quick start

```bash
git clone https://github.com/YEKESONG/franka_quest_teleop.git
cd franka_quest_teleop

# 1 — build the image and start a persistent container
xhost +local:docker                                              # required for RViz
docker compose -f docker_launch_files/docker-compose.yml build
docker compose -f docker_launch_files/docker-compose.yml up -d
docker exec -it franka_dev bash                                  # one exec per terminal you need

# 2 — build the four workspaces (first time takes hours; MoveIt 2 dominates)
bash /docker_volume/scripts/build_all.sh -j 2                    # keep -j low, it is RAM-bound

# 3 — prepare the headset (host side, once)
bash scripts/setup_quest_adb.sh                                  # wear the headset, tap "Always allow"
```

> **Container name.** The compose file names the container `franka_dev`. Override it if that
> name is taken: `FRANKA_CONTAINER_NAME=franka_dev2 docker compose ... up -d`.

## Usage

Two terminals, both `docker exec`'d into the same container.

```bash
# Terminal 1 — arms, MoveIt Servo, grippers, RViz
bash /docker_volume/scripts/run_arm_stack.sh --real              # both arms
bash /docker_volume/scripts/run_arm_stack.sh --real --arm right  # right arm only

# Terminal 2 — Quest bridge (must run in the foreground; it reads single keystrokes)
bash /docker_volume/scripts/run_vr_bridge.sh --arm right
```

Drop `--real` to run against mock hardware instead of the robot.

### Controls

| Input | Action |
|---|---|
| `Enter` | Toggle teleoperation. Engaging re-anchors: target ≡ current EE pose, so engagement error is always zero |
| `1` / `2` | Re-zero the left / right arm in place, without disengaging |
| Controller `Grip` | `> 0.6` close gripper, `< 0.4` open gripper |

### Common arguments

| Argument | Default | Meaning |
|---|---|---|
| `--arm` / `active_arm` | `both` | `left`, `right`, or `both` |
| `--real` | off | Use the real robot instead of mock hardware |
| `base_sep` | `1.05` | Distance in metres between the two robot bases — must match your setup |
| `control_tip` | `hand` | Servo control point: `hand` or `link8` |
| `--no-rviz`, `--no-gripper` | off | Skip RViz / gripper nodes |
| `SCALE` (script constant) | `2.0` | Controller displacement → arm displacement |

The scripts are thin wrappers. The equivalent raw command is:

```bash
ros2 launch franka_vr dual_franka_teleop.launch.py \
    active_arm:=right use_fake_hardware:=false robot_ip_right:=172.16.0.3
```

## Repository layout

| Path | Contents |
|---|---|
| `ws_franka_vr/src/franka_vr/` | **Core package** — outer-loop controller, dual-arm launch files, all configs, Quest bridge and APK |
| `ros2_ws/src/` | franka_ros2 **v2.3.0** + franka_description **1.6.1** |
| `ws_moveit2/src/` | MoveIt 2 **2.13.0** from source, including `moveit_servo` |
| `ws_ik_plugins/src/` | pick_ik **1.1.2** — must be built from source, see below |
| `docker_launch_files/` | Dockerfile, compose file, entrypoint |
| `scripts/` | Build, run, and headset-setup helpers |
| `docs/` | Architecture, troubleshooting, control-tuning retrospective |
| `tools/`, `datasets/` | Dataset replay utility and sample trajectories (adjacent tooling, not the pipeline) |
| `setup_env.sh` | Sources all four workspaces in dependency order |
| `udp_only.xml` | Forces FastDDS onto UDP — shared memory does not cross the container IPC namespace |

This is a **source snapshot**: `build/`, `install/`, `log/`, and diagnostic recordings are not
tracked and are regenerated by `colcon build`.

Deliberately **not** included: two obsolete libfranka source trees (0.9.2 and 0.13.3 — both wrong
for FR3 server v10), an old franka_ros2 v0.1.15, and a ROS 1 workspace. Vendoring them would only
invite someone to build the wrong version.

## Version matrix

| Component | Version | Why this one |
|---|---|---|
| libfranka | **0.20.4** (apt `ros-humble-libfranka`) | FR3 server v10 requires ≥ 0.20; 0.13.x fails with `Incompatible library version (server 10, library 7)` |
| franka_ros2 | **v2.3.0** | Matches libfranka 0.20.4 |
| franka_description | **1.6.1** | Pinned by franka_ros2 v2.3.0 |
| MoveIt 2 | **2.13.0**, from source | Humble's apt build is 2.5.9, whose old `servo.h` architecture lacks `servo.hpp` / `getNextJointState` / `TwistCommand` |
| pick_ik | **1.1.2**, from source | The apt build pulls in apt `moveit_core`, which collides with the source build (ABI conflict at load time) |

## Two things that surprise everyone

**pick_ik is installed but does not run during teleoperation.** The controller uses MoveIt Servo
in `TWIST` mode, where twist → joints is solved by the Jacobian pseudo-inverse, and no `move_group`
is launched. The weights in `kinematics_{left,right}.yaml` therefore have **no effect on teleop** —
do not tune them to fix teleop behaviour. Those files must still exist; the launch file reads them
unconditionally. Joint-limit handling comes from `joint_limit_margins` in `fr3_servo_*.yaml`.

**There is no calibration file.** Pressing `Enter` re-anchors against live TF so that the target
equals the current end-effector pose. Engagement error is structurally zero, and changing `SCALE`
never requires recalibration. An earlier design persisted calibration to JSON; a zero-byte file
once produced a target several metres away and a full-speed lunge into a reflex stop.

## Documentation

| Document | Read it when |
|---|---|
| [MIGRATION.md](MIGRATION.md) | Rebuilding on a new machine — versions, build order, run commands |
| [docs/architecture.md](docs/architecture.md) | You need topic/frame names or per-stage verification |
| [docs/troubleshooting.md](docs/troubleshooting.md) | Something fails to build, connect, or behave |
| [docs/control_tuning_retrospective.md](docs/control_tuning_retrospective.md) | **Before touching any control parameter** — dozens of rounds of on-hardware iteration, including what did not work |
| [tools/README.md](tools/README.md) | Using the dataset replay utility |

## Troubleshooting

The three most common failures:

| Symptom | Cause |
|---|---|
| Topics are listed but `echo` receives nothing | FastDDS shared memory cannot cross the container IPC namespace — use `udp_only.xml` |
| The arm twitches on a **~5 second** cycle | Another process owns the adb server and its polling interrupts the logcat stream |
| `Incompatible library version` on connect | libfranka is not 0.20.4 |

Full list in [docs/troubleshooting.md](docs/troubleshooting.md).

## Acknowledgements

| Component | Origin |
|---|---|
| Docker skeleton | Forked from [`Fjakob/libfranka-docker`](https://github.com/Fjakob/libfranka-docker) via `ZorAttC/libfranka-docker`, rewritten to match the validated environment |
| franka_vr package, Quest APK, oculus_reader | The original `franka_vr` project; this repository adds dual-arm support and rewrites the control loop |
| libfranka, franka_ros2, franka_description | [Franka Robotics](https://github.com/frankaemika) |
| MoveIt 2, moveit_servo, pick_ik | [MoveIt](https://github.com/moveit) |

## License

No repository-wide licence has been declared. Vendored third-party components keep their own
licences — see the `LICENSE` files under `ros2_ws/src/`, `ws_moveit2/src/`, `ws_ik_plugins/src/`,
and `ws_franka_vr/src/franka_vr/oculus_reader/`. Respect them before redistributing.
