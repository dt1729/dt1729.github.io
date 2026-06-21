---
layout: post
title: "manipulation_vision: Building a Full Manipulation Stack on the Piper Arm"
date: 2026-06-06 10:00:00-0400
description: A detailed walkthrough of building simulation, tactile sensing, gain tuning, motion profiling, and GPU-scale training infrastructure for the PiperOmron robot in robocasa kitchen environments.
tags: Robotics MuJoCo Simulation Manipulation Tactile-Sensing RL Docker GCP
categories: Research
---

This post documents the full arc of [`manipulation_vision`](https://github.com/dt1729/manipulation_vision) — a toolkit I built to develop and deploy manipulation policies on a **Piper arm mounted on an Omron mobile base** (PiperOmron). The stack runs end-to-end in MuJoCo simulation through robocasa kitchen environments, with a clear path toward GPU-scale RL and real hardware deployment.

---

### The Hardware: PiperOmron

The physical platform is a **Piper 6-DOF arm** (joints 1–6, position-controlled via CAN bus) mounted on an **Omron mobile base** (forward/side/yaw + torso height). The gripper has two finger-pad links (joint7, joint8).

For simulation, the full assembly — arm, mobile base, and gripper — is modeled as a single MuJoCo composite robot and integrated into robosuite as `PiperOmron`, callable from any robocasa kitchen task:

```bash
python -m robocasa.demos.demo_kitchen_states \
  --task PnPCounterToCab --robot PiperOmron
```

The MJCF model lives inside the robosuite fork (`includes/robosuite`) so it's available to all robocasa task environments without any path hacking.

---

### The Oscillation Problem

The first real obstacle was the position controller. Out of the box, the PiperOmron sim produced massive oscillations that made every trajectory useless. Three root causes, fixed in sequence:

**1. Timestep + integrator.** robocasa defaults to `dt=0.002s` with the Euler integrator. At high `kp`, Euler discretisation amplifies stiffness and produces spurious oscillations. Switching to `dt=0.001s` + `IMPLICITFAST` (MuJoCo's implicit velocity integrator) killed the oscillations immediately — IMPLICITFAST adds implicit damping that makes stiff joints unconditionally stable.

**2. Free base joints.** The Omron base has three zero-damping DOFs (forward/side/yaw). Reaction forces from arm motion were feeding back into the base and exciting a resonance loop. Locking them with `dof_damping=1e6` broke the loop.

**3. Step input.** Commanding a large position change as a step creates a huge instantaneous error that saturates the actuator. Adding a `settle_time=1.0s` hold period at `init_qpos` before the first command lets the physics transients decay before the arm moves.

---

### Gain Tuning GUI

Once the simulation was stable, I needed a way to tune `kp` and `kv` per joint without running the full kitchen environment. The full robocasa scene has complex coupled dynamics (arm + mobile base + objects + scene geometry) that make it hard to isolate individual joint behavior.

The solution is `includes/gain_tuner/tune_gains.py` — a standalone tkinter GUI that loads the bare arm XML, runs step-response simulations, and plots reference vs actual position. It uses **identical simulation settings** to the full env (timestep, integrator, iterations, `init_qpos`) so tuned gains transfer directly.

```
┌─────────────────────────────────┬────────────────────────────────┐
│  kp / kv sliders (per joint)    │  Target setpoint sliders       │
│                                 │                                │
│  joint1  ──[====]──  8000       │  joint1  ──[==]──  0.00 rad   │
│  joint2  ──[====]──  6000       │  joint2  ──[==]──  1.57 rad   │
│  ...                            │  ...                           │
│                                 │  Duration: 5.0 s              │
│                                 │  [Run Simulation] [Export XML] │
└─────────────────────────────────┴────────────────────────────────┘
```

The **Export XML** button writes tuned `kp`/`kv` back into the robot XML so gains survive across sessions. Once tuned, retuning is not required unless the robot model changes — the same gains work in the full kitchen environment and will transfer exactly to the Newton/MJWarp GPU stack (same MuJoCo physics engine).

```bash
cd includes/gain_tuner
python tune_gains.py
```

---

### Trapezoidal Motion Profile

Raw step commands (jump directly to target) cause large instantaneous position errors that saturate actuators and excite structural oscillations, even with well-tuned gains. All motion in the repo uses a **time-synchronised trapezoidal velocity profile**:

```
Phase 1 — Accelerate  : q(t) = q₀ + ½ a t²
Phase 2 — Cruise      : q(t) = q_acc + v_max (t - t_acc)
Phase 3 — Decelerate  : mirror of Phase 1
```

All joints are **time-synchronised** — the slowest joint (longest travel) sets the total duration, and every other joint scales its cruise velocity down to match. This keeps the end-effector on a straight joint-space path. A triangular profile is used automatically when the distance is too short to reach `max_vel`.

| Parameter | Default | Description |
|---|---|---|
| `max_vel` | 0.4 rad/s | Peak joint velocity during cruise |
| `max_acc` | 0.3 rad/s² | Acceleration / deceleration ramp rate |
| `settle_time` | 1.0 s | Hold time at start before first move |
| `atol` | 0.01 rad | Convergence tolerance |

After adding the trapezoidal profile, tracking errors dropped to near zero and the oscillations were gone completely.

---

### Config-Driven Behavior

All runtime parameters are centralised in `scripts/config/piper_behavior.json`. CLI flags act as per-run overrides. This makes it easy to swap tasks, tune motion parameters, or change waypoints without touching any code:

```json
{
    "sim":    { "timestep": 0.001, "integrator": "IMPLICITFAST", "iterations": 20 },
    "motion": { "settle_time": 1.0, "max_vel": 0.4, "max_acc": 0.3, "atol": 0.01 },
    "robot":  { "env_name": "PickPlaceCounterToCabinet",
                "init_qpos": [0.0, 1.57, -1.57, 0.0, 1.22, 0.0, 0.0, 0.0] },
    "waypoints": [
        [0.0,  0.5, -1.0,  0.0,  0.8,  0.0],
        [0.5,  1.0, -1.5,  0.3,  0.5, -0.5],
        [0.0,  1.57, -1.57, 0.0, 1.22,  0.0]
    ]
}
```

```bash
# Execute config waypoints
python scripts/send_joint_cmd.py

# Override a parameter without editing the file
python scripts/send_joint_cmd.py --max-vel 0.2 --settle 2.0
```

---

### Tactile Sensing

The gripper finger pads (link7 and link8) are instrumented with a **16×16 grid of independent MuJoCo `<touch>` sensors** per finger — 512 sensors total.

**Why not `touch_grid`?** MuJoCo's `touch_grid` plugin fires one cell per contact point, and MuJoCo's CCD contact model generates at most one contact point per geom pair. With a rigid object like an orange (16 convex collision geoms) you get at most 1–2 active cells regardless of contact force. Useless for spatial sensing.

**The working approach:** Inspired by the Shadow Hand in Gymnasium Robotics (`HandManipulateEgg-v1`), which uses 92 independent `<touch>` sensor sites. Each site is a **sphere** that fires independently when any contact point falls within its volume. Placing many overlapping sites on a grid achieves realistic spatial activation without fighting the contact model.

- **512 sensors total**: 256 per finger (16×16 grid)
- **Site pitch**: 1.875 mm (X) × 2.8 mm (Y) across the 30×45 mm contact face
- **Site radius**: 4 mm — enough overlap to catch contacts at each cell location
- **Contact filtering**: orange geoms use `contype=2/conaffinity=2`; finger pads use `contype=3/conaffinity=3` — the orange only collides with the two finger pads, not the wrist or arm body

| Metric | Value |
|---|---|
| Peak cells active (one pad) | 18 / 256 |
| Peak force per cell | ~1.1 N |
| Simulation speed (no render) | 3.7× realtime |
| Sensor read time | 0.15s / 15s sim (4%) |

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        <video class="img-fluid rounded z-depth-1" controls autoplay loop muted playsinline>
            <source src="/assets/video/manipulation_vision/orange_16x16_touch.mp4" type="video/mp4">
        </video>
    </div>
    <div class="col-sm mt-3 mt-md-0">
        <video class="img-fluid rounded z-depth-1" controls autoplay loop muted playsinline>
            <source src="/assets/video/manipulation_vision/orange_individual_touch.mp4" type="video/mp4">
        </video>
    </div>
</div>
<div class="caption">
    Left: 16×16 tactile heatmap on both finger pads as an orange oscillates between them. Right: individual touch site activations visualised in the 3D scene.
</div>

The video shows the 3D gripper scene alongside two 16×16 heatmaps (one per finger) with a live HUD showing active cell count and contact count per frame.

```bash
# Headless benchmark (~4s)
python scripts/orange_individual_touch.py

# Render annotated mp4
python scripts/orange_individual_touch.py --render
```

---

### RGB-D Streaming

<div class="row mt-3">
    <div class="col-sm-8 mt-3 mt-md-0">
        <video class="img-fluid rounded z-depth-1" controls autoplay loop muted playsinline>
            <source src="/assets/video/manipulation_vision/rgbd_stream.mp4" type="video/mp4">
        </video>
    </div>
</div>
<div class="caption">
    RGB-D point cloud streaming from the wrist camera, used as the vision input for VLA inference.
</div>

---

### Docker + Cloud Deployment

The full simulation stack (MuJoCo 3.3.1 + robosuite fork + robocasa fork) is packaged as a Docker image based on `ubuntu:22.04`:

```dockerfile
FROM ubuntu:22.04
ENV MUJOCO_GL=osmesa       # headless by default; override with MUJOCO_GL=glfw + DISPLAY
RUN apt-get install -y python3-pip python3-tk libosmesa6-dev libgl1-mesa-glx libglfw3-dev ...
RUN pip install mujoco==3.3.1 matplotlib trimesh
COPY includes/robosuite includes/robosuite && pip install -e includes/robosuite
COPY includes/robocasa  includes/robocasa  && pip install -e includes/robocasa
```

The repo ships `scripts/deploy/deploy.py` — a single script that deploys and validates the image on **AWS EC2, Lambda Labs, Vast.ai, or GCP** without any manual SSH steps. It auto-resolves instance IPs from each provider's API, installs Docker if missing, clones the repo directly on the remote (no rsync), builds the image, and runs four headless validation tests.

```bash
# GCP example (IP auto-resolved from instance ID)
python3 scripts/deploy/deploy.py \
  --provider gcp \
  --instance-id <instance-id> \
  --gcp-project <project> \
  --gcp-zone us-east4-c \
  --key ~/.ssh/gcp_key \
  --git-token-env GITHUB_TOKEN

# Any SSH host (Paperspace, RunPod, etc.)
python3 scripts/deploy/deploy.py \
  --host <IP> --key ~/.ssh/key.pem
```

---

### Path to GPU-Scale Training

The current stack runs on CPU MuJoCo. The GPU migration path I've mapped out:

| Backend | Effort | Keeps MuJoCo physics | Keeps tactile sensing | GPU rendering |
|---|---|---|---|---|
| MJX (JAX) | 13–19 wk | ✅ | ✅ | ❌ |
| Newton + MJWarp + MJLab | **7–11 wk** | ✅ | ✅ | ✅ |
| ManiSkill3 (PhysX) | 2–4 wk/task | ❌ | ⚠️ | ✅ |

**The recommended path is Newton + MJWarp + MJLab.** The key insight: robocasa builds a `mjModel` on CPU (YAML → fixtures → XML → compile) and MJWarp can ingest any `mjModel` via `mjwarp.put_model()`. Object positions and door states are `qpos` — part of the *state*, not the model — so you only need to pre-compile ~25 kitchen topology variants, not 1,250+. At each episode reset, CPU placement sampling runs once per batch, then bulk-loads `qpos` into all N parallel GPU envs.

[MJLab](https://github.com/mujocolab/mjlab) (built on MJWarp, used by Unitree) provides an Isaac Lab-style manager pattern (observations, rewards, event handlers) that directly replaces robosuite's step loop without requiring a pure-JAX rewrite. Newton (NVIDIA + Google DeepMind + Disney Research, Apache 2.0) achieves ~50M steps/sec on an RTX 4090 — 475× faster than MJX.

**For near-term prototyping at scale:** [ManiSkill3](https://github.com/haosulab/ManiSkill) (SAPIEN/PhysX) already has a partial robocasa kitchen port, native MJCF import, and the [ManiSkill-ViTac 2025 challenge](https://arxiv.org/html/2411.12503v1) specifically benchmarks vision+tactile manipulation — directly relevant to the `<touch>` sensor work here.

**The prototype-then-port workflow:**
1. **Now (CPU):** develop rewards and policy architecture in robocasa. Use geometry-based rewards (object positions, joint angles) not contact-based ones — these transfer cleanly to any GPU framework.
2. **Near-term (ManiSkill3):** port PiperOmron MJCF + kitchen task for fast pixel-based RL. Robocasa kitchen partial port already exists.
3. **Scale (Newton/MJLab):** production GPU training with full MuJoCo fidelity and tactile sensing intact.

---

### What's Next

- **VLA inference pipeline**: connecting the RGB-D stream to a vision-language-action model for language-conditioned pick-and-place
- **Sim-to-real transfer**: validating the trapezoidal motion profile and touch-based grasp detection on the physical Piper arm over CAN bus
- **ManiSkill3 port**: PiperOmron MJCF + PnP kitchen task as the first GPU-scale training environment

Code at [github.com/dt1729/manipulation_vision](https://github.com/dt1729/manipulation_vision).
