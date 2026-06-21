---
layout: post
title: Multi-Agent Search and Rescue using Unmanned Surface Vehicles
date: 2022-08-23 09:47:00-0400
description: Controller design for multi-agent Victor Sierra search using USV16s — consensus formation, leader-follower tracking, and PD trajectory control in Gazebo VRX
tags: Multi-agent-robotics Formation-Control USV ROS2 Gazebo Control
categories: Control
---

### Motivation

Search and rescue at sea has relied on standardized geometric search patterns like the **Victor Sierra search** since the Coast Guard's early days. The pattern works by having a single vessel sweep a clockwise spiral around a datum point. The problem: it's slow. With the tools of multi-agent systems, the same area can be covered much faster — 9 agents each covering 1/9 of the path at the same speed gives a **9× speedup** in search time. This project designs and simulates the full controller stack to make that work.

### The Vehicle: USV16

The USV16 is an underactuated surface vehicle with 3 degrees of freedom — surge (forward), sway (lateral), and yaw (rotation) — but actuation in only two: surge via propellers and yaw via rudders. There is no direct sway actuation, which creates a circling tendency that makes trajectory tracking non-trivial. The dynamics are:

$$\dot{\eta} = J(\eta)\nu, \quad M\dot{\nu} + C(\nu)\nu + D(\nu)\nu = \tau + \tau_w$$

where η = [x, y, φ], ν = [u, v, ω], M is the inertia+added-mass matrix, C is Coriolis, D is drag, and τ_w is environmental disturbance (waves).

The system is **Globally Exponentially Stable** for waypoint following and **Globally Asymptotically Stable** for trajectory tracking in Frenet-Serret coordinates, per the proofs in Do & Pan (2009).

### Modified Victor Sierra Search with 9 Agents

The standard Victor Sierra pattern places a single agent at the datum. The modified version places **9 agents at the waypoints of the pattern simultaneously**, so each agent covers only 1/9 of the total path. At the same individual speed, the full pattern completes 9× faster.

### Controller Architecture

The full stack has three layers:

**1. Multi-agent controller** (consensus + leader-follower):

For n agents sharing position information over a communication graph, the consensus update drives the formation:

$$\dot{X}_i = \sum_{j=1\ldots N}(X_i - X_j) - \zeta_i - \zeta_j$$

where ζᵢ are the desired formation offsets. This is **Globally Asymptotically Stable**, proved via Lasalle's invariance principle using the Laplacian of the complete communication graph. The leader-follower extension drives the formation toward the datum — the leader moves along the Victor Sierra waypoints with smooth heading:

$$\phi_{leader} = \tan^{-1}\left(\frac{y_{waypoint} - y}{x_{waypoint} - x}\right)$$

**2. Trajectory tracking controller** (PD + nonlinear yaw mapping):

A nonlinear speed-yaw coupling handles the underactuation — when yaw error is large, surge slows to allow turning; when yaw error is small, surge increases to track the waypoint quickly:

$$u(\phi_e) = 1 - \frac{1}{1 + 3e^{-0.6|\phi_e|}}$$

A PD controller on yaw error plus this nonlinear speed map, combined with differential thrust allocation (right thruster + rudder for right turns, left for left, both for straight), gives the full low-level controller.

**3. Low-level PID** for actuator commands (implemented by the VRX Gazebo platform).

### Results

**Open-loop characterization**: Three open-loop tests (max thrust, left-only, right-only) characterized the vehicle dynamics. Speed under max thrust is slow (~0.5–1 m/s) due to drag, and wave disturbances introduce oscillation in speed — indicating coupling between surge, sway, yaw, pitch and roll. This confirms that 6-DOF control would be needed for perfect tracking; the 3-DOF design is an accepted approximation.

**Closed-loop trajectory tracking**:
- **U-turn test**: Small overshoot, total drift < 3 m at the sharpest turn. Some yaw oscillation, tunable with more iterations.
- **Circular test**: Drift < 2 m, with chattering around ±π due to angle wrapping at the heading discontinuity.

**Formation control**: Consensus converges to the desired 9-agent formation geometry. Leader-follower tracking drives the formation toward the datum.

**Full system integration**: When all layers are combined to run the actual Victor Sierra search, the system encounters a **fail case due to the absence of dynamic obstacle avoidance** — agents collide in the simulation because no inter-agent collision handling is implemented.

### What's Still In Progress

The individual controllers are complete, but full integration to run the Victor Sierra search end-to-end still needs dynamic obstacle avoidance between agents — the current design has no collision handling between vehicles, which causes a fail case when all layers run together.

### Repository

[github.com/dt1729/MultiAgentSys](https://github.com/dt1729/MultiAgentSys) (`surface_veh` branch)
