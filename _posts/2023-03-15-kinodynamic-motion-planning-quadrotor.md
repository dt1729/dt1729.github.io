---
layout: post
title: Kinodynamic Planning of a Quadrotor in SE(3) using SST*
date: 2023-03-15 10:00:00-0500
description: Implementing SST* on OMPL's SST solver for SE(3) quadrotor planning — static obstacles, dynamic windowed replanning, and Rotors+Octomap integration
tags: Motion-Planning Quadrotor OMPL SST C++ SE3 Gazebo
categories: Planning
---

### Motivation

Planning collision-free trajectories for a quadrotor in 3D is hard because you can't separate geometry from dynamics — the vehicle can't follow an arbitrary geometric path instantly. **Kinodynamic planning** jointly searches for control inputs and collision-free states, giving you a trajectory the vehicle can actually execute. This project implements SST\* for SE(3) quadrotor planning via OMPL, with three progressively harder experiments: static obstacles, dynamic windowed replanning, and integration with a Gazebo simulator and Octomap.

### Vehicle Model

The quadrotor is modeled with a simplified SE(3) kinematic model. Only pitch (θ) and yaw (ψ) are actuated; roll is zero:

$$\dot{x} = v\cos(\psi)\cos(\theta), \quad \dot{y} = v\sin(\psi)\cos(\theta), \quad \dot{z} = v\sin(\theta)$$
$$\dot{\psi} = \omega, \quad \dot{\theta} = \alpha, \quad \dot{\phi} = 0$$

This gives 6 state variables and 3 control inputs [v, ω, α], all bounded. The intent is for a lower-level attitude controller (e.g., Pixhawk) to handle actuation, while the kinodynamic planner handles trajectory generation.

### SST*: Asymptotically Optimal Kinodynamic Planning

The planner is **SST\*** (Sparse methods for efficient asymptotically optimal kinodynamic planning — Li, Littlefield, Bekris 2015), implemented on top of OMPL's `oc::SST` class. SST\* is notable because most kinodynamic planners provide no optimality guarantees — SST\* achieves asymptotic near-optimality with minimal demands on the dynamics, using a sparse witness set to prune redundant nodes and progressively lower solution cost over iterations.

Key parameters:
- δs: witness node radius (pruning radius = 2)
- δv: exploration region near random sample (selection radius = 8 for static, 4 for windowed)
- Tprop: forward propagation time per edge
- Euler integrator with tf2-based quaternion ↔ Euler conversion (more reliable than Eigen for this purpose)

### Experiment 1: Static Obstacle Planning

Start: (1,1,5), Goal: (10,8,10), 100 SST iterations.

The planner successfully found a collision-free path avoiding axis-aligned box obstacles. Cost decreased visibly across iterations as SST\* added nodes and pruned suboptimal branches. The planning time was notable even for this small environment — adding a time budget constraint is an identified improvement.

### Experiment 2: Dynamic Windowed Replanning

Start: (0.5,6,1), Goal: (39,3.5,5) — a 40-meter corridor with obstacles revealed incrementally.

The replanning loop:
1. Identify a local goal collinear with the global goal from the current position
2. Plan to the local goal; execute until 80% complete
3. Update the local goal, refresh the state space bounds with newly revealed obstacles, replan
4. Repeat until within 1 m of the final goal

This worked — the planner navigated the full corridor without intersecting obstacles. The known limitation: the planner **occasionally gets stuck in local minima** and exhausts valid states, requiring a hard restart. A hard restart recovered the situation each time.

### Experiment 3: Rotors Simulator + Octomap Integration

The third experiment integrates the planner with the RotorS Gazebo MAV simulator and Octomap for a sensor-driven occupancy map. Key integration steps:
- RotorS provides a depth camera → point cloud
- A custom `update_frame.py` node stamps the point cloud with the correct frame ID (RotorS doesn't publish frame IDs)
- The Octomap server subscribes to `/firefly/vi_sensor/camera_depth/depth/points` and builds a 3D occupancy map

The occupancy map was successfully generated. The full experiment — moving the quadrotor around to build a map and then planning in it — was **not completable** because the RotorS quadrotor requires a continuous feedback controller (joystick or attitude controller) to stay airborne; autonomous waypoint tracking was not wired up within the project time.

Both Kimera VIO and Voxblox ESDF were also attempted for richer scene representation, but both had integration issues (missing frame IDs, no built-in relocalization) that were out of scope for the project timeline.

### Takeaways

SST\* works well for SE(3) kinodynamic planning when tuned properly. The windowed replanning approach handles incremental obstacle discovery but is susceptible to local minima — a potential-field escape or random restarts are reasonable mitigations. The path to a full hardware-ready system requires wiring in a low-level attitude controller (e.g., Pixhawk via MAVROS) to execute the SE(3) waypoints SST\* produces.

### Repository

[github.com/dt1729/QuadrotorPlanning](https://github.com/dt1729/QuadrotorPlanning)
