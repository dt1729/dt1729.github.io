---
layout: post
title: Building an Autonomy Stack for Last-Mile Delivery Vehicles
date: 2021-08-15 10:00:00-0400
description: A year at IIIT Delhi working on planning, mapping, and control for autonomous last-mile vehicles in the ALIVE project
tags: Autonomous-Vehicles ROS Planning Control LiDAR
categories: Autonomy
---

### Background

After finishing my undergraduate degree at NSIT Delhi, I spent a year at IIIT Delhi as a research assistant on the [ALIVE (Autonomous Last-mile Intelligent Vehicle)](https://sites.google.com/iiitd.ac.in/iiitd-alive) project. The goal: build a full autonomy stack for a small electric vehicle navigating urban last-mile delivery environments. I worked on the planning and controls side of the stack, with the vehicle running ROS and equipped with a Velodyne VLP-16 LiDAR and a Pixhawk flight controller.

### The Planning Stack in CARLA

Before deploying anything on the real vehicle, the team used CARLA simulator for systematic evaluation of planning algorithms. When I joined, the ROS-CARLA bridge was fragmented — individual components worked in isolation but the end-to-end pipeline wasn't coherent enough to run controlled experiments. I refactored the autonomy stack to make algorithm swapping clean and reproducible, which became the foundation for all subsequent benchmarking.

The key planning integration was bringing in a **RRT\*-based local planner via OMPL**. The global planner would hand off a coarse route, and the local planner would compute kinodynamically feasible trajectories around dynamic obstacles in real time. Getting OMPL to interface cleanly with the ROS navigation stack (nav_msgs, costmaps) required writing a custom state space for the vehicle's Ackermann kinematics and plumbing the costmap into OMPL's validity checker.

### Waypoint Smoothing

One of the first things that stood out in early tests was that the vehicle would oscillate through waypoint sequences — the raw planner output had sharp direction changes that the lateral controller couldn't track cleanly at speed. The fix was waypoint smoothing using **ALGLIB spline interpolation**. By fitting a smooth spline through the waypoint sequence and resampling at a fixed arc-length interval, the reference trajectory became C2-continuous and the lateral tracking error dropped noticeably.

### LiDAR Occupancy Grid — Bresenham's Algorithm

The LiDAR occupancy grid was a bottleneck. The naive approach — ray-casting each beam by iterating over floating-point increments — was too slow for real-time mapping at VLP-16 scan rates. Replacing it with **Bresenham's line algorithm** (integer arithmetic, no floating-point per step) cut occupancy grid construction time by **80%**, making real-time mapping viable at full LiDAR rate.

### Lateral Controllers: Pure Pursuit vs. Stanley

I implemented and benchmarked two classical lateral controllers in CARLA:

**Pure Pursuit** is a geometric controller — it picks a lookahead point on the reference path and computes the steering angle to drive the vehicle's rear axle through that point. It's simple and robust but the lookahead distance requires tuning and it can cut corners at high speed.

**Stanley** (used on the Stanford DARPA entry) computes steering from both heading error and cross-track error at the front axle. It's more aggressive about eliminating lateral error and tends to track tighter at moderate speeds.

Both were benchmarked on a set of standardized maneuvers in CARLA. Stanley outperformed Pure Pursuit on cross-track error for the curved urban segments we tested; Pure Pursuit was more stable at higher speeds on straight stretches.

### Formation / Reference Trajectory Control

Towards the end of the project, I worked on a **virtual-leader reference trajectory controller** for a convoy scenario — multiple vehicles maintaining formation behind a lead vehicle. The formulation uses a virtual leader whose trajectory is generated ahead of time, and each follower tracks relative to it using a feedback linearization approach. This outperformed the prior PID baseline on tracking accuracy, particularly through turns where the PID would accumulate lag.

### Takeaways

A year on ALIVE gave me a strong foundation in the full autonomy pipeline — from sensor preprocessing and mapping through planning and control, all the way to hardware integration on a real vehicle. The discipline of systematic evaluation in simulation before deploying on hardware, and the importance of getting the low-level details right (integer vs. floating-point ray casting, spline smoothing), shaped how I approach robotics engineering to this day.
