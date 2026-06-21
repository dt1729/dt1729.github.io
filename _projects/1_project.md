---
layout: page
title: Kinodynamic Motion Planning of Quadrotor
description: SST* solver for SE(3) free-space planning with optimal control, replanning at 100 Hz on map updates
img: assets/img/12.jpg
importance: 1
category: work
---

Real-time kinodynamic motion planning for a quadrotor in SE(3) free space using the SST* (Stable Sparse RRT*) solver via OMPL.

**Key contributions:**
- Implemented SST* for SE(3) planning with full quadrotor dynamics
- Replanning loop runs at **100 Hz** on half-plane map updates
- Extending to Octomap-based 3D occupancy representations

**Stack:** C++, OMPL, ROS
