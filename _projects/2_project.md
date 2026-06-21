---
layout: page
title: Trajectory Optimization via Successive Convexification
description: Exploits quadrotor differential flatness to eliminate polynomial fitting; runs at 5 Hz in Julia
img: assets/img/3.jpg
importance: 2
category: work
---

Trajectory optimization for quadrotors using successive convexification, exploiting the differential flatness of quadrotor dynamics to sidestep polynomial fitting entirely.

**Key contributions:**
- Derived flatness-based formulation eliminating polynomial trajectory representation
- Successive convexification outer loop converges to locally optimal trajectories
- Runs at **5 Hz** in Julia using CVXPY-compatible convex solvers

**Stack:** Julia, CVXPY, CasADi
