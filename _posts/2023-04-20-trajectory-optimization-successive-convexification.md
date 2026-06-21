---
layout: post
title: Trajectory Optimization via Successive Convexification
date: 2023-04-20 10:00:00-0400
description: Applying successive convexification to quadrotor trajectory optimization in Julia — virtual control, trust regions, and ECOS vs SCS solver benchmarking
tags: Trajectory-Optimization Convex-Optimization Quadrotor Julia SCvx ECOS
categories: Planning
---

### The Problem

Trajectory optimization sits between a motion planner (which gives a free-space path) and a low-level controller (which drives actuators). It needs to be fast enough to provide actionable reference states in response to environment changes. The challenge: the quadrotor dynamics are nonlinear, making the optimal control problem non-convex.

**Successive convexification (SCvx)** addresses this by iteratively linearizing the nonlinear problem and solving the resulting convex subproblem. Under mild conditions (proved in Mao, Szmuk & Açıkmeşe, CDC 2016) the method achieves a solution as good as the original nonlinear problem — **lossless convexification**.

### Vehicle Model

The same simplified SE(3) kinematic model used in the planning project applies here — pitch (θ) and yaw (ψ) actuated, no roll:

$$\dot{x} = v\cos(\psi)\cos(\theta), \quad \dot{y} = v\sin(\psi)\cos(\theta), \quad \dot{z} = v\sin(\theta)$$
$$\dot{\psi} = \omega, \quad \dot{\theta} = \alpha, \quad \dot{\phi} = 0$$

State: [x, y, z, ψ, θ, φ] ∈ ℝ⁶ — Control: [v, ω, α] ∈ ℝ³.

### Linearization: A, B, D Matrices

At each SCvx iteration, the nonlinear dynamics are linearized via first-order Taylor expansion around the current trajectory (xᵢ⁻¹, uᵢ⁻¹):

$$\dot{d}(t) = A(t)d(t) + B(t)w(t) + D(t) + \text{H.O.T.}$$

where d(t) = x(t) − xᵢ⁻¹(t), w(t) = u(t) − uᵢ⁻¹(t). The Jacobians A(t), B(t), D(t) are computed analytically from the dynamics.

### Virtual Control and Trust Regions

Two mechanisms handle the pathologies of linearization:

**Virtual control**: Linearization can create artificial infeasibility — problems that are feasible in the nonlinear sense but infeasible after linearization. A virtual control input v(t) is added to guarantee feasibility at every iteration, penalized in the cost as λ‖v‖₁ to drive it toward zero at convergence.

**Trust regions**: Linearization fails far from the expansion point. The control deviation is bounded by ‖w‖∞ ≤ Δ. The trust region radius Δ is updated at each iteration using the ratio ρ = ΔJ/ΔL (actual vs. predicted cost reduction): shrink if ρ < ρ₀ (bad step), keep if ρ₀ < ρ < ρ₁, grow if ρ > ρ₁.

A run **without** virtual control and trust regions failed to find a solution, confirming both mechanisms are necessary.

### Results

**Test case**: Start (0,0,0) → Goal (3,4,5), 5 SCvx iterations, δt = 0.1s, 100 timesteps.

The optimizer converged to an optimal solution in **2 iterations**. Control signals show bang-bang behavior on some channels — the expected signature of a correctly solved minimum-effort optimal control problem.

**Solver comparison (SCS vs ECOS)**:

Both SCS and ECOS are cone-program solvers. ECOS is significantly faster and takes a clear win:
- SCS: ~0.66s first iteration, ~0.29s subsequent (after JIT warmup)  
- ECOS: ~0.18s first iteration, ~0.15s subsequent

At ~5–6 Hz per ECOS solve, this is fast enough to use in a receding-horizon MPC loop for real-time trajectory updates.

The cost behavior is identical between the two solvers, so ECOS is the right choice for this problem.

### What's Still In Progress

The closed-loop trajectory tracking wrapper (`TrajectoryTrackingLoop`) — which would run SCvx in a receding-horizon MPC fashion — is partially implemented. Obstacle avoidance constraints exist in the formulation but are currently commented out.

### Repository

[github.com/dt1729/Successive-Convexification-Trajopt](https://github.com/dt1729/Successive-Convexification-Trajopt)
