---
layout: post
title: Formation Control via Visual Feedback and Hybrid Switching Controllers
date: 2022-11-10 10:00:00-0500
description: Designing a switching hybrid controller for multi-agent source seeking using a CNN-based localizer
tags: Multi-agent-robotics Control Computer-Vision CNN TensorFlow
categories: Control
---

### Motivation

Most formation control work assumes agents have access to accurate global position estimates — GPS, motion capture, or a well-calibrated localization stack. What happens when you want to do formation control using only a camera? This project explored exactly that: a **switching hybrid controller** for multi-agent source seeking, where each agent uses a CNN to localize itself relative to a visual source, and a hybrid controller coordinates collective motion.

### The Localization Problem

The first sub-problem is giving each agent a sense of where it is relative to the source purely from an onboard camera image. I trained a **convolutional neural network in TensorFlow** to regress relative position (bearing and approximate distance) from raw camera frames. The network architecture used a lightweight encoder — small enough to run at control rates — followed by a regression head outputting the relative pose estimate.

Training data came from a structured collection across varying distances, angles, and lighting conditions. The trained model achieved **92% accuracy on the training set and 85% on validation**, which was sufficient for the controller to close the loop reliably in simulation.

### Hybrid Switching Controller

The controller design was motivated by the observation that source-seeking and formation-maintenance are qualitatively different objectives that don't compose cleanly into a single continuous control law. A **switching hybrid controller** handles this by defining discrete modes:

- **Seeking mode**: Each agent drives toward the source independently, using the CNN bearing estimate as a reference signal.
- **Formation mode**: Once agents are within a threshold distance of the source, the controller switches to consensus-based formation control to maintain relative geometry.

The switching logic is a guarded transition: the system enters formation mode when all agents satisfy a proximity condition, and falls back to seeking mode if any agent loses the source from its field of view.

### Consensus Extension

The formation control layer uses a standard **Laplacian consensus protocol** over the communication graph. Each agent maintains an estimate of its neighbors' relative positions (exchanged over a simulated communication channel) and applies a linear consensus update to drive the formation error to zero. The control law is:

```
u_i = -Σ_{j ∈ N_i} (p_i - p_j - d_ij)
```

where `d_ij` is the desired inter-agent offset vector. This drives the formation to the desired geometry while the group collectively seeks the source.

### Results

The hybrid controller converged to the source and maintained formation in simulation across a range of initial configurations. The switching behavior was stable — the system didn't chatter between modes because the proximity threshold was set with sufficient hysteresis. The main limitation was CNN accuracy degrading at oblique angles and low light, which occasionally caused a brief fallback to seeking mode even when agents were near the source.

### What I Learned

This project was my first deep exposure to the interplay between perception and control. A key insight: the CNN doesn't need to be perfect — it just needs to be accurate enough that the closed-loop system is stable and converges. The hybrid controller design also introduced me to formal reasoning about mode switching and stability, which has influenced how I think about robustness in autonomy stacks ever since.
