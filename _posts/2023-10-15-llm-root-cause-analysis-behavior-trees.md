---
layout: post
title: Using LLMs for Root Cause Analysis in Robotic Behavior Trees
date: 2023-10-15 10:00:00-0400
description: Building a GPT-4/LangChain chatbot for automated behavior tree root cause analysis using K-Means prompt clustering at CU Boulder's CAIRO Lab
tags: LLM Behavior-Trees ROS2 GPT-4 LangChain Robotics
categories: Research
---

### The Problem: Debugging Behavior Trees at Scale

Behavior trees have become a popular architecture for robot task execution — they're modular, readable, and handle complex conditional logic gracefully (I wrote about the fundamentals in an [earlier post](/blog/2023/BehaviorTree-FSM-and-HFSM/)). But when a behavior tree fails in the middle of a task, diagnosing *why* is hard. The tree might have hundreds of nodes, the failure might be transient, and the relevant log entries are buried in a stream of ROS topic messages, node status updates, and sensor readings.

This is the root cause analysis (RCA) problem for behavior trees — and it's what I worked on at the CAIRO Lab during my M.S.

### Approach: LLM-Based Chatbot with Log Retrieval

The core idea: rather than manually sifting through logs, give an LLM (GPT-4) the relevant log context and let it reason about the failure. The challenge is that ROS logs are verbose — you can't dump the entire log into the context window. You need to retrieve the *right* subset of logs.

The pipeline:
1. **Log ingestion**: After a task execution, collect all ROS logs, behavior tree node status events, and sensor readings into a structured store.
2. **K-Means prompt clustering**: Embed log lines using a sentence embedding model, cluster with K-Means, and select representative logs from each cluster. This gives a compact, diverse summary of what happened during execution without naively truncating to the most recent N lines.
3. **LangChain chatbot**: Feed the retrieved log context into a GPT-4 chain with a system prompt instructing it to act as a robotics debugging assistant. The user (or an automated evaluation harness) can ask natural-language questions: *"Why did the pick attempt fail?"* or *"At what point did the arm leave its safe zone?"*
4. **Interactive RCA**: The chatbot can ask follow-up questions and request additional log context, simulating a debugging session.

### The Testbed: Pick-and-Insert with a Sawyer Arm

To have a concrete task to debug, I implemented a **pick-and-insert pipeline for a Sawyer manipulator arm** using MoveIt2 (for motion planning) and BehaviorTree.CPP (for task execution). The behavior tree had nodes for:
- Object detection and pose estimation (camera + ArUco or point cloud segmentation)
- Grasp planning (MoveIt2)
- Insertion with force-torque feedback
- Error recovery (retry, re-home, abort)

This gave a rich source of realistic failures: grasp misses, IK failures, insertion contact errors, timeout conditions. Each failure left a distinct trace in the logs — the kind of thing an experienced robotics engineer could diagnose in 5–10 minutes. The question was whether an LLM could do it automatically.

### On-Device Inference: Llama-2 with AWQ Quantization

GPT-4 requires cloud API access, which isn't always acceptable (latency, cost, data privacy). I also benchmarked **Llama-2 with AWQ (Activation-aware Weight Quantization)** for on-device inference on an **Intel i7** edge machine. AWQ is a post-training quantization method that preserves accuracy better than naive weight rounding by identifying and protecting salient weights.

The results: Llama-2 (13B, AWQ 4-bit) ran at ~4 tokens/second on the i7 iGPU — slow for interactive use but viable for batch offline analysis. Accuracy on the RCA task was noticeably behind GPT-4, particularly for multi-step reasoning (e.g., tracing a failure back through three node transitions). The gap narrowed with fine-tuning on domain-specific logs, which remains future work.

### SAM-Based Scene Graph Pipeline

A complementary component was a **scene graph pipeline from robot video streams** using Segment Anything Model (SAM). The idea: during task execution, the robot camera records video. SAM segments each frame into semantic regions (objects, workspace, gripper). By tracking which objects are present and where across frames, you can build a scene graph timeline — a structured record of the workspace state at each point in the task. This gives the LLM richer context than raw logs alone: *"At t=12.3s, the object was in the gripper; at t=12.8s, it was no longer detected."*

### What This Research Taught Me

The most important lesson: LLMs are genuinely useful for structured diagnostic reasoning when given the right context — but context retrieval is the hard part. K-Means clustering of log embeddings is a pragmatic solution; better approaches (dense retrieval, hierarchical summarization) are obvious extensions. The work also reinforced that simulation/hardware testbeds are essential for evaluating AI-assisted robotics tools: you need ground-truth failure cases to measure against.
