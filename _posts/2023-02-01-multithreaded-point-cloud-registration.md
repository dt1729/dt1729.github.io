---
layout: post
title: Multithreaded Point Cloud Pipeline for RealSense + SLAM
date: 2023-02-01 10:00:00-0500
description: Replacing ROS IPC between a RealSense D455 and a SLAM backend using a C++ producer-consumer pipeline to reduce network-level overhead
tags: SLAM C++ Point-Cloud RealSense Threading
categories: Perception
---

### Motivation

ROS's inter-process communication uses a network socket layer even between processes on the same machine. When passing dense point clouds between a sensor driver node and a SLAM node at high frequency, this introduces measurable latency and consumes significant network bandwidth — ROS's `xmlrpc_manager` traffic alone can be a problem at scale. The goal of this project was to replace that ROS IPC boundary with a direct in-process queue, keeping the sensor and SLAM algorithm in the same process and moving data through CPU memory rather than the network stack.

### Architecture: Producer-Consumer with std::queue

The pipeline uses a classic producer-consumer pattern:

- **Producer thread** reads depth frames from an Intel RealSense D455 via the RealSense SDK and pushes them onto a bounded `std::queue`.
- **Consumer thread** pops from the queue and feeds the point clouds into the SLAM front-end.

Thread safety is managed with `std::unique_lock` and a condition variable. The queue is bounded — when the producer fills the queue, it blocks rather than growing without bound, preventing runaway memory usage when the consumer falls behind.

The design also anticipates multiple consumers reading from the same queue (e.g., multiple SLAM algorithms reading the same sensor stream), which is why a producer-consumer model was chosen over a simple reader-writer pattern.

```cpp
// Producer pushes; consumer pops under unique_lock
std::unique_lock<std::mutex> lock(mtx);
cv.wait(lock, [&]{ return queue.size() < MAX_QUEUE_SIZE; });
queue.push(frame);
lock.unlock();
cv.notify_all();
```

### Results

Running the pipeline sustained **30 Hz** point cloud capture with no frame drops, and reduced network bandwidth utilization by approximately **60%** compared to the ROS-based baseline (where point clouds travel through ROS's socket layer between processes). The bounded queue depth provided enough buffer to absorb transient slowdowns in the SLAM consumer without stalling the producer.

### What's Done

- RealSense D455 capture class (`PointCloudCapture.cpp`) with IMU integration for orientation estimation (complementary filter on gyro + accelerometer)
- Producer-consumer threading infrastructure with bounded queue and `std::unique_lock`
- Benchmarked end-to-end at 30 Hz with 60% bandwidth reduction vs. ROS IPC baseline
- Submodule wiring for Lego-LOAM and floam-multithreaded

### What's Still In Progress

The Lego-LOAM integration is not yet complete — Lego-LOAM's internals still have ROS dependencies that need to be decoupled before it can run as an in-process library rather than a separate node. CUDA acceleration for PCL operations is the next planned step.

### Repository

[github.com/dt1729/MultithreadedRegistration](https://github.com/dt1729/MultithreadedRegistration)
