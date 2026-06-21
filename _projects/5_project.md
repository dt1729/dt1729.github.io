---
layout: page
title: Multithreaded Point Cloud Registration
description: Producer-consumer pipeline feeding RealSense D455 point clouds to CUDA-accelerated Lego-LOAM
img: assets/img/1.jpg
importance: 5
category: work
---

High-throughput point cloud registration pipeline using a producer-consumer architecture to feed live RealSense D455 depth data into CUDA-accelerated Lego-LOAM SLAM.

**Key contributions:**
- Designed multithreaded producer-consumer pipeline in C++17 to decouple sensor I/O from SLAM processing
- Offloaded CUDA-accelerated Lego-LOAM onto separate threads, reducing network bandwidth bottlenecks
- Integrated with Open3D for visualization and point cloud management

**Stack:** C++17, CUDA, Open3D, Intel RealSense SDK
