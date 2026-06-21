---
layout: page
title: LLM-Based Root Cause Analysis for Behavior Trees
description: GPT-3.1/LangChain chatbot for automated robot behavior tree root cause analysis using K-Means prompt clustering on a Sawyer pick-and-insert pipeline
img: assets/img/BehaviorTree.svg
importance: 6
category: work
---

Research project at the CAIRO Lab (CU Boulder) applying large language models to automated root cause analysis of robotic behavior tree failures.

**Key contributions:**
- Built GPT-3.1/LangChain chatbot that retrieves relevant logs via K-Means prompt clustering for targeted RCA queries
- Implemented 3-object pick-and-insert pipeline for Sawyer arm using BehaviorTree.CPP and ROS Noetic as the evaluation testbed
- Vision pipeline using Canny edge detection + RealSense depth for pick pose estimation; AprilTag for insertion pose
- Developed SAM-PT based scene graph pipeline from robot video streams for task state analysis

**Stack:** Python, LangChain, GPT-3.1, BehaviorTree.CPP, ROS Noetic, C++17, Open3D

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/BehaviorTree.svg" class="img-fluid rounded" zoomable=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/PickPose.png" class="img-fluid rounded" zoomable=true %}
    </div>
</div>
<div class="caption">
    Left: behavior tree subtree design. Right: pick pose detection output.
</div>

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        <video class="img-fluid rounded" controls>
            <source src="/assets/video/SmallKetPickAndPlace.mp4" type="video/mp4">
        </video>
    </div>
</div>
<div class="caption">
    Full ket pick-and-place run on the Sawyer arm.
</div>
