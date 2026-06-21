---
layout: post
title: "BehaviorTree-RCA: LLM-Assisted Debugging for a Sawyer Pick-and-Insert Robot"
date: 2026-05-26 10:00:00-0400
description: A concrete walkthrough of using BehaviorTree.CPP, ROS Noetic, and a GPT-3.1/LangChain chatbot to build and debug a 3-object pick-and-insert pipeline on a Rethink Sawyer arm.
tags: Behavior-Trees ROS LLM GPT Robotics Manipulation Vision
categories: Research
---

This post walks through a real system I built at the CAIRO Lab: an autonomous pick-and-insert pipeline running on a Rethink Sawyer arm, orchestrated with [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP), and equipped with a GPT-3.1 chatbot that helps diagnose failures after each experimental run. (For a conceptual comparison of BTs vs FSMs vs HFSMs, see my [earlier post](/blog/2023/BehaviorTree-FSM-and-HFSM/).)

---

### The Task

The robot must pick three objects — a **ket**, a **big cylinder**, and a **small cylinder** — from a textured mat and insert each one into its corresponding hole on a **NIST assembly board**. All three objects have different geometries; none are distinguishable by color alone. The insertion holes are sub-centimeter in diameter, so pose accuracy matters.

**Hardware:**
- Rethink Sawyer arm (7-DOF)
- Rethink electric parallel gripper
- Intel RealSense D400 (aligned RGB + depth)
- NIST board with AprilTag `tag_118` as an insertion pose reference

---

### Architecture: Client-Server Split

The software has a clean two-process split mandated by the Intera SDK:

```
┌──────────────────────────────┐        ROS services         ┌──────────────────────────────┐
│  BTClient.cpp  (C++)         │ ──────────────────────────> │  BTNodeServer.py  (Python)    │
│  BehaviorTree.CPP executor   │                             │  Intera SDK / Sawyer hardware │
└──────────────────────────────┘                             └──────────────────────────────┘
             ▲                                                             ▲
             │ reads TF frames                                             │ looks up TF at runtime
             └─────────────────────────────────────────────────────────────┘
                              ┌──────────────────────────┐
                              │  Vision nodes  (Python)  │
                              │  pickPlacePoseDetermination.py
                              │  inference_node.py        │
                              └──────────────────────────┘
```

The C++ client is the BT executor. It knows nothing about hardware — it calls services. The Python server wraps `intera_interface` and never imports BehaviorTree.CPP. Vision nodes run independently and publish TF frames; the server looks up those frames by name on demand.

This separation was practically important: the Intera SDK has its own threading model that conflicts with the BT tick loop if run in the same process.

---

### Behavior Tree Design

The active tree is `finalBehaviorTree`, which executes the full three-object sequence. Each pick-insert cycle follows the same pattern:

```
Sequence
├── RetryUntilSuccessful(5)
│   └── ServoToPose("bigCylinder_location")   # vision-guided approach
├── RetryUntilSuccessful(5)
│   └── gripperClose
├── RetryUntilSuccessful(5)
│   └── retract(over_mat=False)               # lift, move to board side
├── RetryUntilSuccessful(5)
│   └── ServoToPose("bigCylinder_insertion")  # AprilTag-guided insert
├── RetryUntilSuccessful(5)
│   └── gripperOpen
└── RetryUntilSuccessful(5)
    └── retract(over_mat=True)                # return to mat side
```

The tree repeats this for all three objects. Wrapping every leaf in `RetryUntilSuccessful(5)` provides fault tolerance against transient IK failures and service timeouts without requiring any manual error handling inside the action nodes.

The action nodes defined in `BTClient.cpp`:

| Node | What it does |
|---|---|
| `gripperOpen / gripperClose` | Calls `GripperCmd` service |
| `approach` | Calls `ApproachCmd` with a hover pose from the blackboard |
| `ServoToPose` | Passes a TF frame name to `ServoToPoseCmd`; the server resolves the pose at call time |
| `retract` | Moves to one of two safe joint configurations (mat-side / board-side) |
| `visualFeedback` | Subscribes to `/visionFeedback/MeanValue`, transforms pose camera→gripper frame, writes to blackboard |

The late binding in `ServoToPose` is the key design choice: the BT encodes the *name* of the target frame, not the pose itself. The vision node can keep updating the frame while the BT runs, and the robot always acts on the freshest estimate.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/BehaviorTree.svg" class="img-fluid rounded" zoomable=true %}
    </div>
</div>
<div class="caption">
    Subtree structure for a single pick-insert cycle.
</div>

---

### Vision Pipeline

#### Pick pose — `pickPlacePoseDetermination.py`

Detecting the ket in a cluttered scene:

1. **Bilateral filter** on RGB to smooth noise while preserving edges
2. **HSV threshold** to isolate the grey ket against the violet mat
3. **Canny edge detection** → contour extraction
4. **Area filter** (320–380 px²) calibrated for 90 cm camera height to reject noise and large surfaces
5. **Ellipse fit** on the largest matching contour; centroid gives the (u, v) pixel location
6. **Back-projection** using aligned depth + Open3D intrinsics to get the 3D point in camera frame
7. Apply hand-eye calibration (`GripperToCameraTransform.json`) to transform to gripper frame
8. Compute bigCylinder and smallCylinder positions via fixed offsets from `referencePickLocations.json`
9. Broadcast TF frames: `ket_location`, `bigCylinder_location`, `smallCylinder_location`

The fixed-offset approach for the two cylinders works because they are placed in consistent positions relative to the ket on the mat. It avoids needing a separate detector per object.

#### Insert pose — AprilTag + offsets

The NIST board carries a single AprilTag (`tag_118`), detected continuously by `apriltag_ros`. Fixed offsets from `referenceInsertLocations.json` give the 3D insertion point for each hole relative to the tag. This broadcasts: `ket_insertion`, `bigCylinder_insertion`, `smallCylinder_insertion`.

#### Hole detection — `inference_node.py`

A **YOLOv7** model fine-tuned on the NIST board runs on the RealSense stream to detect individual hole positions when finer precision is needed. The trained weights are stored in `NIST_board.pt`.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/CalibrationOutput.png" class="img-fluid rounded" zoomable=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/PickPose.png" class="img-fluid rounded" zoomable=true %}
    </div>
</div>
<div class="caption">
    Left: hand-eye calibration output. Right: pick pose detection overlay.
</div>

---

### Root Cause Analysis Chatbot

After a run, the observer starts `textFeedback.py`. The pipeline:

#### 1. Context loading

The script parses **docstrings from `BTNodeServer.py`** and **C++ doc-comments from `BTClient.cpp`** to build a plain-text description of every node and service. It also reads the behavior tree XML. These form the static system context injected once into the chain.

#### 2. Log ingestion

`~/.ros/log/CommandServer.log` is parsed into a Pandas DataFrame keyed by BT node name and severity level. Entries are stored by node so they can be retrieved selectively.

#### 3. Seeding the LangChain chain

A `ConversationChain` backed by **GPT-3.1** receives a system prompt that includes:
- The architecture overview
- Code descriptions extracted above
- The behavior tree XML
- Instructions to act as a robotics debugging assistant

#### 4. Node classification via KMeans

At each turn, the AI's reply is embedded with `text-embedding-ada-002` and classified by a pre-trained **KMeans model** (`QuestionClassificationModel.pkl`). The model was trained by generating 20 representative questions per BT node using GPT, embedding them, and fitting clusters. Each cluster maps to a node name via `ClassificationLabels.json`.

If a node is identified, its full log entries are prepended to the *user's next message*, giving the model the actual runtime data when reasoning about the failure.

#### 5. Conversation loop

The observer describes what they saw in natural language. The chatbot asks targeted follow-up questions. Typing `ANALYSIS COMPLETE` ends the loop, saves the transcript, and archives the log file.

This cleanly separates concerns: GPT-3.1 handles reasoning, KMeans handles retrieval routing, and the log store provides ground truth. An observer who saw something go wrong can describe it in plain English and get a focused hypothesis about which node misbehaved — in a few exchanges rather than 20 minutes of manual log inspection.

---

### Current Work: SAM-PT Tracking

The active branch (`visionGPT`) adds **SAM-PT mobile** tracking to the vision pipeline. Instead of running Canny + threshold on every frame independently, the idea is to bootstrap the object mask once with SAM-PT and propagate it across frames. This should handle cases where lighting or mat color shifts enough to break the HSV threshold mid-run.

The branch also adds interactive picture functions for annotating frames on-the-fly during an experiment — useful for collecting failure examples for the RCA chatbot.

---

### What Worked and What Didn't

**Worked well:**
- The client-server split avoided every Intera SDK / ROS threading conflict I ran into in earlier prototypes.
- `RetryUntilSuccessful` on every leaf is blunt but effective. Most pick failures were IK solver non-determinism, not perception errors.
- KMeans retrieval routing is simple but surprisingly accurate at identifying the relevant BT node from a natural-language question.

**Harder than expected:**
- The fixed contour-area filter for the ket is fragile to camera height variation — a 5 cm difference shifts the ket contour outside the accepted range.
- Insertion precision on sub-centimeter holes is right at the edge of what open-loop `ServoToPose` can achieve. Force-torque feedback is the obvious next step.

---

### Repository

[github.com/dt1729/Behaviortree-rca](https://github.com/dt1729/Behaviortree-rca)
