---
layout: page
title: Formation Control via Visual Feedback
description: Switching hybrid controller for multi-agent source seeking with CNN localizer at 92% train accuracy
img: assets/img/ROC_visual_feedback.png
importance: 4
category: work
---

Hybrid controller for multi-agent source seeking using visual feedback from a CNN-based localizer.

**Key contributions:**
- Designed switching hybrid controller for multi-agent source seeking
- Trained CNN localizer achieving **92% train / 85% val accuracy** in TensorFlow
- Extended consensus protocol to full formation control

**Stack:** Python, TensorFlow, C++, ROS

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/ROC_visual_feedback.png" class="img-fluid rounded" zoomable=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/confusion_matrix_train.png" class="img-fluid rounded" zoomable=true %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/confusion_matrix_val.png" class="img-fluid rounded" zoomable=true %}
    </div>
</div>
<div class="caption">
    ROC curve (AUC 0.60) and train/validation confusion matrices for the CNN localizer.
</div>
