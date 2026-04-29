# Real-Time Video Stabilization for a Warehouse Mobile Robot

Bachelor's thesis code repository — Korableva Viktoriia, HSE Faculty of Computer Science, BSc Data Science and Business Analytics, 2026.

## Overview

This repository contains the full implementation of a lightweight video stabilization module for a warehouse mobile robot, evaluated as a preprocessing step for downstream person detection, tracking, and person-following. Target deployment platform is the GMKtec NucBox K10 mini PC (CPU only).

The work is split into two components:

1. **Stabilization & perception pipeline** (`notebooks/`) — six stabilization variants (LK + MA, LK + Kalman, ORB + MA, robust LK + MA, robust LK + Kalman, RAFT-Small reference) compared on simulator-recorded onboard sequences. Selected production method (V2: LK + Kalman RTS) is then plugged into a YOLO11n + OC-SORT detection-tracking pipeline with a target-locking extension, evaluated across five shake regimes.

2. **Gazebo warehouse simulator integration** (`gazebo_simulator/`) — custom warehouse-like Gazebo world, robot model with onboard Intel RealSense D435, ROS 2 navigation stack with logical map and CV-driven navigator. This is the validation environment from which the stabilization sequences were captured.

## Repository structure

```
.
├── notebooks/
│   └── stabilization_perception_pipeline.ipynb
│       └─ stabilization variants V1–V5, comparison metrics,
│          detection (YOLO11n) + tracking (OC-SORT) + person-following,
│          5-regime shake evaluation, consistent-target methodology
│
├── gazebo_simulator/
│   ├── ros2_ws/src/kolestel_rover_description/
│   │   ├── worlds/user_saved_layout.sdf      # warehouse world
│   │   ├── launch/gazebo.launch.py           # Gazebo + robot launch
│   │   ├── launch/sim_cv_nav.launch.py       # full sim + CV nav
│   │   └── scripts/cv_navigator.py           # logical-map navigation node
│   ├── shared/warehouse_map.yaml             # logical map (nodes, lanes, stations)
│   ├── web_app/                              # web app + ROS 2 bridges
│   ├── build_ros2.sh                         # build the workspace
│   └── run_gazebo_cv_nav.sh                  # launch the full stack
│
└── README.md
```

## Stabilization & perception pipeline

The Colab notebook is self-contained and runnable end-to-end on a fresh Colab session.

Reproduces:
- six stabilization variants, all built on the same skeleton (feature stage → motion estimation → cleanup → smoothing → warpAffine), differing only in the feature stage and smoothing stage
- comparative metrics (residual std, cropping retention, ITF, Liu et al. stability score, ms/frame)
- V2 vs V4_pro deep-dive analysis (per-frame, cumulative trajectory, FFT spectrum, residual histogram)
- downstream YOLO11n + OC-SORT pipeline with target-locking and control signals
- five-regime shake evaluation (calm, mild, medium, strong realistic, white noise) with consistent-target identification via IoU matching to a reference bounding-box trajectory

### Quickstart

```bash
# in Google Colab:
# upload notebook + your test video, then run cells top-to-bottom
# the notebook installs ultralytics + boxmot + scipy automatically
```

The selected production stabilization (V2: LK + Kalman RTS, configuration A_R10) reduces target lock-loss events by **18%** on the deployment-relevant strong-realistic-shake regime.

## Gazebo simulator integration

The `gazebo_simulator/` directory contains the minimal set of files responsible for integrating the warehouse map with the Gazebo world (without ArUco markers; that part is intentionally excluded from this public release).

Key integration points:

- **Gazebo world**: `ros2_ws/src/kolestel_rover_description/worlds/user_saved_layout.sdf`
- **Logical warehouse map**: `shared/warehouse_map.yaml` (nodes, stations, lanes)
- **Robot pose projection into app coordinates**: `web_app/app/ros2_bridge/robot_status_bridge.py`
- **Navigation logic using the logical map**: `ros2_ws/src/kolestel_rover_description/scripts/cv_navigator.py`

### Build & run

```bash
cd gazebo_simulator
./build_ros2.sh
./run_gazebo_cv_nav.sh
```

Requires ROS 2 (Humble) + Gazebo + Nav2 + the `kolestel_rover_description` package.

## Hardware target

- **Platform**: GMKtec NucBox K10 mini PC (CPU only, no GPU)
- **Camera**: Intel RealSense D435 (RGB 1920×1080, FOV ~69°)
- **Stack**: OpenCV (stabilization), Ultralytics (YOLO11n), boxmot (OC-SORT), ROS 2 + Nav2 (navigation)

## Citation

> Korableva V. (2026). *Real-Time Video Stabilization for a Warehouse Mobile Robot: Effect on Downstream Person Perception.* Bachelor's thesis, HSE Faculty of Computer Science.
