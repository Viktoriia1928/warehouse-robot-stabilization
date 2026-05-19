# Warehouse-Robot Video Stabilization and Downstream Tracking

Bachelor thesis repository. Lightweight video stabilization for a warehouse mobile robot, evaluated by its effect on downstream person detection and tracking on the IndoorCrowd dataset.

## Project goal

Determine whether a lightweight, classical video stabilization method improves downstream person-following performance under realistic robot vibration. The stabilizer is intended for offline deployment on a CPU-only target platform (GMKtec NucBox K10).

## Selected stabilizer (V2)

The production configuration used throughout this repository:

- Shi–Tomasi corner detection (`goodFeaturesToTrack`)
- Pyramidal Lucas–Kanade optical flow with forward–backward consistency check
- RANSAC partial-affine motion fit between consecutive frames
- Median filter + MAD-based clipping on per-frame `(dx, dy, da)`
- Cumulative trajectory smoothed by a Kalman RTS smoother (constant-velocity model)
- Two-pass design, applied per frame via `warpAffine` with replicate borders

Default smoothing strengths used in the thesis: `q = (1e-3, 1e-3, 1e-5)`, `r = (1.0, 1.0, 1e-3)`, mode `rts`.

## Downstream evaluation design

The stabilizer is evaluated as a preprocessing step in front of a fixed detection + tracking stack. Detection is performed with YOLO11n (Ultralytics), and tracking with OC-SORT as the main tracker. BoT-SORT is included only as a secondary tracker-sensitivity check.

Two ground-truth target identities are selected from the IndoorCrowd recording:

- Primary target: `tid = 2`
- Secondary target: `tid = 10`

Four evaluation regimes are compared, identical detector and tracker across all of them:

1. Original
2. Original + V2 stabilization
3. Shaken (synthetic realistic shake injected into the original video)
4. Shaken + V2 stabilization

Per-target GT-based metrics:

- `lock_rate` — fraction of GT frames where the predicted track was matched
- `mean_iou` — mean IoU over matched frames
- `lost_segments` — number of `found → not found` transitions
- `longest_continuous_run` — longest streak of consecutive matches
- `id_switches` — switches of the assigned predicted track ID over matched frames
- `dominant_id_purity` — share of the most frequent predicted ID among all assigned IDs

The IoU matching threshold is 0.3 throughout.

## Repository layout

```
.
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── data/                 IndoorCrowd loader, recording-to-mp4 builder, GT extractor
│   ├── stabilization/        Features, cleanup, Kalman smoother, V2 pipeline
│   ├── shake/                Synthetic realistic shake injection
│   ├── tracking/             YOLO11n + OC-SORT main, YOLO11n + BoT-SORT secondary
│   └── evaluation/           IoU helpers, GT matching, per-target metrics
├── scripts/
│   ├── 01_prepare_dataset.py
│   ├── 02_stabilize_v2.py
│   ├── 03_inject_shake.py
│   ├── 04_run_tracker.py
│   ├── 05_evaluate_targets.py
│   └── 06_generate_figures.py
├── notebooks/
│   └── experimental/         Original Colab notebook (kept for transparency)
├── gazebo_simulator/         ROS 2 + Gazebo on-robot stack (warehouse with pedestrians)
│   ├── ros2_ws/              colcon workspace with the kolestel_rover_description package
│   ├── shared/               logical warehouse map
│   ├── build_ros2.sh
│   └── run_gazebo_tracking.sh
├── results/
│   ├── figures/              Bar charts produced by 06_generate_figures.py
│   ├── tables/               CSV tables produced by 05_evaluate_targets.py
│   └── examples/             Small placeholder for example outputs
└── docs/
    └── methodology.md
```

`gazebo_simulator/` contains the ROS 2 nodes that run the same V2 stabilization + YOLO11n + OC-SORT + person-follower stack on a simulated robot in a warehouse world with walking pedestrians. The offline downstream evaluation (the rest of this repository) feeds the same algorithms with recorded video instead. See `gazebo_simulator/README.md` for build / run instructions and the list of public assets that must be downloaded separately.

## How to run

Install dependencies into a clean environment (Python 3.10+):

```
pip install -r requirements.txt
```

End-to-end pipeline (the `EXAMPLE` paths below are placeholders):

```
# 1. Build the IndoorCrowd evaluation video and dump GT tracks
python -m scripts.01_prepare_dataset \
    --split acs_eg \
    --recording acs_eg_recording_1771861251_v2 \
    --output-video results/examples/indoorcrowd_raw.mp4 \
    --gt-json     results/examples/gt_tracks.json

# 2. Apply V2 stabilization to the raw video
python -m scripts.02_stabilize_v2 \
    --input  results/examples/indoorcrowd_raw.mp4 \
    --output results/examples/indoorcrowd_raw_stab.mp4

# 3. Inject realistic shake into the raw video
python -m scripts.03_inject_shake \
    --input  results/examples/indoorcrowd_raw.mp4 \
    --output results/examples/indoorcrowd_shake.mp4

# 4. Apply V2 stabilization to the shaken video
python -m scripts.02_stabilize_v2 \
    --input  results/examples/indoorcrowd_shake.mp4 \
    --output results/examples/indoorcrowd_shake_stab.mp4

# 5. Run OC-SORT on each of the four regimes
python -m scripts.04_run_tracker --tracker ocsort \
    --input  results/examples/indoorcrowd_raw.mp4 \
    --output results/examples/tracks_original_ocsort.mp4
python -m scripts.04_run_tracker --tracker ocsort \
    --input  results/examples/indoorcrowd_raw_stab.mp4 \
    --output results/examples/tracks_original_stab_ocsort.mp4
python -m scripts.04_run_tracker --tracker ocsort \
    --input  results/examples/indoorcrowd_shake.mp4 \
    --output results/examples/tracks_shake_ocsort.mp4
python -m scripts.04_run_tracker --tracker ocsort \
    --input  results/examples/indoorcrowd_shake_stab.mp4 \
    --output results/examples/tracks_shake_stab_ocsort.mp4

# 6. Compute GT-based metrics and save tables
python -m scripts.05_evaluate_targets \
    --gt-json     results/examples/gt_tracks.json \
    --tracks-dir  results/examples \
    --output-dir  results/tables

# 7. Render thesis bar charts
python -m scripts.06_generate_figures \
    --results-csv results/tables/ocsort_main_results.csv \
    --output-dir  results/figures
```

For the BoT-SORT sensitivity check, repeat step 5 with `--tracker botsort` and the `tracks_*_botsort.mp4` naming convention. `scripts/05_evaluate_targets.py` will pick the BoT-SORT files up automatically and write `results/tables/botsort_sensitivity_results.csv` and `results/tables/tracker_sensitivity_combined.csv`.

## Main outputs

After the full pipeline runs, the thesis-relevant artefacts live in:

- `results/tables/ocsort_main_results.csv` — main quantitative comparison (OC-SORT, four regimes × two targets, all six metrics)
- `results/tables/botsort_sensitivity_results.csv` — tracker-sensitivity check (BoT-SORT, same structure)
- `results/figures/ocsort_lost_segments.png` — bar chart of lost segments across regimes
- `results/figures/ocsort_lock_rate.png` — bar chart of lock rate across regimes
- `results/figures/ocsort_id_switches.png` — bar chart of ID switches across regimes

## Data note

The IndoorCrowd dataset is not bundled with this repository. It is downloaded on demand from the Hugging Face hub via `datasets.load_dataset("sebnae/IndoorCrowd", "obj_det_seg")` and requires an `HF_TOKEN`. Large intermediate artefacts (mp4 videos, per-frame `.npy` trajectories, generated PNGs) are excluded from the repository through `.gitignore`.

## Notebook archive

The original Colab notebook used during the experimental phase is kept under `notebooks/experimental/` for transparency. The repository code does not depend on it; it should be treated as a research workbench rather than a clean entry point.

## Citation

> Korableva V. (2026). *Real-Time Video Stabilization for a Warehouse Mobile Robot: Effect on Downstream Person Perception.* Bachelor's thesis, HSE Faculty of Computer Science.
