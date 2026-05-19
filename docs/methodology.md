# Methodology

## Selected stabilizer (V2)

V2 is the production stabilization configuration carried into the downstream evaluation.

Pipeline:

1. Convert each frame to grayscale.
2. Detect Shi–Tomasi corners on the previous frame (`maxCorners=500`, `qualityLevel=0.005`, `minDistance=12`, `blockSize=3`).
3. Track corners with pyramidal Lucas–Kanade (`winSize=(21, 21)`, `maxLevel=3`); reject points whose round-trip forward–backward error exceeds 1.0 px.
4. Fit a partial-affine transform with RANSAC (`ransacReprojThreshold=2.0`, `confidence=0.999`, `refineIters=15`).
5. Accumulate per-frame `(dx, dy, da)` over time.
6. Clean accumulated transforms with a 1D median filter (kernel 5) followed by symmetric MAD-based clipping at `k=4.0`.
7. Smooth the cumulative trajectory with a Kalman RTS smoother under a constant-velocity model. Default process and measurement noise: `q = (1e-3, 1e-3, 1e-5)`, `r = (1.0, 1.0, 1e-3)`.
8. Subtract the smoothed trajectory from the raw cumulative trajectory to obtain per-frame corrective transforms and apply them via `warpAffine` with `INTER_LINEAR` and `BORDER_REPLICATE`.

This is the only stabilizer used in the downstream evaluation. Earlier variants (moving-average smoothing, ORB front-end, robust-LK front-end, RAFT-based reference) are documented in the experimental notebook but are not part of the production code path.

## Synthetic shake

To probe the operating regime of V2, a deterministic shake-injection routine produces a "shaken" version of the original video. The shake is a sum of low-frequency sinusoids (2.5, 4.0, 7.0 Hz) per axis, plus low-amplitude Gaussian noise, plus occasional impulse bursts that decay geometrically across subsequent frames. Default amplitudes: translation `sin_amp_trans=10.0`, rotation `sin_amp_rot=0.015 rad`, burst magnitude 12.0, burst probability 0.015 per frame, seed 42.

## Detection and tracking

Detection: YOLO11n (Ultralytics), pretrained on COCO, person class only. Inference at `imgsz=960`, `conf=0.20`, `iou=0.45`.

Tracking:

- Main tracker: OC-SORT (motion-only, no appearance features). Used for every reported number in the thesis.
- Secondary tracker: BoT-SORT, included only to confirm that the direction of the V2 effect is not specific to OC-SORT. Reported separately as a sensitivity check.

The downstream stack is held fixed across all four evaluation regimes.

## Dataset and target selection

The downstream evaluation is performed on a single recording from the IndoorCrowd dataset (`obj_det_seg` configuration, split `acs_eg`, recording `acs_eg_recording_1771861251_v2`). Frames are rendered to MP4 at 10 fps.

Two GT identities are followed:

- Primary target: `tid = 2`
- Secondary target: `tid = 10`

Selection criterion: these two IDs provide complementary trajectories (one mostly visible, one with several occlusions) that exercise the metrics in different regimes.

## Evaluation regimes

Each of the two targets is evaluated under all four regimes:

1. Original
2. Original + V2 stabilization
3. Shaken
4. Shaken + V2 stabilization

The exact same detector and tracker are used in all four regimes; only the input video changes.

## GT-based metrics

For a target, let `F` be the ordered list of GT frames for that target. For each frame the highest-IoU prediction is matched against the GT box; a match requires IoU ≥ 0.3.

- `lock_rate` = matched_frames / |F|
- `mean_iou` = average IoU over matched frames
- `lost_segments` = number of `found → not found` transitions in the matched-frame sequence
- `longest_continuous_run` = longest streak of consecutive matched frames
- `id_switches` = number of changes of the matched predicted track ID across matched frames
- `dominant_id_purity` = (count of the most-frequent predicted ID across matched frames) / (number of matched frames)

These six metrics are reported per target and per regime. Tables go into `results/tables/`, figures into `results/figures/`.
