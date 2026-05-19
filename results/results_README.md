# Results

This folder contains selected visual and quantitative artifacts from the thesis project **“Real-Time Video Stabilization for a Warehouse Mobile Robot”**.

The materials are grouped into three parts:

## `examples/`
Illustrative examples of the pipeline output.

- `gazebo_world_simulator.png` — screenshot of the warehouse-like Gazebo environment used for simulator-based stabilization experiments.
- `v2_stabilization_sbs.mp4` — side-by-side comparison of the raw and stabilized video for the selected V2 method.
- `indoorcrowd_shaken_v2_sbs.mp4` — side-by-side comparison of the shaken IndoorCrowd sequence before and after stabilization with V2.
- `detection_following_demo_poster.jpg` — example frame showing downstream person detection/tracking behavior.

## `figures/`
Main figures used to support the experimental conclusions.

- `chart_comparison_4variants.png` — compact comparison of the strongest classical stabilization variants.
- `chart_v2_vs_v4pro_spectrum.png` — spectral comparison of V2 and V4_pro.
- `chart_v2_vs_v4pro_trajectory.png` — cumulative trajectory comparison of V2 and V4_pro.
- `fig_ocsort_lock_rate.png` — OC-SORT lock-rate comparison across the four evaluation regimes.
- `fig_ocsort_lost_segments.png` — OC-SORT lost-segment comparison across the four evaluation regimes.
- `fig_ocsort_id_switches.png` — OC-SORT ID-switch comparison across the four evaluation regimes.

## `tables/`
Tabular summaries of secondary comparisons.

- `tracker_sensitivity_table.png` — tracker sensitivity check comparing OC-SORT and BoT-SORT under shaken conditions.

## Notes
- The main stabilization method selected in the thesis is **V2**.
- The main downstream detector-tracker evaluation is based on **YOLO11n + OC-SORT**.
- BoT-SORT is included only as a secondary sensitivity check.
- This folder contains only selected artifacts, not the full set of intermediate experiment outputs.
