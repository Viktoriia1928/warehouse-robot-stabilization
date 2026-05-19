"""Run the full GT-based evaluation across the four regimes for OC-SORT (main) and BoT-SORT (sensitivity)."""
import argparse
import json
import os

import pandas as pd

from src.evaluation import evaluate_gt_target
from src.evaluation.metrics import tracks_log_to_pred_by_image_id


REGIMES_OCSORT = [
    ("orig_ocsort",       "tracks_original_ocsort"),
    ("orig_stab_ocsort",  "tracks_original_stab_ocsort"),
    ("shake_ocsort",      "tracks_shake_ocsort"),
    ("shake_stab_ocsort", "tracks_shake_stab_ocsort"),
]

REGIMES_BOTSORT = [
    ("orig_botsort",       "tracks_original_botsort"),
    ("orig_stab_botsort",  "tracks_original_stab_botsort"),
    ("shake_botsort",      "tracks_shake_botsort"),
    ("shake_stab_botsort", "tracks_shake_stab_botsort"),
]


def load_tracks_log(path):
    with open(path) as f:
        data = json.load(f)
    return data["frames"]


def evaluate_regime(label, tracks_file, gt_tracks, target_tids, iou_threshold):
    tracks_log = load_tracks_log(tracks_file)
    preds_by_image = tracks_log_to_pred_by_image_id(tracks_log)
    out = {}
    for tid in target_tids:
        out[(tid, label)] = evaluate_gt_target(
            gt_tracks[tid],
            preds_by_image,
            iou_threshold=iou_threshold,
        )
    return out


def to_dataframe(results, tracker_name):
    rows = []
    for (tid, pipeline), res in results.items():
        rows.append({
            "tracker": tracker_name,
            "target_tid": tid,
            "pipeline": pipeline,
            "gt_frames": res["total_gt_frames"],
            "matched_frames": res["matched_frames"],
            "lock_rate": round(res["lock_rate"], 4),
            "mean_iou": round(res["mean_iou"], 4),
            "lost_segments": res["lost_segments"],
            "longest_run": res["longest_continuous_run"],
            "id_switches": res["id_switches"],
            "unique_pred_ids": res["unique_assigned_pred_ids"],
            "dominant_pred_id": res["dominant_pred_id"],
            "dominant_id_purity": round(res["dominant_id_purity"], 4),
        })
    return pd.DataFrame(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt-json", required=True)
    parser.add_argument("--tracks-dir", required=True,
                        help="Folder containing tracks_*.json files for each regime")
    parser.add_argument("--output-dir", default="results/tables")
    parser.add_argument("--targets", type=int, nargs="+", default=[2, 10])
    parser.add_argument("--iou-threshold", type=float, default=0.3)
    parser.add_argument("--skip-botsort", action="store_true")
    args = parser.parse_args()

    with open(args.gt_json) as f:
        raw_gt = json.load(f)
    gt_tracks = {
        int(tid): {int(k): v for k, v in frames.items()}
        for tid, frames in raw_gt.items()
    }

    results_ocsort = {}
    for label, stem in REGIMES_OCSORT:
        path = os.path.join(args.tracks_dir, stem + ".json")
        if os.path.exists(path):
            results_ocsort.update(
                evaluate_regime(label, path, gt_tracks, args.targets, args.iou_threshold)
            )
        else:
            print(f"missing: {path}")

    os.makedirs(args.output_dir, exist_ok=True)
    df_ocsort = to_dataframe(results_ocsort, "OC-SORT")
    df_ocsort.sort_values(["target_tid", "pipeline"], inplace=True)
    df_ocsort.to_csv(os.path.join(args.output_dir, "ocsort_main_results.csv"), index=False)
    print(df_ocsort.to_string(index=False))

    if not args.skip_botsort:
        results_botsort = {}
        for label, stem in REGIMES_BOTSORT:
            path = os.path.join(args.tracks_dir, stem + ".json")
            if os.path.exists(path):
                results_botsort.update(
                    evaluate_regime(label, path, gt_tracks, args.targets, args.iou_threshold)
                )
        if results_botsort:
            df_botsort = to_dataframe(results_botsort, "BoT-SORT")
            df_botsort.sort_values(["target_tid", "pipeline"], inplace=True)
            df_botsort.to_csv(
                os.path.join(args.output_dir, "botsort_sensitivity_results.csv"),
                index=False,
            )
            df_combined = pd.concat([df_ocsort, df_botsort], ignore_index=True)
            df_combined.to_csv(
                os.path.join(args.output_dir, "tracker_sensitivity_combined.csv"),
                index=False,
            )

    print(f"\ntables saved to: {args.output_dir}")


if __name__ == "__main__":
    main()
