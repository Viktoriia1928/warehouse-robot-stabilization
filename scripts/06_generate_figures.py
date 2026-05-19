"""Render the three OC-SORT bar charts (lost segments, lock rate, id switches) used in the thesis."""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PIPELINES = ["orig_ocsort", "orig_stab_ocsort", "shake_ocsort", "shake_stab_ocsort"]
LABELS = ["Original", "Original + V2", "Shaken", "Shaken + V2"]


def bar_plot(df, metric, title, ylabel, targets, output_path):
    x = np.arange(len(PIPELINES))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    for i, tid in enumerate(targets):
        sub = df[df["target_tid"] == tid].set_index("pipeline")
        vals = [sub.loc[p, metric] if p in sub.index else 0 for p in PIPELINES]
        offset = (i - (len(targets) - 1) / 2) * width
        ax.bar(x + offset, vals, width, label=f"Target {tid}")

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.set_xticklabels(LABELS, rotation=15)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"saved: {output_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-csv", default="results/tables/ocsort_main_results.csv")
    parser.add_argument("--output-dir", default="results/figures")
    parser.add_argument("--targets", type=int, nargs="+", default=[2, 10])
    args = parser.parse_args()

    df = pd.read_csv(args.results_csv)
    os.makedirs(args.output_dir, exist_ok=True)

    bar_plot(
        df, "lost_segments",
        "OC-SORT: lost segments across four evaluation regimes",
        "Lost segments",
        args.targets,
        os.path.join(args.output_dir, "ocsort_lost_segments.png"),
    )
    bar_plot(
        df, "lock_rate",
        "OC-SORT: lock rate across four evaluation regimes",
        "Lock rate",
        args.targets,
        os.path.join(args.output_dir, "ocsort_lock_rate.png"),
    )
    bar_plot(
        df, "id_switches",
        "OC-SORT: ID switches across four evaluation regimes",
        "ID switches",
        args.targets,
        os.path.join(args.output_dir, "ocsort_id_switches.png"),
    )


if __name__ == "__main__":
    main()
