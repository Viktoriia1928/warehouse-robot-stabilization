"""Build the IndoorCrowd evaluation video and dump GT tracks for the two targets."""
import argparse
import json
import os

from src.data import (
    load_indoorcrowd,
    build_video_from_split,
    extract_gt_tracks,
)
from src.data.indoorcrowd import select_recording


PRIMARY_TID = 2
SECONDARY_TID = 10


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--split", default="acs_eg")
    parser.add_argument("--recording", default="acs_eg_recording_1771861251_v2")
    parser.add_argument("--output-video", default="results/examples/indoorcrowd_raw.mp4")
    parser.add_argument("--gt-json", default="results/examples/gt_tracks.json")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--hf-token", default=None)
    args = parser.parse_args()

    ds = load_indoorcrowd(hf_token=args.hf_token)
    subset = select_recording(ds, args.split, args.recording)
    print(f"frames in {args.recording}: {len(subset)}")

    os.makedirs(os.path.dirname(args.output_video) or ".", exist_ok=True)
    path, (w, h) = build_video_from_split(subset, args.output_video, fps=args.fps)
    print(f"video: {path}  {w}x{h}")

    gt_tracks = extract_gt_tracks(subset, [PRIMARY_TID, SECONDARY_TID])
    for tid, frames in gt_tracks.items():
        print(f"GT tid={tid}: {len(frames)} frames")

    serialisable = {str(tid): {str(k): list(v) for k, v in frames.items()}
                    for tid, frames in gt_tracks.items()}
    with open(args.gt_json, "w") as f:
        json.dump(serialisable, f, indent=2)
    print(f"GT saved: {args.gt_json}")


if __name__ == "__main__":
    main()
