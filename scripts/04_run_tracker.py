"""Run a tracker (OC-SORT or BoT-SORT) on a video and dump per-frame tracks JSON."""
import argparse

from src.tracking import detect_and_track_ocsort, detect_and_track_botsort


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--tracker", choices=["ocsort", "botsort"], default="ocsort")
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.45)
    args = parser.parse_args()

    if args.tracker == "ocsort":
        detect_and_track_ocsort(
            input_video=args.input,
            output_video=args.output,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
        )
    else:
        detect_and_track_botsort(
            input_video=args.input,
            output_video=args.output,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
        )


if __name__ == "__main__":
    main()
