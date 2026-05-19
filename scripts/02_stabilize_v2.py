"""Apply V2 stabilization (LK + Kalman RTS) to a video."""
import argparse
import os

from src.stabilization import stabilize_v2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--side-by-side", default=None)
    parser.add_argument("--q", type=float, nargs=3, default=(1e-3, 1e-3, 1e-5))
    parser.add_argument("--r", type=float, nargs=3, default=(1.0, 1.0, 1e-3))
    parser.add_argument("--mode", default="rts", choices=["rts", "filter"])
    args = parser.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
    stabilize_v2(
        input_path=args.input,
        output_path=args.output,
        side_by_side_path=args.side_by_side,
        q=tuple(args.q),
        r=tuple(args.r),
        kalman_mode=args.mode,
    )


if __name__ == "__main__":
    main()
