"""Inject realistic shake into a video."""
import argparse

from src.shake import add_realistic_shake


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--trans-amp", type=float, default=10.0)
    parser.add_argument("--rot-amp", type=float, default=0.015)
    parser.add_argument("--freqs", type=float, nargs="+", default=(2.5, 4.0, 7.0))
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    add_realistic_shake(
        input_video=args.input,
        output_video=args.output,
        sin_amp_trans=args.trans_amp,
        sin_amp_rot=args.rot_amp,
        freqs=tuple(args.freqs),
        seed=args.seed,
    )
    print(f"shaken video: {args.output}")


if __name__ == "__main__":
    main()
