"""Backward-compatible community evaluation entrypoint."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from landmark_slam import EvalConfig, run_eval, write_eval_outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run robust SLAM community evaluation")
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--outlier-scale", type=float, default=25.0)
    parser.add_argument("--outlier-rates", type=float, nargs="+", default=[0.0, 0.1, 0.2, 0.3, 0.4])
    parser.add_argument("--output-dir", type=str, default="artifacts")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = run_eval(
        EvalConfig(
            trials=args.trials,
            outlier_scale=args.outlier_scale,
            outlier_rates=args.outlier_rates,
        )
    )
    json_path, md_path = write_eval_outputs(result, Path(args.output_dir))

    print("Community evaluation complete")
    print(f"JSON report: {json_path}")
    print(f"Markdown report: {md_path}")


if __name__ == "__main__":
    main()
