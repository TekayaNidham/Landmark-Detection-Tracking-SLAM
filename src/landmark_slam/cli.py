from __future__ import annotations

import argparse
from pathlib import Path

from .evaluation import BenchmarkConfig, EvalConfig, run_benchmark, run_eval, write_eval_outputs


def _benchmark_cmd(args: argparse.Namespace) -> None:
    summary = run_benchmark(
        BenchmarkConfig(
            trials=args.trials,
            n_steps=args.n_steps,
            num_landmarks=args.num_landmarks,
            world_size=args.world_size,
            measurement_range=args.measurement_range,
            motion_noise=args.motion_noise,
            measurement_noise=args.measurement_noise,
            distance=args.distance,
            outlier_rate=args.outlier_rate,
            outlier_scale=args.outlier_scale,
        )
    )

    print("Robust GraphSLAM Benchmark")
    print("==========================")
    print(f"Trials: {args.trials}")
    print(f"Outlier rate: {args.outlier_rate:.2f}")
    print(f"Outlier scale: {args.outlier_scale:.1f}")
    print()
    print("Pose RMSE")
    print(f"  Baseline mean +/- std: {summary['pose_baseline_mean']:.3f} +/- {summary['pose_baseline_std']:.3f}")
    print(f"  Robust   mean +/- std: {summary['pose_robust_mean']:.3f} +/- {summary['pose_robust_std']:.3f}")
    print()
    print("Landmark RMSE")
    print(f"  Baseline mean +/- std: {summary['landmark_baseline_mean']:.3f} +/- {summary['landmark_baseline_std']:.3f}")
    print(f"  Robust   mean +/- std: {summary['landmark_robust_mean']:.3f} +/- {summary['landmark_robust_std']:.3f}")


def _eval_cmd(args: argparse.Namespace) -> None:
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="landmark-slam", description="Landmark SLAM tooling")
    sub = parser.add_subparsers(dest="command", required=True)

    p_bench = sub.add_parser("benchmark", help="Run robust-vs-baseline benchmark")
    p_bench.add_argument("--trials", type=int, default=20)
    p_bench.add_argument("--n-steps", type=int, default=25)
    p_bench.add_argument("--num-landmarks", type=int, default=8)
    p_bench.add_argument("--world-size", type=float, default=100.0)
    p_bench.add_argument("--measurement-range", type=float, default=45.0)
    p_bench.add_argument("--motion-noise", type=float, default=2.0)
    p_bench.add_argument("--measurement-noise", type=float, default=2.0)
    p_bench.add_argument("--distance", type=float, default=12.0)
    p_bench.add_argument("--outlier-rate", type=float, default=0.2)
    p_bench.add_argument("--outlier-scale", type=float, default=25.0)
    p_bench.set_defaults(func=_benchmark_cmd)

    p_eval = sub.add_parser("evaluate", help="Run multi-rate evaluation and write artifacts")
    p_eval.add_argument("--trials", type=int, default=20)
    p_eval.add_argument("--outlier-scale", type=float, default=25.0)
    p_eval.add_argument("--outlier-rates", type=float, nargs="+", default=[0.0, 0.1, 0.2, 0.3, 0.4])
    p_eval.add_argument("--output-dir", type=str, default="artifacts")
    p_eval.set_defaults(func=_eval_cmd)

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
