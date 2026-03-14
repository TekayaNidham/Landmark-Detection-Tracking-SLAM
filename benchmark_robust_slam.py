"""Backward-compatible benchmark entrypoint."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from landmark_slam import BenchmarkConfig, run_benchmark


def main() -> None:
    cfg = BenchmarkConfig()
    summary = run_benchmark(cfg)

    print("Robust GraphSLAM Benchmark")
    print("==========================")
    print(f"Trials: {cfg.trials}")
    print(f"Outlier rate: {cfg.outlier_rate:.2f}")
    print(f"Outlier scale: {cfg.outlier_scale:.1f}")
    print()
    print("Pose RMSE")
    print(f"  Baseline mean +/- std: {summary['pose_baseline_mean']:.3f} +/- {summary['pose_baseline_std']:.3f}")
    print(f"  Robust   mean +/- std: {summary['pose_robust_mean']:.3f} +/- {summary['pose_robust_std']:.3f}")
    print()
    print("Landmark RMSE")
    print(f"  Baseline mean +/- std: {summary['landmark_baseline_mean']:.3f} +/- {summary['landmark_baseline_std']:.3f}")
    print(f"  Robust   mean +/- std: {summary['landmark_robust_mean']:.3f} +/- {summary['landmark_robust_std']:.3f}")


if __name__ == "__main__":
    main()
