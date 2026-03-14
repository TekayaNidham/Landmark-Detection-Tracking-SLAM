from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import UTC, datetime
import json
from pathlib import Path
from statistics import mean, pstdev
from typing import List

from .algorithms import baseline_graph_slam, robust_graph_slam
from .models import RobustSlamConfig
from .simulation import get_poses_landmarks, inject_measurement_outliers, rmse_points, simulate_slam_data


@dataclass
class BenchmarkConfig:
    trials: int = 20
    n_steps: int = 25
    num_landmarks: int = 8
    world_size: float = 100.0
    measurement_range: float = 45.0
    motion_noise: float = 2.0
    measurement_noise: float = 2.0
    distance: float = 12.0
    outlier_rate: float = 0.2
    outlier_scale: float = 25.0


@dataclass
class EvalConfig:
    trials: int = 20
    n_steps: int = 25
    num_landmarks: int = 8
    world_size: float = 100.0
    measurement_range: float = 45.0
    motion_noise: float = 2.0
    measurement_noise: float = 2.0
    distance: float = 12.0
    outlier_scale: float = 25.0
    outlier_rates: List[float] | None = None


def run_benchmark(cfg: BenchmarkConfig) -> dict:
    baseline_pose_errors = []
    robust_pose_errors = []
    baseline_landmark_errors = []
    robust_landmark_errors = []

    for trial in range(cfg.trials):
        data, true_poses, true_landmarks = simulate_slam_data(
            n_steps=cfg.n_steps,
            num_landmarks=cfg.num_landmarks,
            world_size=cfg.world_size,
            measurement_range=cfg.measurement_range,
            motion_noise=cfg.motion_noise,
            measurement_noise=cfg.measurement_noise,
            distance=cfg.distance,
            seed=trial,
        )
        corrupted = inject_measurement_outliers(data, cfg.outlier_rate, cfg.outlier_scale, seed=1000 + trial)

        base = baseline_graph_slam(
            corrupted,
            n_steps=cfg.n_steps,
            num_landmarks=cfg.num_landmarks,
            world_size=cfg.world_size,
            motion_noise=cfg.motion_noise,
            measurement_noise=cfg.measurement_noise,
        )
        robust = robust_graph_slam(
            corrupted,
            n_steps=cfg.n_steps,
            num_landmarks=cfg.num_landmarks,
            config=RobustSlamConfig(
                world_size=cfg.world_size,
                motion_noise=cfg.motion_noise,
                measurement_noise=cfg.measurement_noise,
                irls_iterations=7,
                huber_delta=2.0,
                gate_threshold=7.0,
                use_gating=True,
            ),
        )

        base_poses, base_landmarks = get_poses_landmarks(base.mu, cfg.n_steps, cfg.num_landmarks)
        robust_poses, robust_landmarks = get_poses_landmarks(robust.mu, cfg.n_steps, cfg.num_landmarks)

        baseline_pose_errors.append(rmse_points(base_poses, true_poses))
        robust_pose_errors.append(rmse_points(robust_poses, true_poses))
        baseline_landmark_errors.append(rmse_points(base_landmarks, true_landmarks))
        robust_landmark_errors.append(rmse_points(robust_landmarks, true_landmarks))

    return {
        "pose_baseline_mean": mean(baseline_pose_errors),
        "pose_baseline_std": pstdev(baseline_pose_errors),
        "pose_robust_mean": mean(robust_pose_errors),
        "pose_robust_std": pstdev(robust_pose_errors),
        "landmark_baseline_mean": mean(baseline_landmark_errors),
        "landmark_baseline_std": pstdev(baseline_landmark_errors),
        "landmark_robust_mean": mean(robust_landmark_errors),
        "landmark_robust_std": pstdev(robust_landmark_errors),
    }


def run_eval(cfg: EvalConfig) -> dict:
    if cfg.outlier_rates is None:
        cfg.outlier_rates = [0.0, 0.1, 0.2, 0.3, 0.4]

    series = []
    for outlier_rate in cfg.outlier_rates:
        summary = run_benchmark(
            BenchmarkConfig(
                trials=cfg.trials,
                n_steps=cfg.n_steps,
                num_landmarks=cfg.num_landmarks,
                world_size=cfg.world_size,
                measurement_range=cfg.measurement_range,
                motion_noise=cfg.motion_noise,
                measurement_noise=cfg.measurement_noise,
                distance=cfg.distance,
                outlier_rate=outlier_rate,
                outlier_scale=cfg.outlier_scale,
            )
        )

        gain = 100.0 * (summary["pose_baseline_mean"] - summary["pose_robust_mean"]) / max(summary["pose_baseline_mean"], 1e-9)
        summary["outlier_rate"] = outlier_rate
        summary["pose_gain_pct"] = gain
        series.append(summary)

    return {
        "generated_at": datetime.now(UTC).isoformat(),
        "config": asdict(cfg),
        "series": series,
    }


def write_eval_outputs(result: dict, output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "community_eval.json"
    md_path = output_dir / "community_eval.md"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)

    lines = [
        "# Community Evaluation Report",
        "",
        f"Generated: {result['generated_at']}",
        "",
        "| Outlier Rate | Pose RMSE Baseline | Pose RMSE Robust | Pose Gain % |",
        "|---:|---:|---:|---:|",
    ]

    for row in result["series"]:
        lines.append(
            f"| {row['outlier_rate']:.2f} | {row['pose_baseline_mean']:.3f} | {row['pose_robust_mean']:.3f} | {row['pose_gain_pct']:.1f}% |"
        )

    lines.append("")
    lines.append("Lower RMSE is better. Positive gain means robust outperforms baseline.")

    with md_path.open("w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    return json_path, md_path
