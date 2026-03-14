import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import numpy as np

from landmark_slam import (
    RobustSlamConfig,
    baseline_graph_slam,
    get_poses_landmarks,
    inject_measurement_outliers,
    rmse_points,
    robust_graph_slam,
    simulate_slam_data,
)


class TestRobustGraphSlam(unittest.TestCase):
    def setUp(self):
        self.n_steps = 20
        self.num_landmarks = 6
        self.world_size = 100.0
        self.measurement_range = 50.0
        self.motion_noise = 2.0
        self.measurement_noise = 2.0
        self.distance = 10.0

    def test_output_shape(self):
        data, _, _ = simulate_slam_data(
            n_steps=self.n_steps,
            num_landmarks=self.num_landmarks,
            world_size=self.world_size,
            measurement_range=self.measurement_range,
            motion_noise=self.motion_noise,
            measurement_noise=self.measurement_noise,
            distance=self.distance,
            seed=11,
        )

        result = robust_graph_slam(
            data,
            n_steps=self.n_steps,
            num_landmarks=self.num_landmarks,
            config=RobustSlamConfig(
                world_size=self.world_size,
                motion_noise=self.motion_noise,
                measurement_noise=self.measurement_noise,
            ),
        )

        expected_rows = 2 * (self.n_steps + self.num_landmarks)
        self.assertEqual(result.mu.shape, (expected_rows, 1))
        self.assertEqual(result.covariance_diag.shape, (expected_rows, 1))
        self.assertTrue(np.all(np.isfinite(result.mu)))

    def test_robust_beats_baseline_under_outliers(self):
        data, true_poses, true_landmarks = simulate_slam_data(
            n_steps=self.n_steps,
            num_landmarks=self.num_landmarks,
            world_size=self.world_size,
            measurement_range=self.measurement_range,
            motion_noise=self.motion_noise,
            measurement_noise=self.measurement_noise,
            distance=self.distance,
            seed=42,
        )
        corrupted = inject_measurement_outliers(
            data,
            outlier_rate=0.3,
            outlier_scale=35.0,
            seed=2026,
        )

        base = baseline_graph_slam(
            corrupted,
            n_steps=self.n_steps,
            num_landmarks=self.num_landmarks,
            world_size=self.world_size,
            motion_noise=self.motion_noise,
            measurement_noise=self.measurement_noise,
        )
        robust = robust_graph_slam(
            corrupted,
            n_steps=self.n_steps,
            num_landmarks=self.num_landmarks,
            config=RobustSlamConfig(
                world_size=self.world_size,
                motion_noise=self.motion_noise,
                measurement_noise=self.measurement_noise,
                irls_iterations=8,
                huber_delta=2.0,
                gate_threshold=7.0,
                use_gating=True,
            ),
        )

        base_poses, base_landmarks = get_poses_landmarks(base.mu, self.n_steps, self.num_landmarks)
        robust_poses, robust_landmarks = get_poses_landmarks(robust.mu, self.n_steps, self.num_landmarks)

        base_error = rmse_points(base_poses, true_poses) + rmse_points(base_landmarks, true_landmarks)
        robust_error = rmse_points(robust_poses, true_poses) + rmse_points(robust_landmarks, true_landmarks)

        self.assertLess(robust_error, base_error)


if __name__ == "__main__":
    unittest.main()
