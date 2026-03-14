from __future__ import annotations

from math import cos, pi, sin
import random
from typing import List, Sequence, Tuple

import numpy as np

from .robot import Robot
from .types import Measurement, SlamData


def get_poses_landmarks(mu: np.ndarray, n_steps: int, num_landmarks: int) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    poses = [(mu[2 * i, 0].item(), mu[2 * i + 1, 0].item()) for i in range(n_steps)]
    landmarks = [(mu[2 * (n_steps + i), 0].item(), mu[2 * (n_steps + i) + 1, 0].item()) for i in range(num_landmarks)]
    return poses, landmarks


def rmse_points(estimated: Sequence[Tuple[float, float]], truth: Sequence[Tuple[float, float]]) -> float:
    e = np.array(estimated, dtype=float)
    t = np.array(truth, dtype=float)
    diff = e - t
    return float(np.sqrt(np.mean(np.sum(diff * diff, axis=1))))


def simulate_slam_data(
    n_steps: int,
    num_landmarks: int,
    world_size: float,
    measurement_range: float,
    motion_noise: float,
    measurement_noise: float,
    distance: float,
    seed: int = 0,
) -> Tuple[SlamData, List[Tuple[float, float]], List[Tuple[float, float]]]:
    random.seed(seed)
    np.random.seed(seed)

    r = Robot(world_size, measurement_range, motion_noise, measurement_noise)
    r.make_landmarks(num_landmarks)
    true_landmarks = [(float(x), float(y)) for x, y in r.landmarks]

    orientation = random.random() * 2.0 * pi
    dx = cos(orientation) * distance
    dy = sin(orientation) * distance

    true_poses = [(r.x, r.y)]
    data: SlamData = []

    for _ in range(n_steps - 1):
        z = r.sense()
        while not r.move(dx, dy):
            orientation = random.random() * 2.0 * pi
            dx = cos(orientation) * distance
            dy = sin(orientation) * distance

        data.append([z, [dx, dy]])
        true_poses.append((r.x, r.y))

    return data, true_poses, true_landmarks


def inject_measurement_outliers(data: SlamData, outlier_rate: float, outlier_scale: float, seed: int = 1) -> SlamData:
    random.seed(seed)
    noisy: SlamData = []
    for measurements, motion in data:
        new_measurements: List[Measurement] = []
        for landmark_id, dx, dy in measurements:
            if random.random() < outlier_rate:
                dx += random.uniform(-outlier_scale, outlier_scale)
                dy += random.uniform(-outlier_scale, outlier_scale)
            new_measurements.append([landmark_id, dx, dy])
        noisy.append([new_measurements, list(motion)])
    return noisy
