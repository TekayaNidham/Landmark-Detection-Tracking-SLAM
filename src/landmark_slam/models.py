from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class RobustSlamConfig:
    world_size: float
    motion_noise: float
    measurement_noise: float
    irls_iterations: int = 6
    huber_delta: float = 2.0
    gate_threshold: float = 6.0
    use_gating: bool = True


@dataclass
class SlamResult:
    mu: np.ndarray
    covariance_diag: np.ndarray
