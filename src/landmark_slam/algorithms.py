from __future__ import annotations

import numpy as np

from .models import RobustSlamConfig, SlamResult
from .types import SlamData


def initialize_constraints(n_steps: int, num_landmarks: int, world_size: float):
    dim = 2 * (n_steps + num_landmarks)
    omega = np.zeros((dim, dim), dtype=float)
    xi = np.zeros((dim, 1), dtype=float)
    omega[0, 0] = 1.0
    omega[1, 1] = 1.0
    xi[0, 0] = world_size / 2.0
    xi[1, 0] = world_size / 2.0
    return omega, xi


def _huber_weight(residual_norm: float, delta: float) -> float:
    if residual_norm <= delta:
        return 1.0
    if residual_norm == 0.0:
        return 1.0
    return delta / residual_norm


def _pose_indices(t: int):
    px = 2 * t
    return px, px + 1


def _landmark_indices(landmark_id: int, n_steps: int):
    lx = 2 * (n_steps + landmark_id)
    return lx, lx + 1


def _solve_system(omega: np.ndarray, xi: np.ndarray) -> np.ndarray:
    try:
        return np.linalg.solve(omega, xi)
    except np.linalg.LinAlgError:
        damping = 1e-8
        return np.linalg.solve(omega + damping * np.eye(omega.shape[0]), xi)


def _measurement_residual(mu: np.ndarray, px: int, py: int, lx: int, ly: int, zdx: float, zdy: float) -> float:
    rx = (mu[lx, 0] - mu[px, 0]) - zdx
    ry = (mu[ly, 0] - mu[py, 0]) - zdy
    return float(np.sqrt(rx * rx + ry * ry))


def _motion_residual(mu: np.ndarray, px: int, py: int, pnx: int, pny: int, udx: float, udy: float) -> float:
    rx = (mu[pnx, 0] - mu[px, 0]) - udx
    ry = (mu[pny, 0] - mu[py, 0]) - udy
    return float(np.sqrt(rx * rx + ry * ry))


def baseline_graph_slam(
    data: SlamData,
    n_steps: int,
    num_landmarks: int,
    world_size: float,
    motion_noise: float,
    measurement_noise: float,
) -> SlamResult:
    omega, xi = initialize_constraints(n_steps, num_landmarks, world_size)
    meas_weight = 1.0 / measurement_noise
    mot_weight = 1.0 / motion_noise

    for t, (measurements, motion) in enumerate(data):
        px, py = _pose_indices(t)
        pnx, pny = _pose_indices(t + 1)

        for landmark_id, dx, dy in measurements:
            lx, ly = _landmark_indices(int(landmark_id), n_steps)

            omega[px, px] += meas_weight
            omega[px, lx] -= meas_weight
            omega[lx, px] -= meas_weight
            omega[lx, lx] += meas_weight

            omega[py, py] += meas_weight
            omega[py, ly] -= meas_weight
            omega[ly, py] -= meas_weight
            omega[ly, ly] += meas_weight

            xi[px, 0] -= dx * meas_weight
            xi[lx, 0] += dx * meas_weight
            xi[py, 0] -= dy * meas_weight
            xi[ly, 0] += dy * meas_weight

        udx, udy = motion
        omega[px, px] += mot_weight
        omega[px, pnx] -= mot_weight
        omega[pnx, px] -= mot_weight
        omega[pnx, pnx] += mot_weight

        omega[py, py] += mot_weight
        omega[py, pny] -= mot_weight
        omega[pny, py] -= mot_weight
        omega[pny, pny] += mot_weight

        xi[px, 0] -= udx * mot_weight
        xi[pnx, 0] += udx * mot_weight
        xi[py, 0] -= udy * mot_weight
        xi[pny, 0] += udy * mot_weight

    mu = _solve_system(omega, xi)
    cov_diag = np.diag(np.linalg.pinv(omega)).reshape(-1, 1)
    return SlamResult(mu=mu, covariance_diag=cov_diag)


def robust_graph_slam(data: SlamData, n_steps: int, num_landmarks: int, config: RobustSlamConfig) -> SlamResult:
    mu = baseline_graph_slam(
        data,
        n_steps,
        num_landmarks,
        config.world_size,
        config.motion_noise,
        config.measurement_noise,
    ).mu

    meas_base_weight = 1.0 / config.measurement_noise
    mot_base_weight = 1.0 / config.motion_noise

    for _ in range(config.irls_iterations):
        omega, xi = initialize_constraints(n_steps, num_landmarks, config.world_size)

        for t, (measurements, motion) in enumerate(data):
            px, py = _pose_indices(t)
            pnx, pny = _pose_indices(t + 1)

            for landmark_id, dx, dy in measurements:
                lx, ly = _landmark_indices(int(landmark_id), n_steps)
                residual = _measurement_residual(mu, px, py, lx, ly, dx, dy)
                robust = _huber_weight(residual, config.huber_delta)
                if config.use_gating and residual > config.gate_threshold:
                    robust = 0.0

                w = meas_base_weight * robust
                if w == 0.0:
                    continue

                omega[px, px] += w
                omega[px, lx] -= w
                omega[lx, px] -= w
                omega[lx, lx] += w

                omega[py, py] += w
                omega[py, ly] -= w
                omega[ly, py] -= w
                omega[ly, ly] += w

                xi[px, 0] -= dx * w
                xi[lx, 0] += dx * w
                xi[py, 0] -= dy * w
                xi[ly, 0] += dy * w

            udx, udy = motion
            motion_residual = _motion_residual(mu, px, py, pnx, pny, udx, udy)
            motion_robust = _huber_weight(motion_residual, config.huber_delta)
            w = mot_base_weight * motion_robust

            omega[px, px] += w
            omega[px, pnx] -= w
            omega[pnx, px] -= w
            omega[pnx, pnx] += w

            omega[py, py] += w
            omega[py, pny] -= w
            omega[pny, py] -= w
            omega[pny, pny] += w

            xi[px, 0] -= udx * w
            xi[pnx, 0] += udx * w
            xi[py, 0] -= udy * w
            xi[pny, 0] += udy * w

        mu = _solve_system(omega, xi)

    cov_diag = np.diag(np.linalg.pinv(omega)).reshape(-1, 1)
    return SlamResult(mu=mu, covariance_diag=cov_diag)
