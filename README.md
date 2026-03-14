# Landmark Detection, Tracking, and Robust GraphSLAM

[![CI](https://github.com/<OWNER>/<REPO>/actions/workflows/ci.yml/badge.svg)](https://github.com/<OWNER>/<REPO>/actions/workflows/ci.yml)
[![CodeQL](https://github.com/<OWNER>/<REPO>/actions/workflows/codeql.yml/badge.svg)](https://github.com/<OWNER>/<REPO>/actions/workflows/codeql.yml)
[![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)](#)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE.md)

This repository started as an educational SLAM project and is now upgraded with a robust SLAM pipeline focused on a practical open-source gap:

- Baseline educational GraphSLAM examples usually assume clean measurements.
- Real-world robotics data contains outliers and inconsistent constraints.
- Lightweight, readable, and benchmarked robust GraphSLAM examples are still uncommon in beginner-friendly repos.

This update adds robust optimization and reproducible benchmarking while preserving the original learning scripts.

## What Was Added

- Robust 2D GraphSLAM with IRLS (iteratively reweighted least squares).
- Huber loss style downweighting for large residuals.
- Optional residual gating to reject gross outliers.
- Covariance-diagonal output for uncertainty introspection.
- Deterministic synthetic-data generator and outlier injector.
- Baseline-vs-robust benchmark script.
- Unit tests and multi-OS/multi-version CI.
- CodeQL security analysis workflow.

## Repository Layout

- Original educational scripts:
  - `1. Robot Moving and Sensing.py`
  - `2. Omega and Xi, Constraints.py`
  - `3. Landmark Detection and Tracking.py`
- Core robust module:
  - `slam_robust.py`
- Benchmark:
  - `benchmark_robust_slam.py`
- Tests:
  - `tests/test_robust_slam.py`
- GitHub automation:
  - `.github/workflows/ci.yml`
  - `.github/workflows/codeql.yml`

## Quick Start

```bash
python -m pip install -r requirements.txt
python -m pip install -e .
python -m unittest discover -s tests -v
python benchmark_robust_slam.py
python community_eval.py --trials 20 --outlier-rates 0.0 0.1 0.2 0.3 0.4 --output-dir artifacts
```

## Library Usage

Use this project as a Python library:

```python
from landmark_slam import RobustSlamConfig, robust_graph_slam, simulate_slam_data
```

Command-line interface:

```bash
landmark-slam benchmark
landmark-slam evaluate --trials 20 --outlier-rates 0.0 0.1 0.2 0.3 0.4
```

## Benchmark Snapshot

Using the included benchmark (`20` trials, `20%` injected measurement outliers):

- Pose RMSE (mean): baseline `4.963`, robust `1.148`
- Landmark RMSE (mean): baseline `8.993`, robust `7.626`

The robust solver significantly improves trajectory quality under outlier corruption.

## Community Evaluation Pipeline

To help contributors compare algorithm changes reliably, the project includes:

- A reproducible multi-rate evaluator: `community_eval.py`
- Machine-readable outputs: `artifacts/community_eval.json`
- Human-readable report: `artifacts/community_eval.md`
- Weekly benchmark workflow: `.github/workflows/benchmark.yml`

This enables regression tracking and objective review in pull requests.

## Community Standards

- Contribution guide: `CONTRIBUTING.md`
- Code of conduct: `CODE_OF_CONDUCT.md`
- Security policy: `SECURITY.md`
- Issue templates and PR template under `.github/`

## Why This Matters

For practical SLAM/CV systems, map quality degrades rapidly with bad associations and noisy detections.
Adding robust weighting and gating is often the minimum step from educational demos toward field-ready behavior.

## How To Enable Real Badge Links

Replace `<OWNER>/<REPO>` in the badge URLs with your GitHub namespace and repository name after pushing.

## Original Project Scope

The original project content and scripts remain available for the foundational GraphSLAM learning flow.

License: MIT (see `LICENSE.md`).
