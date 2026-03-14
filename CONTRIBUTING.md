# Contributing

Thanks for your interest in improving this SLAM project.

## Development Setup

1. Install dependencies:

   python -m pip install -r requirements.txt

2. Run tests:

   python -m unittest discover -s tests -v

3. Run benchmark:

   python benchmark_robust_slam.py

## What We Need Most

- Better robust data association ideas for landmarks
- More stress-test scenarios (high outliers, low visibility, loop-closure patterns)
- Better uncertainty metrics and calibration checks
- Performance improvements with the same numerical behavior

## Pull Request Rules

- Keep PRs focused and small.
- Add or update tests for behavioral changes.
- Add benchmark notes for algorithmic changes.
- Update README docs when adding user-facing functionality.

## Coding Guidelines

- Prefer readable and deterministic code.
- Avoid hidden randomness in tests; use fixed seeds.
- Keep educational scripts intact unless a PR explicitly modernizes them.

## Suggested PR Checklist

- [ ] Tests pass locally
- [ ] Benchmark was run for algorithm changes
- [ ] Docs updated
- [ ] No unrelated file changes
