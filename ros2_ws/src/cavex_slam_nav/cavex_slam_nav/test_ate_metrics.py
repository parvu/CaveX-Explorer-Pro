"""
test_ate_metrics.py

Unit tests for ate_metrics.py. Run with:
    python3 -m pytest test_ate_metrics.py -v
or directly:
    python3 test_ate_metrics.py
"""

import numpy as np
from ate_metrics import compute_ate, umeyama_alignment, apply_transform, summarize_multi_run


def test_perfect_match_zero_error():
    gt = np.random.default_rng(1).normal(size=(50, 3))
    result = compute_ate(gt.copy(), gt, align=True)
    assert result["rmse"] < 1e-8, "identical trajectories should have ~zero ATE"


def test_rotation_translation_invariance():
    """ATE after alignment should be insensitive to an arbitrary rigid
    transform applied to the estimated trajectory -- this is the whole
    point of aligning before scoring."""
    rng = np.random.default_rng(2)
    gt = rng.normal(size=(100, 3)) * 2.0
    noise = rng.normal(0, 0.02, size=gt.shape)
    est_noisy = gt + noise

    theta = np.radians(73)
    R = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    t = np.array([12.0, -7.5, 3.2])
    est_transformed = (R @ est_noisy.T).T + t

    r1 = compute_ate(est_noisy, gt, align=True)
    r2 = compute_ate(est_transformed, gt, align=True)

    assert abs(r1["rmse"] - r2["rmse"]) < 1e-6, \
        "ATE should be identical regardless of arbitrary rigid transform on the estimate"


def test_unaligned_shows_frame_offset():
    gt = np.zeros((10, 3))
    est = np.ones((10, 3)) * 5.0  # constant 5m offset in every axis
    result = compute_ate(est, gt, align=False)
    expected = np.linalg.norm([5, 5, 5])
    assert abs(result["rmse"] - expected) < 1e-9


def test_summarize_multi_run():
    values = [0.10, 0.12, 0.11, 0.09, 0.13, 0.10, 0.11, 0.12, 0.10, 0.11]
    summary = summarize_multi_run(values)
    assert summary["n_runs"] == 10
    assert abs(summary["mean"] - np.mean(values)) < 1e-9
    assert abs(summary["std"] - np.std(values)) < 1e-9


def test_umeyama_recovers_known_transform():
    rng = np.random.default_rng(3)
    src = rng.normal(size=(30, 3))
    theta = np.radians(41)
    R_true = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    t_true = np.array([1.0, 2.0, -3.0])
    tgt = (R_true @ src.T).T + t_true

    R_est, t_est = umeyama_alignment(src, tgt)
    recovered = apply_transform(src, R_est, t_est)
    assert np.allclose(recovered, tgt, atol=1e-9)


if __name__ == "__main__":
    tests = [
        test_perfect_match_zero_error,
        test_rotation_translation_invariance,
        test_unaligned_shows_frame_offset,
        test_summarize_multi_run,
        test_umeyama_recovers_known_transform,
    ]
    for t in tests:
        t()
        print(f"PASS: {t.__name__}")
    print(f"\nAll {len(tests)} tests passed.")
