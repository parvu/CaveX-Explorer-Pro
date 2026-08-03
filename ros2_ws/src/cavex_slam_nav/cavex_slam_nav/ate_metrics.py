"""
ate_metrics.py

Pure trajectory-evaluation math for CAVE-SLAM / SIC-SLAM, with no ROS2
dependency, so it can be unit-tested standalone (see the __main__ block
and test_ate_metrics.py) before being wrapped in a ROS2 node.

Implements Absolute Trajectory Error (ATE) with Umeyama alignment
(rotation + translation, no scale, since both trajectories are metric),
following the same convention as the widely-used `evo` trajectory
evaluation toolkit and the TUM RGB-D benchmark. This matters because
CAVE-SLAM's estimated trajectory and the basin's acoustic-beacon ground
truth will not, in general, share a coordinate frame origin/orientation
even if both are individually accurate -- ATE must be computed after
best-fit rigid alignment, not raw coordinate subtraction.
"""

import numpy as np


def umeyama_alignment(source: np.ndarray, target: np.ndarray):
    """
    Compute the rigid transform (R, t) that best aligns `source` onto
    `target` in a least-squares sense (Umeyama, 1991), no scaling.

    source, target: (N, 3) arrays of corresponding 3D points (e.g. estimated
    trajectory positions and ground-truth positions at matched timestamps).

    Returns (R, t) such that target ~= (R @ source.T).T + t
    """
    assert source.shape == target.shape
    n, dim = source.shape

    mu_src = source.mean(axis=0)
    mu_tgt = target.mean(axis=0)

    src_c = source - mu_src
    tgt_c = target - mu_tgt

    cov = (tgt_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1

    R = U @ S @ Vt
    t = mu_tgt - R @ mu_src
    return R, t


def apply_transform(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (R @ points.T).T + t


def compute_ate(estimated: np.ndarray, ground_truth: np.ndarray, align: bool = True):
    """
    Compute Absolute Trajectory Error between two (N, 3) position arrays
    at matched timestamps.

    Returns a dict with rmse, mean, median, std, min, max (all in the
    same length units as the input, typically metres), and the aligned
    estimated trajectory (useful for plotting/inspection).
    """
    estimated = np.asarray(estimated, dtype=float)
    ground_truth = np.asarray(ground_truth, dtype=float)
    assert estimated.shape == ground_truth.shape, "trajectories must have matched-length, corresponding samples"
    assert estimated.shape[1] == 3, "expected (N, 3) position arrays"

    if align and len(estimated) >= 3:
        R, t = umeyama_alignment(estimated, ground_truth)
        aligned = apply_transform(estimated, R, t)
    else:
        aligned = estimated
        R, t = np.eye(3), np.zeros(3)

    errors = np.linalg.norm(aligned - ground_truth, axis=1)

    return {
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "mean": float(np.mean(errors)),
        "median": float(np.median(errors)),
        "std": float(np.std(errors)),
        "min": float(np.min(errors)),
        "max": float(np.max(errors)),
        "n_samples": int(len(errors)),
        "aligned_trajectory": aligned,
        "per_sample_error": errors,
        "R": R,
        "t": t,
    }


def summarize_multi_run(rmse_values):
    """Summarize ATE RMSE across multiple independent runs (OS2's own
    stated methodology: >=10 runs, reported as mean +/- std)."""
    arr = np.asarray(rmse_values, dtype=float)
    return {
        "n_runs": int(len(arr)),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


if __name__ == "__main__":
    # Minimal self-test with synthetic data: a known trajectory, a copy of
    # it rotated/translated/perturbed with noise (simulating an estimate
    # in a different frame with some error), and verification that ATE
    # after alignment recovers approximately the injected noise level,
    # not the (much larger) raw frame offset.
    rng = np.random.default_rng(0)

    n = 200
    t = np.linspace(0, 4 * np.pi, n)
    gt = np.stack([np.cos(t), np.sin(t), 0.1 * t], axis=1)  # helical ground truth

    # simulate an estimate: rotate by 30 deg about Z, translate, add small noise
    theta = np.radians(30)
    R_true = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1],
    ])
    t_true = np.array([5.0, -3.0, 1.0])
    noise_sigma = 0.05
    est = (R_true @ gt.T).T + t_true + rng.normal(0, noise_sigma, size=gt.shape)

    raw_error = np.linalg.norm(est - gt, axis=1)
    print(f"Raw (unaligned) error: mean={raw_error.mean():.3f} m  "
          f"(dominated by the {np.linalg.norm(t_true):.2f} m frame offset, not estimation error)")

    result = compute_ate(est, gt, align=True)
    print(f"Aligned ATE RMSE: {result['rmse']:.4f} m")

    # For isotropic 3D Gaussian noise with per-axis std sigma, the expected
    # norm (and hence RMSE of the norm) is sigma * sqrt(3), not sigma itself.
    expected_rmse = noise_sigma * np.sqrt(3)
    print(f"Expected RMSE for {n} samples of 3D noise (sigma={noise_sigma:.3f} m per axis): "
          f"~{expected_rmse:.4f} m")

    assert abs(result["rmse"] - expected_rmse) < 0.02, "alignment sanity check failed"
    print("Self-test PASSED: alignment correctly removes the 30-deg rotation + "
          f"{np.linalg.norm(t_true):.2f} m translation frame offset, recovering "
          "the true injected-noise error level to within tolerance.")
