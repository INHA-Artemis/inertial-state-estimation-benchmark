from __future__ import annotations

import numpy as np


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def compute_metrics(estimates: np.ndarray, ground_truth: np.ndarray) -> dict[str, float]:
    est = np.asarray(estimates, dtype=float)
    gt = np.asarray(ground_truth, dtype=float)
    n = min(len(est), len(gt))
    if n == 0:
        raise ValueError("Cannot compute metrics for empty arrays.")
    est = est[:n]
    gt = gt[:n]
    pos_err_vec = est[:, :3] - gt[:, :3]
    pos_err = np.linalg.norm(pos_err_vec, axis=1)
    attitude_err = wrap_angle(est[:, 3:6] - gt[:, 3:6])
    heading_err = np.abs(wrap_angle(est[:, 5] - gt[:, 5]))
    return {
        "samples": int(n),
        "position_rmse_m": float(np.sqrt(np.mean(pos_err**2))),
        "position_max_error_m": float(np.max(pos_err)),
        "position_final_error_m": float(pos_err[-1]),
        "roll_rmse_deg": float(np.degrees(np.sqrt(np.mean(attitude_err[:, 0] ** 2)))),
        "pitch_rmse_deg": float(np.degrees(np.sqrt(np.mean(attitude_err[:, 1] ** 2)))),
        "heading_rmse_deg": float(np.degrees(np.sqrt(np.mean(heading_err**2)))),
        "heading_final_error_deg": float(np.degrees(heading_err[-1])),
    }
