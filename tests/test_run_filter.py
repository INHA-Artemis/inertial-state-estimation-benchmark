from __future__ import annotations

import argparse
import csv

import numpy as np

from datasets.common import CommonDataset
from runners import run_filter as run_filter_module


class _RecordingFilter:
    instances: list["_RecordingFilter"] = []

    def __init__(self, initial_pose: np.ndarray) -> None:
        self.pose = initial_pose.copy()
        self.predict_calls: list[tuple[np.ndarray, float]] = []
        self.__class__.instances.append(self)

    @classmethod
    def from_configs(cls, dataset_config: dict, compare_config: dict) -> "_RecordingFilter":
        initialization = compare_config["quaternion_ekf_15d"]["initialization"]
        return cls(np.asarray(initialization["mean"], dtype=float))

    def predict(self, control: np.ndarray, dt: float) -> np.ndarray:
        control = np.asarray(control, dtype=float)
        self.predict_calls.append((control.copy(), dt))
        self.pose[0] += control[0] * dt
        return self.pose.copy()

    def estimate_pose(self) -> np.ndarray:
        return self.pose.copy()


def test_run_filter_keeps_initial_state_at_first_timestamp(monkeypatch, tmp_path) -> None:
    initial_pose = np.array([10.0, 20.0, 30.0, 0.1, 0.2, 0.3])
    dataset = CommonDataset(
        name="synthetic",
        sequence="time_alignment",
        timestamps=np.array([1.0, 1.1, 1.3]),
        controls=np.array(
            [
                [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        ),
        dt=np.array([0.1, 0.1, 0.2]),
        position_measurements=np.zeros((3, 3)),
        position_measurement_mask=np.zeros(3, dtype=bool),
        ground_truth=np.tile(initial_pose, (3, 1)),
        initial_velocity=np.zeros(3),
    )

    monkeypatch.setattr(run_filter_module, "_load_yaml", lambda path: {"mode": "imu_only"})
    monkeypatch.setattr(run_filter_module, "load_dataset", lambda config: dataset)
    monkeypatch.setattr(run_filter_module, "get_filter_class", lambda name: _RecordingFilter)
    _RecordingFilter.instances.clear()

    args = argparse.Namespace(
        filter="ekf",
        config="unused.yaml",
        filter_config="unused-filter.yaml",
        output_root=str(tmp_path),
        max_steps=0,
        particles=None,
        no_plots=True,
    )
    run_filter_module.run_filter(args)

    estimator = _RecordingFilter.instances[-1]
    assert len(estimator.predict_calls) == 2
    np.testing.assert_array_equal(estimator.predict_calls[0][0], dataset.controls[1])
    assert estimator.predict_calls[0][1] == dataset.dt[1]

    estimate_path = tmp_path / "synthetic" / "time_alignment" / "ekf" / "estimate.csv"
    with estimate_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    first_estimate = np.array([float(rows[0][f"est_{axis}"]) for axis in ("px", "py", "pz", "roll", "pitch", "yaw")])
    np.testing.assert_allclose(first_estimate, initial_pose)
