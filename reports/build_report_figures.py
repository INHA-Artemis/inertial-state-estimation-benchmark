#!/usr/bin/env python3
"""Build the figures used in the state-estimation report."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "data"
FIG = ROOT / "figures"
COLORS = {"EKF": "#4c78a8", "UKF": "#f58518", "PF": "#e45756", "InEKF": "#54a24b"}


def read_csv(name: str) -> list[dict[str, str]]:
    with (DATA / name).open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def save(fig: plt.Figure, name: str) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / name, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def historical_filters() -> None:
    rows = read_csv("historical_full_sequence.csv")
    datasets = ["i2Nav street01", "Pohang05", "UrbanNav"]
    filters = ["EKF", "UKF", "PF", "InEKF"]
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 6.7))
    for col, dataset in enumerate(datasets):
        block = {r["filter"]: r for r in rows if r["dataset"] == dataset}
        pos = [float(block[f]["position_rmse_m"]) for f in filters]
        yaw = [float(block[f]["heading_rmse_deg"]) for f in filters]
        x = np.arange(len(filters))
        axes[0, col].bar(x, pos, color=[COLORS[f] for f in filters])
        axes[1, col].bar(x, yaw, color=[COLORS[f] for f in filters])
        axes[0, col].set_title(dataset)
        axes[0, col].set_ylabel("Position RMSE [m]")
        axes[1, col].set_ylabel("Heading RMSE [deg]")
        for ax in axes[:, col]:
            ax.set_xticks(x, filters, rotation=20)
            ax.grid(axis="y", alpha=0.25)
    fig.suptitle("Filter comparison recorded in the 08/04 presentation", fontsize=14)
    fig.text(0.5, -0.015,
             "Pohang used the baseline for both position updates and evaluation. UrbanNav was not rerun.",
             ha="center", color="#9c2f2f", fontsize=9)
    fig.tight_layout()
    save(fig, "historical_filter_comparison.png")


def euroc_modes() -> None:
    rows = read_csv("euroc_results.csv")
    filters = ["EKF", "UKF", "PF-500", "InEKF"]
    names = ["EKF", "UKF", "PF", "InEKF"]
    x = np.arange(len(filters))
    width = 0.36
    imu = {r["filter"]: r for r in rows if r["mode"] == "IMU-only"}
    fused = {r["filter"]: r for r in rows if r["mode"] == "IMU+position"}
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.1))
    for ax, key, ylabel, log in [
        (axes[0], "position_rmse_m", "Position RMSE [m]", True),
        (axes[1], "heading_rmse_deg", "Heading RMSE [deg]", False),
    ]:
        a = [float(imu[f][key]) for f in filters]
        b = [float(fused[f][key]) for f in filters]
        ax.bar(x - width / 2, a, width, label="IMU-only", color="#b8b8b8")
        ax.bar(x + width / 2, b, width, label="IMU + 2 Hz position", color="#4c78a8")
        ax.set_xticks(x, names)
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.25)
        if log:
            ax.set_yscale("log")
        ax.legend(frameon=False)
    fig.suptitle("EuRoC V1_01_easy — same inputs and parameters for every filter")
    fig.tight_layout()
    save(fig, "euroc_imu_only_vs_fused.png")


def motion_regimes() -> None:
    rows = read_csv("motion_regime_metrics.csv")
    regimes = ["mobile_robot", "surface_vessel", "drone"]
    regime_labels = ["Mobile robot-like", "Surface vessel-like", "Drone-like"]
    filters = ["ekf", "ukf", "pf", "inekf"]
    colors = {"ekf": "#4c78a8", "ukf": "#f58518", "pf": "#999999", "inekf": "#6f4e9c"}
    panels = [
        ("imu_only", "heading_rmse_deg", "IMU-only heading RMSE [deg]", False),
        ("fused", "heading_rmse_deg", "IMU + position heading RMSE [deg]", False),
        ("imu_only", "position_rmse_m", "IMU-only position RMSE [m]", True),
        ("fused", "position_rmse_m", "IMU + position position RMSE [m]", True),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.3))
    x = np.arange(len(regimes))
    width = 0.19
    for axis, (mode, field, title, log_scale) in zip(axes.ravel(), panels):
        for offset, filter_name in enumerate(filters):
            values = [
                float(next(
                    row[field]
                    for row in rows
                    if row["regime"] == regime
                    and row["mode"] == mode
                    and row["filter"] == filter_name
                ))
                for regime in regimes
            ]
            axis.bar(
                x + (offset - 1.5) * width,
                values,
                width,
                color=colors[filter_name],
                label=filter_name.upper(),
            )
        axis.set_xticks(x, regime_labels)
        axis.set_title(title)
        if log_scale:
            axis.set_yscale("log")
        axis.grid(axis="y", alpha=0.25, which="both")
        axis.spines[["top", "right"]].set_visible(False)
    axes[0, 0].legend(ncol=4, frameon=False, fontsize=9)
    fig.suptitle("Controlled motion comparison with the same noise and update schedule", fontweight="bold")
    fig.tight_layout()
    save(fig, "motion_regime_summary.png")


def learned_results() -> None:
    rows = read_csv("cf231_learned.csv")
    labels = ["Fixed bias\nIMU DR", "Shallow velocity\n+ InEKF", "Small TCN\nloose", "Small TCN\n+ InEKF"]
    x = np.arange(len(rows))
    pos = [float(r["position_rmse_m"]) for r in rows]
    att = [float(r["so3_rmse_deg"]) for r in rows]
    colors = ["#b8b8b8", "#f58518", "#72b7b2", "#54a24b"]
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    axes[0].bar(x, pos, color=colors)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Position RMSE [m] (log scale)")
    axes[1].bar(x, att, color=colors)
    axes[1].set_ylabel("SO(3) attitude RMSE [deg]")
    for ax in axes:
        ax.set_xticks(x, labels)
        ax.grid(axis="y", alpha=0.25)
    fig.suptitle("CF231 Run 5 — Run 5 is excluded from training and uses IMU only")
    fig.tight_layout()
    save(fig, "cf231_learned_comparison.png")


def main() -> None:
    historical_filters()
    euroc_modes()
    motion_regimes()
    learned_results()
    print(f"Figures written to {FIG}")


if __name__ == "__main__":
    main()
