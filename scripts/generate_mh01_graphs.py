#!/usr/bin/env python3
"""Generate presentation-ready graphs from mh01_benchmark --trace-dir output."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

COLORS = {"truth": "#1a1a1a", "prior": "#3677b8", "posterior": "#238b45", "measurement": "#dd7a01", "danger": "#c93c3c"}


def rows(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def value(row, key):
    text = row[key]
    return float(text) if text else math.nan


def time_s(frame_rows):
    start = float(frame_rows[0]["timestamp_ns"])
    return np.array([(float(row["timestamp_ns"]) - start) * 1e-9 for row in frame_rows])


def position_error(frame_rows, prefix):
    estimate = np.array([[value(row, f"{prefix}_p{axis}") for axis in "xyz"] for row in frame_rows])
    truth = np.array([[value(row, f"truth_p{axis}") for axis in "xyz"] for row in frame_rows])
    return np.linalg.norm(estimate - truth, axis=1)


def sigma_position(frame_rows, prefix):
    return np.array([math.sqrt(max(0.0, sum(value(row, f"{prefix}_P{axis}{axis}") for axis in range(3)))) for row in frame_rows])


def quaternion_to_rotation(quaternion):
    w, x, y, z = quaternion
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def so3_log(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]]) / 2.0
    return angle / (2.0 * math.sin(angle)) * np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])


def nees(frame_rows):
    result = []
    for row in frame_rows:
        error = []
        for prefix in ("p", "v"):
            error.extend(value(row, f"truth_{prefix}{axis}") - value(row, f"posterior_{prefix}{axis}") for axis in "xyz")
        estimate_rotation = quaternion_to_rotation([value(row, f"posterior_q{axis}") for axis in "wxyz"])
        truth_rotation = quaternion_to_rotation([value(row, f"truth_q{axis}") for axis in "wxyz"])
        error.extend(so3_log(estimate_rotation.T @ truth_rotation))
        for prefix in ("ba", "bg"):
            error.extend(value(row, f"truth_{prefix}{axis}") - value(row, f"posterior_{prefix}{axis}") for axis in "xyz")
        covariance = np.array([[value(row, f"posterior_P{r}{c}") for c in range(15)] for r in range(15)])
        try:
            result.append(float(np.dot(error, np.linalg.solve(covariance, error))))
        except np.linalg.LinAlgError:
            result.append(math.nan)
    return np.array(result)


def style_axis(axis, title, ylabel, xlabel="Time (s)"):
    axis.set_title(title, loc="left", fontweight="bold")
    axis.set_xlabel(xlabel)
    axis.set_ylabel(ylabel)
    axis.grid(alpha=0.25)
    axis.spines[["top", "right"]].set_visible(False)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def covariance_ellipse(axis, row, prefix, color, label):
    covariance = np.array([[value(row, f"{prefix}_P00"), value(row, f"{prefix}_P01")], [value(row, f"{prefix}_P10"), value(row, f"{prefix}_P11")]])
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    eigenvalues = np.maximum(eigenvalues, 0.0)
    angle = math.degrees(math.atan2(eigenvectors[1, 1], eigenvectors[0, 1]))
    ellipse = Ellipse((value(row, f"{prefix}_px"), value(row, f"{prefix}_py")), 4 * math.sqrt(eigenvalues[1]), 4 * math.sqrt(eigenvalues[0]), angle=angle,
                      facecolor=color, edgecolor=color, alpha=0.18, label=label)
    axis.add_patch(ellipse)


def generate(trace, output):
    camera = rows(trace / "camera_trace.csv")
    observations = rows(trace / "observation_trace.csv")
    output.mkdir(parents=True, exist_ok=True)
    times = time_s(camera)
    prior_error = position_error(camera, "prior")
    posterior_error = position_error(camera, "posterior")

    fig, axis = plt.subplots(figsize=(8, 6))
    axis.plot([value(row, "truth_px") for row in camera], [value(row, "truth_py") for row in camera], color=COLORS["truth"], label="Ground truth", linewidth=2)
    axis.plot([value(row, "prior_px") for row in camera], [value(row, "prior_py") for row in camera], color=COLORS["prior"], label="IMU prediction", alpha=0.65)
    axis.plot([value(row, "posterior_px") for row in camera], [value(row, "posterior_py") for row in camera], color=COLORS["posterior"], label="Stereo-updated estimate", linewidth=1.4)
    style_axis(axis, "MH_01 trajectory in the EuRoC world frame", "Y position (m)", "X position (m)")
    axis.axis("equal"); axis.legend(); save(fig, output / "01_trajectory_overlay.png")

    correction = np.linalg.norm(np.array([[value(row, f"posterior_p{axis}") - value(row, f"prior_p{axis}") for axis in "xyz"] for row in camera]), axis=1)
    chosen = int(np.nanargmax(correction))
    row = camera[chosen]
    window = slice(max(0, chosen - 60), min(len(camera), chosen + 61))
    fig, axis = plt.subplots(figsize=(8, 6))
    axis.plot([value(item, "truth_px") for item in camera[window]], [value(item, "truth_py") for item in camera[window]], color="0.55", label="Local ground truth")
    covariance_ellipse(axis, row, "prior", COLORS["prior"], "Prior 2σ covariance")
    covariance_ellipse(axis, row, "posterior", COLORS["posterior"], "Posterior 2σ covariance")
    axis.scatter(value(row, "truth_px"), value(row, "truth_py"), color=COLORS["truth"], s=65, zorder=5, label="True state")
    axis.scatter(value(row, "prior_px"), value(row, "prior_py"), marker="x", color=COLORS["prior"], s=90, zorder=6, label="IMU prediction")
    axis.scatter(value(row, "posterior_px"), value(row, "posterior_py"), marker="D", color=COLORS["posterior"], s=45, zorder=6, label="Posterior estimate")
    axis.annotate("", xy=(value(row, "posterior_px"), value(row, "posterior_py")), xytext=(value(row, "prior_px"), value(row, "prior_py")), arrowprops={"arrowstyle": "->", "color": COLORS["posterior"], "lw": 2})
    style_axis(axis, f"One Kalman camera update at t = {times[chosen]:.1f} s", "Y position (m)", "X position (m)")
    axis.axis("equal"); axis.legend(loc="best"); save(fig, output / "02_kalman_update_in_action.png")

    fig, axis = plt.subplots(figsize=(9, 4.8))
    axis.semilogy(times, prior_error, color=COLORS["prior"], label="Prior position error")
    axis.semilogy(times, posterior_error, color=COLORS["posterior"], label="Posterior position error")
    axis.semilogy(times, sigma_position(camera, "prior"), "--", color=COLORS["prior"], label="Prior 1σ position")
    axis.semilogy(times, sigma_position(camera, "posterior"), "--", color=COLORS["posterior"], label="Posterior 1σ position")
    style_axis(axis, "Position error and reported uncertainty", "Magnitude (m)")
    axis.legend(ncol=2); save(fig, output / "03_error_and_uncertainty.png")

    fig, axis = plt.subplots(figsize=(9, 4.8))
    axis.semilogy(times, nees(camera), color="#733fa5", linewidth=1)
    axis.axhline(15, color="0.25", linestyle="--", label="Expected 15-dof NEES")
    style_axis(axis, "Posterior robot NEES across MH_01", "NEES (dimensionless)")
    axis.legend(); save(fig, output / "04_nees_over_time.png")

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].semilogy(times, correction, color=COLORS["posterior"])
    style_axis(axes[0], "Camera-update correction magnitude", "‖posterior − prior‖ (m)")
    axes[1].semilogy(times, prior_error, color=COLORS["prior"], label="Prior")
    axes[1].semilogy(times, posterior_error, color=COLORS["posterior"], label="Posterior")
    style_axis(axes[1], "Does each visual update improve position immediately?", "Position error (m)")
    axes[1].legend(); save(fig, output / "05_camera_update_effect.png")

    by_timestamp = {}
    for observation in observations:
        bucket = by_timestamp.setdefault(observation["timestamp_ns"], [0, 0])
        bucket[0 if observation["outcome"] == "applied" else 1] += 1
    count_times = [(float(timestamp) - float(camera[0]["timestamp_ns"])) * 1e-9 for timestamp in by_timestamp]
    applied = [counts[0] for counts in by_timestamp.values()]
    rejected = [counts[1] for counts in by_timestamp.values()]
    distances = np.array([value(observation, "mahalanobis_distance") for observation in observations])
    outcomes = np.array([observation["outcome"] == "applied" for observation in observations])
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=False)
    axes[0].plot(count_times, applied, color=COLORS["posterior"], label="Applied")
    axes[0].plot(count_times, rejected, color=COLORS["danger"], label="Rejected / skipped")
    style_axis(axes[0], "Per-frame observation outcomes", "Observations")
    axes[0].legend()
    finite = np.isfinite(distances)
    axes[1].hist(distances[finite & outcomes], bins=np.linspace(0, 40, 81), alpha=0.65, color=COLORS["posterior"], label="Applied")
    axes[1].hist(distances[finite & ~outcomes], bins=np.linspace(0, 40, 81), alpha=0.6, color=COLORS["danger"], label="Gated")
    axes[1].axvline(9.4877, color="0.2", linestyle="--", label="95% χ² gate")
    style_axis(axes[1], "Innovation-gate distribution", "Observation count", "Mahalanobis distance")
    axes[1].legend(); save(fig, output / "06_innovation_gating.png")

    selected_observations = [observation for observation in observations if observation["timestamp_ns"] == row["timestamp_ns"] and observation["outcome"] == "applied"]
    fig, axis = plt.subplots(figsize=(7, 6))
    if selected_observations:
        predicted = np.array([[value(observation, "predicted_u0"), value(observation, "predicted_v0")] for observation in selected_observations])
        observed = np.array([[value(observation, "observed_u0"), value(observation, "observed_v0")] for observation in selected_observations])
        for predicted_point, observed_point in zip(predicted, observed):
            axis.plot([predicted_point[0], observed_point[0]], [predicted_point[1], observed_point[1]], color="0.7", linewidth=0.7)
        axis.scatter(predicted[:, 0], predicted[:, 1], marker="x", color=COLORS["prior"], label="Predicted pixels")
        axis.scatter(observed[:, 0], observed[:, 1], marker="o", facecolors="none", edgecolors=COLORS["measurement"], label="Observed pixels")
    style_axis(axis, f"Pixel innovations at t = {times[chosen]:.1f} s", "v₀ (pixels)", "u₀ (pixels)")
    axis.invert_yaxis(); axis.legend(); save(fig, output / "07_pixel_innovations.png")

    fig, axis = plt.subplots(figsize=(8, 4.8))
    labels = ["Correct\nseed 1", "Correct\nseed 2", "Correct\nseed 3", "Corrupted F\n(minimum)"]
    values = [0.039283, 0.040966, 0.030136, 0.23]
    axis.bar(labels, values, color=[COLORS["posterior"]] * 3 + [COLORS["danger"]])
    axis.axhline(0.15, color="0.2", linestyle="--", label="Regression gate")
    style_axis(axis, "Monte Carlo covariance test has a demonstrated failure margin", "Max normalized covariance deviation", "")
    axis.legend(); save(fig, output / "08_covariance_monte_carlo_margin.png")

    fig, axis = plt.subplots(figsize=(7, 5))
    rates = np.array([200, 400, 800], dtype=float)
    errors = np.array([0.00137943, 0.000689604, 0.000344773])
    axis.loglog(rates, errors, "o-", color=COLORS["posterior"], label="Measured error")
    axis.loglog(rates, errors[0] * rates[0] / rates, "--", color="0.25", label="First-order reference")
    style_axis(axis, "Measured integration convergence", "Position + velocity error", "IMU rate (Hz)")
    axis.legend(); save(fig, output / "09_integrator_convergence.png")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.8))
    axes[0].bar(["Propagation\nonly", "With camera\nupdates"], [13.56, 24.23], color=[COLORS["prior"], COLORS["danger"]])
    axes[0].axhline(15, color="0.2", linestyle="--", label="Expected")
    axes[0].axhspan(13.52, 16.56, color="#a8d5b5", alpha=0.45, label="95% interval")
    style_axis(axes[0], "Synthetic 15-dof NEES", "Mean NEES", "")
    axes[0].legend(fontsize=8)
    blocks = ["Position", "Velocity", "Orientation", "Accel bias", "Gyro bias"]
    axes[1].bar(blocks, [2.73, 4.05, 6.35, 7.16, 4.81], color=[COLORS["posterior"], COLORS["danger"], COLORS["danger"], COLORS["danger"], COLORS["danger"]])
    axes[1].axhline(3, color="0.2", linestyle="--", label="Expected per block")
    axes[1].tick_params(axis="x", rotation=25)
    style_axis(axes[1], "Where update inconsistency concentrates", "Mean 3-dof NEES", "")
    axes[1].legend(fontsize=8); save(fig, output / "10_nees_summary_and_blocks.png")

    position_block_error = posterior_error
    velocity_block_error = np.array([math.sqrt(sum((value(row, f"truth_v{axis}") - value(row, f"posterior_v{axis}")) ** 2 for axis in "xyz")) for row in camera])
    attitude_block_error = np.array([
        np.linalg.norm(so3_log(quaternion_to_rotation([value(row, f"posterior_q{axis}") for axis in "wxyz"]).T @
                              quaternion_to_rotation([value(row, f"truth_q{axis}") for axis in "wxyz"])))
        for row in camera
    ])
    accel_bias_block_error = np.array([math.sqrt(sum((value(row, f"truth_ba{axis}") - value(row, f"posterior_ba{axis}")) ** 2 for axis in "xyz")) for row in camera])
    gyro_bias_block_error = np.array([math.sqrt(sum((value(row, f"truth_bg{axis}") - value(row, f"posterior_bg{axis}")) ** 2 for axis in "xyz")) for row in camera])
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].semilogy(times, position_block_error, color=COLORS["posterior"], label="Position (m)")
    axes[0].semilogy(times, velocity_block_error, color=COLORS["prior"], label="Velocity (m/s)")
    style_axis(axes[0], "Robot-state error by block", "Norm (native units)")
    axes[0].legend()
    axes[1].semilogy(times, attitude_block_error, color="#733fa5", label="Orientation (rad)")
    style_axis(axes[1], "Attitude error", "Norm (rad)")
    axes[1].legend()
    axes[2].semilogy(times, accel_bias_block_error, color=COLORS["measurement"], label="Accelerometer bias (m/s²)")
    axes[2].semilogy(times, gyro_bias_block_error, color=COLORS["danger"], label="Gyroscope bias (rad/s)")
    style_axis(axes[2], "Bias error", "Norm (native units)")
    axes[2].legend(); save(fig, output / "11_state_block_errors.png")

    observation_times = np.array([(float(item["timestamp_ns"]) - float(camera[0]["timestamp_ns"])) * 1e-9 for item in observations])
    fig, axis = plt.subplots(figsize=(9, 4.8))
    gated = np.array([item["outcome"] == "gated" for item in observations])
    finite = np.isfinite(distances)
    axis.scatter(observation_times[finite & ~gated], distances[finite & ~gated], s=1, alpha=0.08, color=COLORS["posterior"], label="Applied")
    axis.scatter(observation_times[finite & gated], distances[finite & gated], s=1, alpha=0.14, color=COLORS["danger"], label="Gated")
    axis.axhline(9.4877, color="0.15", linestyle="--", label="95% χ² gate")
    style_axis(axis, "Innovation distance over the full MH_01 run", "Mahalanobis distance")
    axis.set_ylim(0, 50); axis.legend(markerscale=5); save(fig, output / "12_gate_distance_over_time.png")

    frame_applied = np.array([value(row, "applied_count") for row in camera])
    frame_gated = np.array([value(row, "gated_count") for row in camera])
    acceptance = np.divide(frame_applied, frame_applied + frame_gated, out=np.full_like(frame_applied, np.nan), where=(frame_applied + frame_gated) > 0)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(times, [value(row, "active_tracks") for row in camera], color=COLORS["prior"])
    style_axis(axes[0], "Frontend health: active tracks", "Track count")
    axes[1].plot(times, acceptance, color=COLORS["posterior"], label="Applied / (applied + gated)")
    axes[1].plot(times, frame_applied, color=COLORS["prior"], alpha=0.65, label="Applied observations")
    axes[1].plot(times, frame_gated, color=COLORS["danger"], alpha=0.65, label="Gated observations")
    style_axis(axes[1], "Update acceptance and observation volume", "Rate / count")
    axes[1].legend(ncol=3, fontsize=8); save(fig, output / "13_frontend_health.png")

    covariance_eigenvalues = np.array([np.linalg.eigvalsh(np.array([[value(row, f"posterior_P{r}{c}") for c in range(15)] for r in range(15)])) for row in camera])
    minimum_eigenvalue = covariance_eigenvalues[:, 0]
    condition_number = covariance_eigenvalues[:, -1] / np.maximum(minimum_eigenvalue, np.finfo(float).tiny)
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].semilogy(times, np.maximum(minimum_eigenvalue, np.finfo(float).tiny), color=COLORS["posterior"])
    style_axis(axes[0], "Covariance numerical health", "Minimum eigenvalue")
    axes[1].semilogy(times, condition_number, color="#733fa5")
    style_axis(axes[1], "Covariance conditioning", "λmax / λmin")
    save(fig, output / "14_covariance_numerical_health.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("trace_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/mh01_graphs"))
    arguments = parser.parse_args()
    generate(arguments.trace_dir, arguments.output_dir)
