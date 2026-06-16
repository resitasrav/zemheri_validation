#!/usr/bin/env python3
"""Analyze the RL policy candidate through the real Gazebo/ROS stack."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DARK = "#1C2541"
MUTED_RED = "#B56576"
BLUE = "#6D8BB0"
CORAL = "#E56B6F"
WHITE = "#FFFFFF"

TOPICS = {
    "/ground_truth/odometry",
    "/odometry/ukf",
    "/guidance/goal",
    "/guidance/setpoint",
    "/navigation/status",
    "/ocean_current",
}


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def read_bag(path):
    reader = open_reader(path)
    available = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = sorted(TOPICS - available.keys())
    if missing:
        raise RuntimeError("RL doğrulama bag topicleri eksik: " + ", ".join(missing))
    types = {topic: get_message(available[topic]) for topic in TOPICS}
    rows = {topic: [] for topic in TOPICS}

    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(raw, types[topic])
        t = bag_time * 1e-9
        if topic in {"/ground_truth/odometry", "/odometry/ukf"}:
            pose = msg.pose.pose
            velocity = msg.twist.twist.linear
            rows[topic].append([
                t, pose.position.x, pose.position.y, pose.position.z,
                velocity.x, velocity.y, velocity.z,
                yaw_from_quaternion(pose.orientation),
            ])
        elif topic == "/guidance/goal":
            rows[topic].append([
                t, msg.guidance_mode, msg.target_x, msg.target_y,
                msg.target_depth, msg.target_yaw, msg.target_speed,
            ])
        elif topic == "/guidance/setpoint":
            rows[topic].append([
                t, msg.guidance_mode, msg.target_depth, msg.target_yaw,
                msg.target_speed, msg.heading_error, msg.depth_error,
            ])
        elif topic == "/navigation/status":
            rows[topic].append([
                t, msg.navigation_valid, msg.navigation_degraded,
                msg.dvl_ok, msg.ukf_ok,
            ])
        else:
            rows[topic].append([t, msg.x, msg.y, msg.z])

    columns = {
        "/ground_truth/odometry": [
            "t", "x", "y", "z", "vx", "vy", "vz", "yaw",
        ],
        "/odometry/ukf": ["t", "x", "y", "z", "vx", "vy", "vz", "yaw"],
        "/guidance/goal": [
            "t", "mode", "target_x", "target_y", "target_depth",
            "target_yaw", "target_speed",
        ],
        "/guidance/setpoint": [
            "t", "mode", "target_depth", "target_yaw", "target_speed",
            "heading_error", "depth_error",
        ],
        "/navigation/status": [
            "t", "navigation_valid", "navigation_degraded", "dvl_ok", "ukf_ok",
        ],
        "/ocean_current": ["t", "current_x", "current_y", "current_z"],
    }
    return {
        topic: pd.DataFrame(values, columns=columns[topic]).sort_values("t")
        for topic, values in rows.items()
    }


def rmse(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values * values))) if len(values) else math.nan


def nearest(left, right, suffix):
    stamped = right.copy()
    stamped[f"sample_t_{suffix}"] = stamped["t"]
    renamed = stamped.rename(
        columns={column: f"{column}_{suffix}" for column in right if column != "t"}
    )
    return pd.merge_asof(
        left.sort_values("t"),
        renamed.sort_values("t"),
        on="t",
        direction="nearest",
    )


def build_timeline(frames):
    gt = frames["/ground_truth/odometry"]
    ukf = frames["/odometry/ukf"]
    goals = frames["/guidance/goal"]
    active_goals = goals[goals["mode"] == "LOS"]
    if gt.empty or ukf.empty or active_goals.empty:
        raise RuntimeError("RL doğrulaması için GT, UKF ve aktif LOS hedefi gerekli.")

    start = max(gt["t"].min(), ukf["t"].min(), active_goals["t"].min())

    def normalized(frame):
        result = frame[frame["t"] >= start].copy()
        result["t"] -= start
        return result

    gt = normalized(gt)
    ukf = normalized(ukf)
    active_goals = normalized(active_goals)
    normalized_frames = {
        topic: normalized(frame) for topic, frame in frames.items()
    }

    timeline = nearest(gt, ukf, "ukf")
    timeline = pd.merge_asof(
        timeline.sort_values("t"),
        active_goals.sort_values("t"),
        on="t",
        direction="backward",
    ).dropna(subset=["target_x"])
    timeline = nearest(
        timeline, normalized_frames["/guidance/setpoint"], "setpoint"
    )
    timeline = nearest(
        timeline, normalized_frames["/navigation/status"], "navigation"
    )
    timeline = nearest(timeline, normalized_frames["/ocean_current"], "current")

    gt_x0 = float(timeline["x"].iloc[0])
    gt_y0 = float(timeline["y"].iloc[0])
    gt_z0 = float(timeline["z"].iloc[0])
    ukf_x0 = float(timeline["x_ukf"].iloc[0])
    ukf_y0 = float(timeline["y_ukf"].iloc[0])
    ukf_z0 = float(timeline["z_ukf"].iloc[0])
    target_dx = timeline["target_x"] - ukf_x0
    target_dy = timeline["target_y"] - ukf_y0
    target_length = np.hypot(target_dx, target_dy)
    axis_x = target_dx / target_length.clip(lower=1e-9)
    axis_y = target_dy / target_length.clip(lower=1e-9)
    dx = timeline["x"] - gt_x0
    dy = timeline["y"] - gt_y0
    timeline["relative_x"] = dx
    timeline["relative_y"] = dy
    timeline["relative_x_ukf"] = timeline["x_ukf"] - ukf_x0
    timeline["relative_y_ukf"] = timeline["y_ukf"] - ukf_y0
    timeline["target_relative_x"] = timeline["target_x"] - ukf_x0
    timeline["target_relative_y"] = timeline["target_y"] - ukf_y0
    timeline["along_track"] = dx * axis_x + dy * axis_y
    timeline["cross_track"] = -dx * axis_y + dy * axis_x
    timeline["speed"] = np.sqrt(
        timeline["vx"] ** 2 + timeline["vy"] ** 2 + timeline["vz"] ** 2
    )
    timeline["position_error"] = np.sqrt(
        (timeline["relative_x_ukf"] - timeline["relative_x"]) ** 2
        + (timeline["relative_y_ukf"] - timeline["relative_y"]) ** 2
        + (
            (timeline["z_ukf"] - ukf_z0)
            - (timeline["z"] - gt_z0)
        ) ** 2
    )
    timeline["ukf_alignment_error_ms"] = (
        1000.0 * (timeline["t"] - timeline["sample_t_ukf"]).abs()
    )
    timeline.attrs["initial_frame_offset_m"] = math.sqrt(
        (gt_x0 - ukf_x0) ** 2
        + (gt_y0 - ukf_y0) ** 2
        + (gt_z0 - ukf_z0) ** 2
    )
    return timeline


def configure_plot():
    plt.rcParams.update({
        "figure.facecolor": WHITE,
        "axes.facecolor": WHITE,
        "axes.edgecolor": DARK,
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linestyle": "--",
        "font.family": "DejaVu Sans",
        "savefig.dpi": 240,
        "savefig.bbox": "tight",
    })


def save_figures(timeline, output):
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(9.2, 5.2))
    ax.plot(
        timeline["relative_x"], timeline["relative_y"], color=CORAL, linewidth=2.0,
        label="Ground truth",
    )
    ax.plot(
        timeline["relative_x_ukf"], timeline["relative_y_ukf"],
        color=BLUE, linewidth=1.5,
        label="UKF",
    )
    ax.plot(
        [0.0, timeline["target_relative_x"].iloc[-1]],
        [0.0, timeline["target_relative_y"].iloc[-1]],
        color=DARK, linestyle="--", linewidth=1.5, label="Politika referansı",
    )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "rl_policy_trajectory.png")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9.2, 8.0), sharex=True)
    axes[0].plot(timeline["t"], -timeline["z"], color=CORAL, label="Gerçek derinlik")
    axes[0].plot(
        timeline["t"], timeline["target_depth"], color=DARK,
        linestyle="--", label="Hedef derinlik",
    )
    axes[0].set_ylabel("Derinlik (m)")
    axes[0].legend()
    axes[1].plot(timeline["t"], timeline["speed"], color=BLUE, label="Gerçek hız")
    axes[1].plot(
        timeline["t"], timeline["target_speed"], color=DARK,
        linestyle="--", label="Politika hız hedefi",
    )
    axes[1].axhline(2.5, color=MUTED_RED, linestyle=":", label="DVL sınırı")
    axes[1].set_ylabel("Hız (m/s)")
    axes[1].legend()
    axes[2].plot(
        timeline["t"], timeline["cross_track"], color=CORAL,
        label="Cross-track hata",
    )
    axes[2].plot(
        timeline["t"], timeline["position_error"], color=BLUE,
        label="UKF konum hatası",
    )
    axes[2].set_xlabel("Zaman (s)")
    axes[2].set_ylabel("Hata (m)")
    axes[2].legend()
    fig.tight_layout()
    fig.savefig(figures / "rl_policy_tracking.png")
    plt.close(fig)


def write_markdown(summary, path):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("| Ölçüt | Değer |\n|---|---:|\n")
        for key, value in summary.items():
            if isinstance(value, float):
                value = f"{value:.5f}"
            stream.write(f"| {key} | {value} |\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--min-progress-ratio", type=float, default=0.90)
    parser.add_argument("--max-cross-track-rmse", type=float, default=2.0)
    parser.add_argument("--max-depth-rmse", type=float, default=0.35)
    parser.add_argument("--max-speed", type=float, default=2.50)
    parser.add_argument("--min-navigation-valid-ratio", type=float, default=0.95)
    args = parser.parse_args()
    bag_input = args.bag.expanduser().resolve()
    bag = bag_input.parent if bag_input.is_file() else bag_input
    output = (args.output or bag.parent).expanduser().resolve()
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)

    frames = read_bag(bag)
    timeline = build_timeline(frames)
    target_distance = math.hypot(
        timeline["target_relative_x"].iloc[-1],
        timeline["target_relative_y"].iloc[-1],
    )
    final_progress = float(timeline["along_track"].iloc[-1])
    depth_error = -timeline["z"] - timeline["target_depth"]
    navigation_valid_ratio = float(
        timeline["navigation_valid_navigation"].astype(bool).mean()
    )
    gt_displacement = math.sqrt(
        float(timeline["relative_x"].iloc[-1]) ** 2
        + float(timeline["relative_y"].iloc[-1]) ** 2
        + float(timeline["z"].iloc[-1] - timeline["z"].iloc[0]) ** 2
    )
    ukf_displacement = math.sqrt(
        float(timeline["relative_x_ukf"].iloc[-1]) ** 2
        + float(timeline["relative_y_ukf"].iloc[-1]) ** 2
        + float(timeline["z_ukf"].iloc[-1] - timeline["z_ukf"].iloc[0]) ** 2
    )
    ukf_motion_valid = ukf_displacement >= max(0.10, 0.05 * gt_displacement)
    ukf_position_rmse = (
        rmse(timeline["position_error"]) if ukf_motion_valid else math.nan
    )
    accepted = (
        final_progress >= args.min_progress_ratio * target_distance
        and rmse(timeline["cross_track"]) <= args.max_cross_track_rmse
        and rmse(depth_error) <= args.max_depth_rmse
        and float(timeline["speed"].max()) <= args.max_speed
        and navigation_valid_ratio >= args.min_navigation_valid_ratio
    )
    summary = {
        "Doğrulama kararı": (
            "KABUL - ROS/Gazebo politika adayı koşulları sağlandı"
            if accepted
            else "BAŞARISIZ - ROS/Gazebo politika adayı koşulları sağlanmadı"
        ),
        "Test süresi (s)": float(timeline["t"].iloc[-1] - timeline["t"].iloc[0]),
        "Hedef mesafe (m)": target_distance,
        "İlerleme (m)": final_progress,
        "Son cross-track hata (m)": float(timeline["cross_track"].iloc[-1]),
        "Cross-track RMSE (m)": rmse(timeline["cross_track"]),
        "Derinlik RMSE (m)": rmse(depth_error),
        "Başlangıç GT-UKF çerçeve ofseti (m)": (
            timeline.attrs["initial_frame_offset_m"]
        ),
        "GT yer değiştirme (m)": gt_displacement,
        "UKF yer değiştirme (m)": ukf_displacement,
        "UKF karşılaştırma durumu": (
            "GEÇERLİ"
            if ukf_motion_valid
            else "GEÇERSİZ - UKF konum hareketi gözlemlenmedi"
        ),
        "UKF konum RMSE (m)": ukf_position_rmse,
        "UKF zaman eşleme ortalama hatası (ms)": float(
            timeline["ukf_alignment_error_ms"].mean()
        ),
        "UKF zaman eşleme maksimum hatası (ms)": float(
            timeline["ukf_alignment_error_ms"].max()
        ),
        "Maksimum hız (m/s)": float(timeline["speed"].max()),
        "DVL hız sınırı ihlal sayısı": int(
            (timeline["speed"] > args.max_speed).sum()
        ),
        "Navigasyon geçerli oranı": navigation_valid_ratio,
        "Navigasyon degraded oranı": float(
            timeline["navigation_degraded_navigation"].astype(bool).mean()
        ),
    }

    configure_plot()
    save_figures(timeline, output)
    timeline.to_csv(metrics / "rl_policy_timeseries.csv", index=False)
    pd.DataFrame([summary]).to_csv(metrics / "rl_policy_summary.csv", index=False)
    pd.DataFrame([{
        "episode": output.name,
        **summary,
    }]).to_csv(metrics / "rl_episode_summary.csv", index=False)
    write_markdown(summary, metrics / "rl_policy_summary.md")
    print(pd.DataFrame([summary]).to_string(index=False))


if __name__ == "__main__":
    main()
