#!/usr/bin/env python3
"""Generate report-ready waypoint and LOS guidance validation assets."""

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
from matplotlib.patches import Circle  # noqa: E402


DARK = "#1C2541"
MUTED_RED = "#B56576"
BLUE = "#6D8BB0"
CORAL = "#E56B6F"
WHITE = "#FFFFFF"

GT_TOPIC = "/ground_truth/odometry"
GOAL_TOPIC = "/guidance/goal"
SETPOINT_TOPIC = "/guidance/setpoint"


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
    types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = [GT_TOPIC, GOAL_TOPIC, SETPOINT_TOPIC]
    missing = [topic for topic in required if topic not in types]
    if missing:
        raise RuntimeError(
            "Bag içinde gerekli topicler yok: " + ", ".join(missing)
        )
    message_types = {topic: get_message(types[topic]) for topic in required}
    gt_rows = []
    goal_rows = []
    setpoint_rows = []
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(raw, message_types[topic])
        # Guidance goals may be produced by a wall-time test runner while the
        # simulated vehicle uses /clock. Bag reception time provides one
        # consistent timeline for all guidance-validation topics.
        timestamp = bag_time * 1e-9
        if topic == GT_TOPIC:
            pose = msg.pose.pose
            velocity = msg.twist.twist.linear
            gt_rows.append([
                timestamp,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                velocity.x,
                velocity.y,
                velocity.z,
                yaw_from_quaternion(pose.orientation),
            ])
        elif topic == GOAL_TOPIC:
            goal_rows.append([
                timestamp,
                msg.guidance_mode,
                msg.target_x,
                msg.target_y,
                msg.target_yaw,
                msg.target_speed,
            ])
        else:
            setpoint_rows.append([
                timestamp,
                msg.guidance_mode,
                msg.target_yaw,
                msg.target_speed,
                msg.heading_error,
                msg.distance_error,
            ])
    gt = pd.DataFrame(
        gt_rows, columns=["t", "x", "y", "z", "vx", "vy", "vz", "yaw"]
    ).sort_values("t")
    goals = pd.DataFrame(
        goal_rows,
        columns=[
            "t", "mode", "target_x", "target_y", "target_yaw", "target_speed",
        ],
    ).sort_values("t")
    setpoints = pd.DataFrame(
        setpoint_rows,
        columns=[
            "t", "mode", "target_yaw", "target_speed",
            "heading_error", "distance_error",
        ],
    ).sort_values("t")
    return gt, goals, setpoints


def consecutive_goals(goals):
    active = goals[goals["mode"].isin(["LOS", "WAYPOINT"])].copy()
    changed = (
        active[["mode", "target_x", "target_y"]]
        != active[["mode", "target_x", "target_y"]].shift()
    ).any(axis=1)
    return active.loc[changed].reset_index(drop=True)


def relative_time(*frames):
    starts = [frame["t"].iloc[0] for frame in frames if not frame.empty]
    origin = min(starts)
    return [frame.assign(t=frame["t"] - origin) for frame in frames]


def active_reference(gt, goals):
    goals = goals[goals["mode"].isin(["LOS", "WAYPOINT"])].copy()
    if goals.empty:
        raise RuntimeError("Bag içinde aktif LOS/WAYPOINT hedefi bulunamadı.")
    references = pd.merge_asof(
        gt.sort_values("t"),
        goals.sort_values("t"),
        on="t",
        direction="backward",
    ).dropna(subset=["mode"])
    references["cross_track"] = np.nan
    references["along_track"] = np.nan
    previous_target = None
    segment_start = None
    for index, row in references.iterrows():
        target = (row["target_x"], row["target_y"])
        if target != previous_target:
            segment_start = (
                (row["x"], row["y"])
                if previous_target is None else previous_target
            )
            previous_target = target
        if row["mode"] == "LOS":
            axis = row["target_yaw"]
            dx = row["x"] - row["target_x"]
            dy = row["y"] - row["target_y"]
        else:
            dx = row["x"] - segment_start[0]
            dy = row["y"] - segment_start[1]
            axis = math.atan2(
                row["target_y"] - segment_start[1],
                row["target_x"] - segment_start[0],
            )
        references.at[index, "along_track"] = (
            dx * math.cos(axis) + dy * math.sin(axis)
        )
        references.at[index, "cross_track"] = (
            -dx * math.sin(axis) + dy * math.cos(axis)
        )
    return references


def trim_to_active_guidance_window(gt, goals, setpoints):
    """Exclude warm-up and post-STOP vehicle drift from guidance metrics."""
    active = goals[goals["mode"].isin(["LOS", "WAYPOINT"])]
    if active.empty:
        return gt, goals, setpoints
    active_start = float(active["t"].min())
    stop_after_start = goals[
        (goals["mode"] == "STOP") & (goals["t"] > active_start)
    ]
    active_end = (
        float(stop_after_start["t"].min())
        if not stop_after_start.empty
        else float(gt["t"].max())
    )
    return (
        gt[(gt["t"] >= active_start) & (gt["t"] <= active_end)].copy(),
        goals[(goals["t"] >= active_start) & (goals["t"] <= active_end)].copy(),
        setpoints[
            (setpoints["t"] >= active_start) & (setpoints["t"] <= active_end)
        ].copy(),
    )


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


def save_figures(gt, goals, setpoints, references, output, acceptance):
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    unique_goals = consecutive_goals(goals)
    mode = unique_goals["mode"].iloc[-1]

    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    ax.plot(gt["x"], gt["y"], color=CORAL, linewidth=2.0, label="Ground truth iz")
    if mode == "LOS":
        goal = unique_goals.iloc[-1]
        axis = goal["target_yaw"]
        along = (
            (gt["x"] - goal["target_x"]) * math.cos(axis)
            + (gt["y"] - goal["target_y"]) * math.sin(axis)
        )
        line_start = float(along.min()) - 2.0
        line_end = float(along.max()) + 2.0
        line_x = [
            goal["target_x"] + line_start * math.cos(axis),
            goal["target_x"] + line_end * math.cos(axis),
        ]
        line_y = [
            goal["target_y"] + line_start * math.sin(axis),
            goal["target_y"] + line_end * math.sin(axis),
        ]
        ax.plot(line_x, line_y, color=DARK, linestyle="--",
                linewidth=2.0, label="LOS referans ekseni")
    else:
        start = [gt["x"].iloc[0], gt["y"].iloc[0]]
        route_x = [start[0], *unique_goals["target_x"].tolist()]
        route_y = [start[1], *unique_goals["target_y"].tolist()]
        ax.plot(route_x, route_y, color=DARK, linestyle="--",
                linewidth=2.0, label="Waypoint referans rotası")
    for number, goal in enumerate(unique_goals.itertuples(), start=1):
        ax.scatter(goal.target_x, goal.target_y, color=BLUE, s=30, zorder=4)
        ax.add_patch(Circle(
            (goal.target_x, goal.target_y),
            acceptance,
            fill=False,
            color=BLUE,
            alpha=0.45,
            linestyle=":",
        ))
        ax.annotate(
            str(number),
            (goal.target_x, goal.target_y),
            xytext=(0, 8),
            textcoords="offset points",
            ha="center",
            color=DARK,
            fontweight="bold",
        )
    ax.set(xlabel="X (m)", ylabel="Y (m)")
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "guidance_path_tracking.png")
    plt.close(fig)

    active_setpoints = setpoints[
        setpoints["mode"].isin(["LOS", "WAYPOINT"])
    ].copy()
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 6.6), sharex=True)
    axes[0].plot(
        references["t"], references["cross_track"],
        color=CORAL, label="Cross-track hata",
    )
    axes[0].axhline(0.0, color=DARK, linestyle="--", linewidth=1.2)
    axes[0].set(ylabel="Yanal hata (m)")
    axes[0].legend()
    axes[1].plot(
        active_setpoints["t"],
        np.degrees(active_setpoints["heading_error"]),
        color=MUTED_RED,
        label="Heading hata",
    )
    axes[1].plot(
        active_setpoints["t"],
        active_setpoints["distance_error"],
        color=BLUE,
        label="Waypoint mesafe hatası",
    )
    axes[1].set(xlabel="Zaman (s)", ylabel="Hata")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / "guidance_error_history.png")
    plt.close(fig)

    speed = np.sqrt(gt["vx"] ** 2 + gt["vy"] ** 2 + gt["vz"] ** 2)
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 6.6), sharex=True)
    axes[0].plot(gt["t"], np.degrees(gt["yaw"]), color=DARK, label="Gerçek yaw")
    axes[0].plot(
        active_setpoints["t"],
        np.degrees(active_setpoints["target_yaw"]),
        color=CORAL,
        label="Hedef yaw",
    )
    axes[0].set(ylabel="Yaw (deg)")
    axes[0].legend()
    axes[1].plot(gt["t"], speed, color=BLUE, label="Gerçek hız")
    axes[1].plot(
        active_setpoints["t"],
        active_setpoints["target_speed"],
        color=MUTED_RED,
        label="Hedef hız",
    )
    axes[1].set(xlabel="Zaman (s)", ylabel="Hız (m/s)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / "guidance_command_tracking.png")
    plt.close(fig)


def rmse(values):
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def write_markdown(table, path):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("| " + " | ".join(table.columns) + " |\n")
        stream.write("|" + "|".join(["---"] * len(table.columns)) + "|\n")
        for row in table.itertuples(index=False, name=None):
            values = [
                f"{value:.4f}" if isinstance(value, float) else str(value)
                for value in row
            ]
            stream.write("| " + " | ".join(values) + " |\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--waypoint-acceptance", type=float, default=1.5)
    args = parser.parse_args()
    bag_input = args.bag.expanduser().resolve()
    bag = bag_input.parent if bag_input.is_file() else bag_input
    output = (args.output or bag.parent).expanduser().resolve()
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)

    gt, goals, setpoints = read_bag(bag)
    gt, goals, setpoints = relative_time(gt, goals, setpoints)
    gt, goals, setpoints = trim_to_active_guidance_window(
        gt, goals, setpoints
    )
    references = active_reference(gt, goals)
    unique_goals = consecutive_goals(goals)
    if unique_goals.empty or references.empty:
        raise RuntimeError(
            "Aktif LOS/WAYPOINT hedefi ground-truth zaman aralığıyla "
            "eşleştirilemedi."
        )
    active_setpoints = setpoints[
        setpoints["mode"].isin(["LOS", "WAYPOINT"])
    ]
    cross_track = references["cross_track"].dropna()
    heading_error = np.degrees(active_setpoints["heading_error"])
    final_goal = unique_goals.iloc[-1]
    final_distance = math.hypot(
        gt["x"].iloc[-1] - final_goal["target_x"],
        gt["y"].iloc[-1] - final_goal["target_y"],
    )
    initial_cross_track = float(cross_track.iloc[0])
    final_cross_track = float(cross_track.iloc[-1])
    cross_track_reduction = (
        1.0 - abs(final_cross_track) / abs(initial_cross_track)
        if abs(initial_cross_track) > 1e-6
        else math.nan
    )
    if final_goal["mode"] == "LOS":
        accepted = abs(final_cross_track) < abs(initial_cross_track)
        decision = (
            "KABUL - LOS rota eksenine yakınsadı"
            if accepted else "BAŞARISIZ - LOS yanal hatayı azaltmadı"
        )
    else:
        previous_goal = (
            unique_goals.iloc[-2]
            if len(unique_goals) > 1
            else pd.Series({
                "target_x": gt["x"].iloc[0],
                "target_y": gt["y"].iloc[0],
            })
        )
        segment_x = final_goal["target_x"] - previous_goal["target_x"]
        segment_y = final_goal["target_y"] - previous_goal["target_y"]
        segment_length = math.hypot(segment_x, segment_y)
        axis_x = segment_x / max(segment_length, 1e-12)
        axis_y = segment_y / max(segment_length, 1e-12)
        relative_x = gt["x"].iloc[-1] - previous_goal["target_x"]
        relative_y = gt["y"].iloc[-1] - previous_goal["target_y"]
        final_along_track = relative_x * axis_x + relative_y * axis_y
        final_segment_cross_track = -relative_x * axis_y + relative_y * axis_x
        accepted = (
            final_distance <= args.waypoint_acceptance
            or (
                final_along_track
                >= segment_length - args.waypoint_acceptance
                and abs(final_segment_cross_track)
                <= 2.0 * args.waypoint_acceptance
            )
        )
        decision = (
            "KABUL - waypoint rotası tamamlandı"
            if accepted else "BAŞARISIZ - son waypoint kabul edilmedi"
        )
    summary = pd.DataFrame([{
        "Güdüm modu": final_goal["mode"],
        "Doğrulama kararı": decision,
        "Waypoint sayısı": len(unique_goals),
        "Cross-track RMSE (m)": rmse(cross_track),
        "Maksimum mutlak cross-track hata (m)": float(cross_track.abs().max()),
        "Başlangıç cross-track hata (m)": initial_cross_track,
        "Son cross-track hata (m)": final_cross_track,
        "Cross-track azalma oranı": cross_track_reduction,
        "Heading hata RMSE (deg)": rmse(heading_error),
        "Maksimum mutlak heading hata (deg)": float(heading_error.abs().max()),
        "Son waypoint mesafesi (m)": final_distance,
        "Test süresi (s)": float(gt["t"].iloc[-1] - gt["t"].iloc[0]),
    }])
    configure_plot()
    save_figures(
        gt, goals, setpoints, references, output, args.waypoint_acceptance
    )
    references.to_csv(metrics / "guidance_aligned_reference.csv", index=False)
    unique_goals.to_csv(metrics / "guidance_waypoints.csv", index=False)
    summary.to_csv(metrics / "guidance_validation_summary.csv", index=False)
    write_markdown(summary, metrics / "guidance_validation_summary.md")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
