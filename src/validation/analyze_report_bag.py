#!/usr/bin/env python3
"""Generate report metrics and figures from a report-validation rosbag."""

import argparse
import json
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


def rmse(values):
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def stamp_seconds(msg, bag_time):
    if hasattr(msg, "header"):
        value = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if value > 0.0:
            return value
    return bag_time * 1e-9


def quaternion_to_euler(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def read_odometry(path):
    reader = open_reader(path)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    odom_topics = {
        topic: get_message(msg_type)
        for topic, msg_type in types.items()
        if msg_type == "nav_msgs/msg/Odometry"
    }
    rows = {topic: [] for topic in odom_topics}
    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic not in odom_topics:
            continue
        msg = deserialize_message(data, odom_topics[topic])
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        roll, pitch, yaw = quaternion_to_euler(q)
        rows[topic].append([
            stamp_seconds(msg, bag_time), p.x, p.y, p.z,
            v.x, v.y, v.z, roll, pitch, yaw,
        ])
    columns = [
        "t", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
    ]
    return {
        topic: pd.DataFrame(values, columns=columns).sort_values("t")
        for topic, values in rows.items() if values
    }


def read_turn_evidence(path):
    reader = open_reader(path)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {
        topic: get_message(types[topic])
        for topic in [
            "/auv/mission/status",
            "/guidance/goal",
            "/sara_uuv/propeller/cmd_angvel",
        ]
        if topic in types
    }
    phases = []
    goals = []
    propeller = []
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in selected:
            continue
        msg = deserialize_message(raw, selected[topic])
        timestamp = stamp_seconds(msg, bag_time)
        if topic == "/auv/mission/status":
            phases.append([timestamp, msg.stage_name])
        elif topic == "/guidance/goal":
            goals.append([timestamp, msg.guidance_mode, msg.target_speed])
        else:
            propeller.append([timestamp, msg.data])
    return (
        pd.DataFrame(phases, columns=["t", "phase"]),
        pd.DataFrame(goals, columns=["t", "mode", "target_speed"]),
        pd.DataFrame(propeller, columns=["t", "propeller_rad_s"]),
    )


def relative(frame):
    result = frame.copy()
    for column in ["x", "y"]:
        result[column] -= result[column].iloc[0]
    result["t"] -= result["t"].iloc[0]
    return result


def align(gt, ukf):
    gt = relative(gt)
    ukf = relative(ukf)
    query = ukf["t"].to_numpy()
    output = ukf.copy()
    for column in ["x", "y", "z", "vx", "vy", "vz"]:
        output[f"gt_{column}"] = np.interp(query, gt["t"], gt[column])
    for column in ["roll", "pitch", "yaw"]:
        output[f"gt_{column}"] = np.interp(
            query, gt["t"], np.unwrap(gt[column].to_numpy())
        )
    return output


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


def configure_trajectory_axes(ax, data):
    x_span = float(data["gt_x"].max() - data["gt_x"].min())
    y_span = float(data["gt_y"].max() - data["gt_y"].min())
    if x_span > 5.0 * max(y_span, 1e-6):
        ax.set_aspect("auto")
        return "Ground truth ve UKF bağıl X-Y rotası (Y ekseni büyütülmüştür)"
    ax.axis("equal")
    return "Ground truth ve UKF bağıl X-Y rotası"


def save_turn_evidence(path, output):
    phases, goals, propeller = read_turn_evidence(path)
    if phases.empty or goals.empty or propeller.empty:
        return {}
    turn_times = phases.loc[phases["phase"] == "TURN_AROUND", "t"]
    if turn_times.empty:
        return {}

    turn_start = float(turn_times.min())
    turn_end = float(turn_times.max())
    turn_goals = goals[(goals["t"] >= turn_start) & (goals["t"] <= turn_end)]
    turn_propeller = propeller[
        (propeller["t"] >= turn_start) & (propeller["t"] <= turn_end)
    ]
    if turn_goals.empty or turn_propeller.empty:
        return {}

    t0 = min(float(goals["t"].min()), float(propeller["t"].min()))
    figures = output / "figures"
    metrics = output / "metrics"
    fig, speed_ax = plt.subplots(figsize=(9.2, 4.8))
    propeller_ax = speed_ax.twinx()
    speed_ax.plot(
        goals["t"] - t0, goals["target_speed"], color=BLUE,
        linewidth=1.8, label="Hedef ileri hız",
    )
    propeller_ax.plot(
        propeller["t"] - t0, propeller["propeller_rad_s"], color=CORAL,
        linewidth=1.4, alpha=0.85, label="Pervane açısal hızı",
    )
    speed_ax.axvspan(
        turn_start - t0, turn_end - t0, color=MUTED_RED, alpha=0.16,
        label="TURN_AROUND",
    )
    speed_ax.set(xlabel="Zaman (s)", ylabel="Hedef ileri hız (m/s)")
    propeller_ax.set_ylabel("Pervane açısal hızı (rad/s)")
    speed_handles, speed_labels = speed_ax.get_legend_handles_labels()
    prop_handles, prop_labels = propeller_ax.get_legend_handles_labels()
    speed_ax.legend(
        speed_handles + prop_handles, speed_labels + prop_labels, loc="best"
    )
    fig.tight_layout()
    fig.savefig(figures / "turn_thrust_evidence.png")
    plt.close(fig)

    turn_goals.to_csv(metrics / "turn_guidance_goal.csv", index=False)
    turn_propeller.to_csv(metrics / "turn_propeller_command.csv", index=False)
    return {
        "turn_duration_sec": turn_end - turn_start,
        "turn_target_speed_mean_mps": float(turn_goals["target_speed"].mean()),
        "turn_target_speed_min_mps": float(turn_goals["target_speed"].min()),
        "turn_propeller_nonzero_ratio": float(
            (turn_propeller["propeller_rad_s"].abs() > 1e-3).mean()
        ),
    }


def save_figures(data, output):
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    position_error = np.sqrt(
        (data["x"] - data["gt_x"]) ** 2 +
        (data["y"] - data["gt_y"]) ** 2 +
        (data["z"] - data["gt_z"]) ** 2
    )
    depth_error = data["z"] - data["gt_z"]
    yaw_error = np.arctan2(
        np.sin(data["yaw"] - data["gt_yaw"]),
        np.cos(data["yaw"] - data["gt_yaw"]),
    )
    roll_error = np.arctan2(
        np.sin(data["roll"] - data["gt_roll"]),
        np.cos(data["roll"] - data["gt_roll"]),
    )
    pitch_error = np.arctan2(
        np.sin(data["pitch"] - data["gt_pitch"]),
        np.cos(data["pitch"] - data["gt_pitch"]),
    )
    speed = np.sqrt(data["vx"] ** 2 + data["vy"] ** 2 + data["vz"] ** 2)
    gt_speed = np.sqrt(
        data["gt_vx"] ** 2 + data["gt_vy"] ** 2 + data["gt_vz"] ** 2
    )
    speed_error = speed - gt_speed

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    axes[0].plot(data["gt_x"], data["gt_y"], color=DARK, label="Ground truth")
    axes[0].plot(
        data["x"], data["y"], color=CORAL,
        label=f"UKF | 3B konum RMSE: {rmse(position_error):.3f} m",
    )
    axes[0].set(xlabel="X (m)", ylabel="Y (m)")
    axes[0].set_title(configure_trajectory_axes(axes[0], data))
    axes[0].legend()
    axes[1].plot(data["t"], data["gt_z"], color=DARK, label="Ground truth")
    axes[1].plot(
        data["t"], data["z"], color=CORAL,
        label=f"UKF | Derinlik RMSE: {rmse(depth_error):.3f} m",
    )
    axes[1].set_title("Derinlik takibi")
    axes[1].set(xlabel="Zaman (s)", ylabel="Z (m)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / "trajectory_and_depth.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2, 6.2))
    ax.plot(data["gt_x"], data["gt_y"], color=DARK, linestyle="--",
            linewidth=2.2, label="Ground truth")
    ax.plot(
        data["x"], data["y"], color=CORAL, linewidth=1.8,
        label=f"UKF | 3B konum RMSE: {rmse(position_error):.3f} m",
    )
    ax.scatter(data["gt_x"].iloc[0], data["gt_y"].iloc[0], color=BLUE,
               marker="o", s=55, label="Başlangıç", zorder=3)
    ax.scatter(data["gt_x"].iloc[-1], data["gt_y"].iloc[-1], color=MUTED_RED,
               marker="X", s=65, label="Bitiş", zorder=3)
    ax.set(xlabel="Bağıl X (m)", ylabel="Bağıl Y (m)")
    ax.set_title(configure_trajectory_axes(ax, data))
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "trajectory_xy.png")
    plt.close(fig)

    fig = plt.figure(figsize=(8.2, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(data["gt_x"], data["gt_y"], data["gt_z"], color=DARK,
            linestyle="--", linewidth=2.2, label="Ground truth")
    ax.plot(
        data["x"], data["y"], data["z"], color=CORAL,
        linewidth=1.8,
        label=f"UKF | 3B konum RMSE: {rmse(position_error):.3f} m",
    )
    ax.set_title("Ground truth ve UKF üç boyutlu rota karşılaştırması")
    ax.set(xlabel="Bağıl X (m)", ylabel="Bağıl Y (m)", zlabel="Z (m)")
    ax.legend()
    ax.view_init(elev=22, azim=-58)
    fig.tight_layout()
    fig.savefig(figures / "trajectory_3d.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    ax.plot(data["t"], data["gt_z"], color=DARK, linestyle="--",
            linewidth=2.2, label="Ground truth")
    ax.plot(
        data["t"], data["z"], color=CORAL, linewidth=1.8,
        label=f"UKF | RMSE: {rmse(depth_error):.3f} m",
    )
    ax.axhline(0.0, color=MUTED_RED, linestyle=":", linewidth=1.0,
               label="Su yüzeyi")
    ax.set(xlabel="Zaman (s)", ylabel="Z (m)")
    ax.set_title("Ground truth ve UKF derinlik takibi")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "depth_tracking.png")
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    axes[0].plot(
        data["t"], position_error, color=MUTED_RED,
        label=(
            f"RMSE: {rmse(position_error):.3f} m | "
            f"Maksimum: {float(position_error.max()):.3f} m"
        ),
    )
    axes[0].set_title("UKF üç boyutlu konum hatası")
    axes[0].set(xlabel="Zaman (s)", ylabel="3B konum hatası (m)")
    axes[0].legend()
    axes[1].plot(data["t"], gt_speed, color=DARK, label="Ground truth")
    axes[1].plot(
        data["t"], speed, color=BLUE,
        label=f"UKF | Hız RMSE: {rmse(speed_error):.3f} m/s",
    )
    axes[1].set_title("Toplam hız büyüklüğü karşılaştırması")
    axes[1].set(xlabel="Zaman (s)", ylabel="Hız (m/s)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / "navigation_error_and_speed.png")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9.2, 8.4), sharex=True)
    for ax, column, label in zip(
        axes, ["vx", "vy", "vz"], ["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"]
    ):
        ax.plot(data["t"], data[f"gt_{column}"], color=DARK,
                linestyle="--", linewidth=2.0, label="Ground truth")
        component_error = data[column] - data[f"gt_{column}"]
        ax.plot(
            data["t"], data[column], color=BLUE, linewidth=1.6,
            label=f"UKF | RMSE: {rmse(component_error):.3f} m/s",
        )
        ax.set_title(f"{label.split()[0]} hız bileşeni karşılaştırması")
        ax.set_ylabel(label)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Zaman (s)")
    fig.tight_layout()
    fig.savefig(figures / "velocity_components.png")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9.2, 8.4), sharex=True)
    for ax, error, label, color in zip(
        axes,
        [roll_error, pitch_error, yaw_error],
        ["Roll hatası (°)", "Pitch hatası (°)", "Yaw hatası (°)"],
        [BLUE, CORAL, MUTED_RED],
    ):
        error_deg = np.degrees(error)
        ax.plot(
            data["t"], error_deg, color=color, linewidth=1.7,
            label=(
                f"RMSE: {rmse(error_deg):.3f}° | "
                f"Maksimum: {float(np.max(np.abs(error_deg))):.3f}°"
            ),
        )
        ax.axhline(0.0, color=DARK, linestyle="--", linewidth=0.8)
        ax.set_ylabel(label)
        ax.set_title(f"{label.replace(' (°)', '')} zaman geçmişi")
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Zaman (s)")
    fig.tight_layout()
    fig.savefig(figures / "orientation_errors.png")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9.2, 8.4), sharex=True)
    for ax, column, label, color in zip(
        axes,
        ["gt_roll", "gt_pitch", "gt_yaw"],
        ["Gerçek roll (°)", "Gerçek pitch (°)", "Gerçek yaw (°)"],
        [BLUE, CORAL, MUTED_RED],
    ):
        values = np.degrees(data[column])
        ax.plot(
            data["t"], values, color=color, linewidth=1.7,
            label=f"Maksimum mutlak değer: {float(np.max(np.abs(values))):.2f}°",
        )
        ax.axhline(0.0, color=DARK, linestyle="--", linewidth=0.8)
        ax.set_ylabel(label)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("Zaman (s)")
    fig.tight_layout()
    fig.savefig(figures / "vehicle_attitude_ground_truth.png")
    plt.close(fig)
    return position_error, speed_error, roll_error, pitch_error, yaw_error


def save_report_summary(summary, metrics):
    descriptions = [
        ("Örnek sayısı", "samples", "adet", "Analizde kullanılan hizalı UKF örneği"),
        ("Test süresi", "duration_sec", "s", "Analiz edilen kayıt süresi"),
        (
            "3B konum RMSE", "position_rmse_m", "m",
            "UKF ile ground truth arasındaki ortalama 3B konum hata büyüklüğü",
        ),
        (
            "Maksimum 3B konum hatası", "position_max_error_m", "m",
            "Test boyunca gözlenen en yüksek 3B konum hatası",
        ),
        (
            "Derinlik RMSE", "depth_rmse_m", "m",
            "UKF ve ground truth Z değerleri arasındaki hata",
        ),
        (
            "Toplam hız RMSE", "speed_rmse_mps", "m/s",
            "UKF ve ground truth toplam hız büyüklükleri arasındaki hata",
        ),
        ("Roll RMSE", "roll_rmse_deg", "°", "Roll kestirim hatası"),
        ("Pitch RMSE", "pitch_rmse_deg", "°", "Pitch kestirim hatası"),
        ("Yaw RMSE", "yaw_rmse_deg", "°", "Heading/yaw kestirim hatası"),
        (
            "Maksimum yaw hatası", "yaw_max_error_deg", "°",
            "Test boyunca gözlenen en yüksek mutlak yaw hatası",
        ),
        (
            "Gerçek maksimum mutlak roll", "gt_max_abs_roll_deg", "°",
            "Ground truth üzerinde gözlenen araç roll açısı; kestirim hatası değildir",
        ),
        (
            "Gerçek maksimum mutlak pitch", "gt_max_abs_pitch_deg", "°",
            "Ground truth üzerinde gözlenen araç pitch açısı; kestirim hatası değildir",
        ),
        (
            "Maksimum rota doğrultusu mesafesi", "max_along_track_m", "m",
            "Başlangıç heading doğrultusunda ulaşılan en ileri konum",
        ),
        (
            "Maksimum mutlak yanal sapma", "max_abs_cross_track_m", "m",
            "Başlangıç doğrultusuna göre gözlenen en yüksek yanal sapma",
        ),
        (
            "Bitiş çizgisi boyuna hatası", "stage1_finish_line_error_m", "m",
            "Son konum ile başlangıçtan 10 m ilerideki bitiş çizgisi arasındaki boyuna hata",
        ),
        (
            "Bitiş çizgisi yanal hatası", "stage1_finish_cross_track_m", "m",
            "Son konumun görev doğrultusuna göre yanal hatası",
        ),
    ]
    rows = []
    for name, key, unit, explanation in descriptions:
        if key not in summary:
            continue
        value = summary[key]
        formatted = f"{value:.4f}" if isinstance(value, float) else str(value)
        rows.append({
            "Metrik": name,
            "Değer": formatted,
            "Birim": unit,
            "Açıklama": explanation,
            "Okuma Notu": (
                "Kayıt kapsamını gösterir"
                if key in {"samples", "duration_sec"}
                else "Düşük değer daha iyi; simülasyon ground truth referanslıdır"
            ),
        })
    table = pd.DataFrame(rows)
    table.to_csv(metrics / "report_summary_table.csv", index=False)
    with open(metrics / "report_summary_table.md", "w", encoding="utf-8") as stream:
        stream.write("| " + " | ".join(table.columns) + " |\n")
        stream.write("|" + "|".join(["---"] * len(table.columns)) + "|\n")
        for row in table.itertuples(index=False, name=None):
            stream.write("| " + " | ".join(str(value) for value in row) + " |\n")

    narrative = [
        "# Navigasyon Doğrulama Sonuç Özeti",
        "",
        (
            f"Analiz, {summary['duration_sec']:.2f} saniyelik kayıt boyunca "
            f"{summary['samples']} hizalı UKF örneği kullanılarak yapılmıştır."
        ),
        "",
        (
            f"UKF konum kestiriminin ground truth referansına göre 3B konum "
            f"RMSE değeri {summary['position_rmse_m']:.4f} m, test boyunca "
            f"gözlenen maksimum konum hatası {summary['position_max_error_m']:.4f} m'dir."
        ),
        "",
        (
            f"Derinlik RMSE değeri {summary['depth_rmse_m']:.4f} m ve toplam "
            f"hız RMSE değeri {summary['speed_rmse_mps']:.4f} m/s olarak hesaplanmıştır."
        ),
        "",
        (
            f"Yönelim kestiriminde roll, pitch ve yaw RMSE değerleri sırasıyla "
            f"{summary['roll_rmse_deg']:.4f}°, {summary['pitch_rmse_deg']:.4f}° "
            f"ve {summary['yaw_rmse_deg']:.4f}°'dir. Maksimum yaw hatası "
            f"{summary['yaw_max_error_deg']:.4f}° olarak gözlenmiştir."
        ),
        "",
        (
            "Bu değerler simülasyon ground truth referansına göre algoritma "
            "doğrulama sonucudur; gerçek saha performansı olarak yorumlanmamalıdır."
        ),
        "",
    ]
    with open(metrics / "report_result_summary.md", "w", encoding="utf-8") as stream:
        stream.write("\n".join(narrative))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    output = args.output or bag.parent
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)

    odometry = read_odometry(bag)
    gt = odometry.get("/ground_truth/odometry")
    ukf = odometry.get("/odometry/ukf")
    if gt is None or ukf is None:
        raise RuntimeError("Bag içinde ground truth ve /odometry/ukf bulunmalıdır.")

    configure_plot()
    data = align(gt, ukf)
    position_error, speed_error, roll_error, pitch_error, yaw_error = (
        save_figures(data, output)
    )
    turn_summary = save_turn_evidence(bag, output)
    summary = {
        "samples": int(len(data)),
        "duration_sec": float(data["t"].iloc[-1]),
        "position_rmse_m": float(np.sqrt(np.mean(position_error ** 2))),
        "position_max_error_m": float(np.max(position_error)),
        "depth_rmse_m": float(np.sqrt(np.mean((data["z"] - data["gt_z"]) ** 2))),
        "speed_rmse_mps": float(np.sqrt(np.mean(speed_error ** 2))),
        "roll_rmse_deg": float(np.degrees(np.sqrt(np.mean(roll_error ** 2)))),
        "pitch_rmse_deg": float(np.degrees(np.sqrt(np.mean(pitch_error ** 2)))),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
        "yaw_max_error_deg": float(np.degrees(np.max(np.abs(yaw_error)))),
        "gt_max_abs_roll_deg": float(np.degrees(np.max(np.abs(data["gt_roll"])))),
        "gt_max_abs_pitch_deg": float(np.degrees(np.max(np.abs(data["gt_pitch"])))),
        **turn_summary,
    }
    initial_yaw = float(data["gt_yaw"].iloc[0])
    along_track = (
        data["gt_x"] * math.cos(initial_yaw)
        + data["gt_y"] * math.sin(initial_yaw)
    )
    cross_track = (
        -data["gt_x"] * math.sin(initial_yaw)
        + data["gt_y"] * math.cos(initial_yaw)
    )
    summary.update({
        "max_along_track_m": float(along_track.max()),
        "max_abs_cross_track_m": float(cross_track.abs().max()),
    })
    if output.name.startswith("stage1_fsm"):
        summary.update({
            "stage1_finish_line_error_m": float(abs(along_track.iloc[-1] - 10.0)),
            "stage1_finish_cross_track_m": float(abs(cross_track.iloc[-1])),
            "stage1_outbound_50m_valid": bool(along_track.max() >= 60.0),
            "stage1_finish_line_valid": bool(
                abs(along_track.iloc[-1] - 10.0) <= 2.0
                and abs(cross_track.iloc[-1]) <= 3.0
            ),
        })
    if output.name.startswith("stage2_bt"):
        summary["stage2_pitch_over_30_deg_valid"] = bool(
            np.degrees(data["gt_pitch"]).min() <= -30.0
        )
    data.to_csv(metrics / "aligned_navigation.csv", index=False)
    pd.DataFrame([summary]).to_csv(metrics / "summary.csv", index=False)
    save_report_summary(summary, metrics)
    with open(metrics / "summary.json", "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
