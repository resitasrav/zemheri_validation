#!/usr/bin/env python3
"""Compare raw and protected UKF paths against simulation ground truth."""

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

GT_TOPIC = "/ground_truth/odometry"
RAW_TOPIC = "/validation/resilience/raw_ukf"
PROTECTED_TOPIC = "/validation/resilience/protected_ukf"
OOSM_TOPIC = "/validation/resilience/oosm_ukf"
STATUS_TOPIC = "/validation/resilience/status"


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def stamp_seconds(msg, bag_time):
    stamp = msg.header.stamp
    value = stamp.sec + stamp.nanosec * 1e-9
    return value if value > 0.0 else bag_time * 1e-9


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
    required = [GT_TOPIC, RAW_TOPIC, PROTECTED_TOPIC, OOSM_TOPIC]
    missing = [topic for topic in required if topic not in types]
    if missing:
        missing_text = ", ".join(missing)
        raise RuntimeError(f"Bag içinde gerekli topicler yok: {missing_text}")
    message_types = {
        topic: get_message(types[topic])
        for topic in required
        + ([STATUS_TOPIC] if STATUS_TOPIC in types else [])
    }
    odometry = {topic: [] for topic in required}
    status = []
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(raw, message_types[topic])
        timestamp = stamp_seconds(msg, bag_time)
        if topic == STATUS_TOPIC:
            status.append([
                timestamp,
                msg.navigation_valid,
                msg.navigation_degraded,
                msg.failsafe_required,
                msg.dvl_ok,
            ])
            continue
        pose = msg.pose.pose
        velocity = msg.twist.twist.linear
        odometry[topic].append([
            timestamp,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            velocity.x,
            velocity.y,
            velocity.z,
            yaw_from_quaternion(pose.orientation),
        ])
    columns = ["t", "x", "y", "z", "vx", "vy", "vz", "yaw"]
    frames = {
        topic: pd.DataFrame(rows, columns=columns).sort_values("t")
        for topic, rows in odometry.items()
    }
    status_frame = pd.DataFrame(
        status,
        columns=[
            "t", "navigation_valid", "navigation_degraded",
            "failsafe", "dvl_ok",
        ],
    )
    return frames, status_frame


def align(gt, estimate):
    query = estimate["t"].to_numpy()
    valid = (query >= gt["t"].iloc[0]) & (query <= gt["t"].iloc[-1])
    output = estimate.loc[valid].copy()
    query = output["t"].to_numpy()
    for column in ["x", "y", "z", "vx", "vy", "vz"]:
        output[f"gt_{column}"] = np.interp(query, gt["t"], gt[column])
    output["gt_yaw"] = np.interp(query, gt["t"], np.unwrap(gt["yaw"]))
    for prefix in ["", "gt_"]:
        output[f"{prefix}x"] -= output[f"{prefix}x"].iloc[0]
        output[f"{prefix}y"] -= output[f"{prefix}y"].iloc[0]
    output["t"] -= output["t"].iloc[0]
    return output


def error_series(data):
    position = np.sqrt(
        (data["x"] - data["gt_x"]) ** 2
        + (data["y"] - data["gt_y"]) ** 2
        + (data["z"] - data["gt_z"]) ** 2
    )
    horizontal = np.sqrt(
        (data["x"] - data["gt_x"]) ** 2 + (data["y"] - data["gt_y"]) ** 2
    )
    depth = data["z"] - data["gt_z"]
    speed = np.sqrt(data["vx"] ** 2 + data["vy"] ** 2 + data["vz"] ** 2)
    gt_speed = np.sqrt(
        data["gt_vx"] ** 2 + data["gt_vy"] ** 2 + data["gt_vz"] ** 2
    )
    yaw = np.arctan2(
        np.sin(data["yaw"] - data["gt_yaw"]),
        np.cos(data["yaw"] - data["gt_yaw"]),
    )
    return position, horizontal, depth, speed - gt_speed, yaw


def rmse(values):
    return float(np.sqrt(np.mean(np.asarray(values) ** 2)))


def summarize(label, data):
    position, horizontal, depth, speed, yaw = error_series(data)
    return {
        "Yapı": label,
        "Örnek sayısı": len(data),
        "3B konum RMSE (m)": rmse(position),
        "Maksimum 3B hata (m)": float(position.max()),
        "Yatay konum RMSE (m)": rmse(horizontal),
        "Derinlik RMSE (m)": rmse(depth),
        "Hız RMSE (m/s)": rmse(speed),
        "Yaw RMSE (deg)": math.degrees(rmse(yaw)),
    }


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


def save_figures(raw, protected, oosm, status, output):
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    raw_error = error_series(raw)[0]
    protected_error = error_series(protected)[0]
    oosm_error = error_series(oosm)[0]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    axes[0].plot(raw["gt_x"], raw["gt_y"], color=DARK, linestyle="--",
                 linewidth=2.0, label="Ground truth")
    axes[0].plot(raw["x"], raw["y"], color=MUTED_RED, label="Saf UKF")
    axes[0].plot(protected["x"], protected["y"], color=BLUE,
                 label="Sağlık denetimli UKF")
    axes[0].plot(oosm["x"], oosm["y"], color=CORAL,
                 label="OOSM etkin UKF")
    axes[0].set(xlabel="Bağıl X (m)", ylabel="Bağıl Y (m)")
    axes[0].legend()
    axes[1].plot(raw["t"], raw["gt_z"], color=DARK, linestyle="--",
                 linewidth=2.0, label="Ground truth")
    axes[1].plot(raw["t"], raw["z"], color=MUTED_RED, label="Saf UKF")
    axes[1].plot(protected["t"], protected["z"], color=BLUE,
                 label="Sağlık denetimli UKF")
    axes[1].plot(oosm["t"], oosm["z"], color=CORAL,
                 label="OOSM etkin UKF")
    axes[1].set(xlabel="Zaman (s)", ylabel="Z (m)")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / "trajectory_and_depth_comparison.png")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.2, 4.8))
    ax.plot(
        raw["t"], raw_error, color=MUTED_RED,
        label=f"Saf UKF | RMSE {rmse(raw_error):.3f} m",
    )
    ax.plot(
        protected["t"], protected_error, color=BLUE,
        label=f"Sağlık denetimli UKF | RMSE {rmse(protected_error):.3f} m",
    )
    ax.plot(
        oosm["t"], oosm_error, color=CORAL,
        label=f"OOSM etkin UKF | RMSE {rmse(oosm_error):.3f} m",
    )
    ax.set(xlabel="Zaman (s)", ylabel="3B konum hatası (m)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "position_error_comparison.png")
    plt.close(fig)

    if status.empty:
        return
    timeline = status.copy()
    timeline["t"] -= timeline["t"].iloc[0]
    fig, ax = plt.subplots(figsize=(9.2, 4.4))
    ax.step(timeline["t"], timeline["dvl_ok"].astype(int), where="post",
            color=DARK, label="DVL geçerli")
    ax.step(
        timeline["t"], timeline["navigation_valid"].astype(int), where="post",
        color=BLUE, label="Navigasyon geçerli",
    )
    ax.step(
        timeline["t"], timeline["navigation_degraded"].astype(int),
        where="post",
        color=CORAL, label="Degraded",
    )
    ax.step(timeline["t"], timeline["failsafe"].astype(int), where="post",
            color=MUTED_RED, label="Failsafe gerekli")
    ax.set(xlabel="Zaman (s)", ylabel="Durum", yticks=[0, 1])
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(figures / "protected_navigation_status.png")
    plt.close(fig)


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


def write_assessment(rows, status, metrics):
    raw_rmse = rows[0]["3B konum RMSE (m)"]
    protected_rmse = rows[1]["3B konum RMSE (m)"]
    oosm_rmse = rows[2]["3B konum RMSE (m)"]
    raw_max_error = rows[0]["Maksimum 3B hata (m)"]
    oosm_max_error = rows[2]["Maksimum 3B hata (m)"]
    protected_ratio = protected_rmse / max(raw_rmse, 1e-12)
    oosm_ratio = oosm_rmse / max(raw_rmse, 1e-12)
    oosm_max_ratio = oosm_max_error / max(raw_max_error, 1e-12)
    oosm_accepted = oosm_ratio <= 1.05 and oosm_max_ratio <= 1.10
    operational_status = status
    if not status.empty and status["navigation_valid"].any():
        first_valid = status.index[status["navigation_valid"]].min()
        operational_status = status.loc[first_valid:]
    health_valid_ratio = (
        float(operational_status["navigation_valid"].mean())
        if not operational_status.empty else math.nan
    )
    health_degraded_ratio = (
        float(operational_status["navigation_degraded"].mean())
        if not operational_status.empty else math.nan
    )
    dvl_valid_ratio = (
        float(operational_status["dvl_ok"].mean())
        if not operational_status.empty else math.nan
    )
    if health_valid_ratio >= 0.99 and health_degraded_ratio > 0.0:
        health_assessment = (
            "BAŞARILI - DVL kesintileri degraded olarak yönetildi"
        )
    elif health_valid_ratio >= 0.99 and dvl_valid_ratio >= 0.99:
        health_assessment = (
            "BAŞARILI - Kesintisiz DVL akışında navigasyon geçerli kaldı"
        )
    else:
        health_assessment = (
            "İNCELENMELİ - sağlık geçişleri beklenen koşulları sağlamadı"
        )
    assessment = pd.DataFrame([{
        "OOSM performans kararı": (
            "KABUL - OOSM ortalama ve maksimum hata sınırları içinde"
            if oosm_accepted
            else "BAŞARISIZ - OOSM ortalama veya maksimum hatayı fazla artırdı"
        ),
        "OOSM/Saf konum RMSE oranı": oosm_ratio,
        "OOSM/Saf maksimum hata oranı": oosm_max_ratio,
        "Sağlık denetimli/Saf konum RMSE oranı": protected_ratio,
        "Navigasyon sağlık kararı": health_assessment,
        "Navigasyon geçerli oranı": health_valid_ratio,
        "Degraded oranı": health_degraded_ratio,
        "Rapor notu": (
            (
                "OOSM bu test koşulunda doğruluk kazanımı sağlamıştır; sonuç "
                "yalnız test edilen gecikme ve kesinti koşulu için geçerlidir."
                if oosm_ratio < 0.95
                else (
                    "OOSM bu testte doğruluk performansını korumuştur; "
                    "sonuç yalnız test edilen koşul için geçerlidir."
                )
            )
            if oosm_accepted
            else "OOSM performansı iyileştirilmeden doğruluk kazanımı olarak "
            "raporlanmamalıdır; sağlık/degraded davranışı ayrı sonuçtur."
        ),
    }])
    assessment.to_csv(
        metrics / "navigation_resilience_assessment.csv", index=False
    )
    write_markdown(
        assessment, metrics / "navigation_resilience_assessment.md"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    bag_input = args.bag.expanduser().resolve()
    bag = bag_input.parent if bag_input.is_file() else bag_input
    default_output = bag.parent
    output = (args.output or default_output).expanduser().resolve()
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)

    frames, status = read_bag(bag)
    raw = align(frames[GT_TOPIC], frames[RAW_TOPIC])
    protected = align(frames[GT_TOPIC], frames[PROTECTED_TOPIC])
    oosm = align(frames[GT_TOPIC], frames[OOSM_TOPIC])
    rows = [summarize("Saf robot_localization UKF", raw),
            summarize("Sağlık denetimli UKF", protected),
            summarize("OOSM etkin UKF", oosm)]
    table = pd.DataFrame(rows)
    raw_position_rmse = rows[0]["3B konum RMSE (m)"]
    table["Saf UKF'ye göre 3B konum RMSE oranı"] = (
        table["3B konum RMSE (m)"] / max(raw_position_rmse, 1e-12)
    )
    if not status.empty:
        table["Navigasyon geçerli oranı"] = float(
            status["navigation_valid"].mean()
        )
        table["Degraded oranı"] = float(status["navigation_degraded"].mean())
        table["Failsafe gerekli oranı"] = float(status["failsafe"].mean())

    configure_plot()
    save_figures(raw, protected, oosm, status, output)
    raw.to_csv(metrics / "aligned_raw_ukf.csv", index=False)
    protected.to_csv(metrics / "aligned_protected_ukf.csv", index=False)
    oosm.to_csv(metrics / "aligned_oosm_ukf.csv", index=False)
    status.to_csv(metrics / "protected_navigation_status.csv", index=False)
    table.to_csv(metrics / "navigation_resilience_summary.csv", index=False)
    write_markdown(table, metrics / "navigation_resilience_summary.md")
    write_assessment(rows, status, metrics)
    print(table.to_string(index=False))


if __name__ == "__main__":
    main()
