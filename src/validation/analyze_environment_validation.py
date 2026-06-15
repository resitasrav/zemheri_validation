#!/usr/bin/env python3
"""Validate simulated sensors and deterministic ocean-current response."""

import argparse
import math
from pathlib import Path

import matplotlib
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

SENSOR_TOPICS = [
    "/imu/data",
    "/dvl/raw",
    "/dvl/quality_twist",
    "/pressure/raw",
    "/pressure/depth_pose",
    "/battery/state",
    "/navigation/status",
]


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def read_bag(path):
    reader = open_reader(path)
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = {
        topic: get_message(types[topic])
        for topic in [
            *SENSOR_TOPICS,
            "/ocean_current",
            "/ground_truth/odometry",
        ]
        if topic in types
    }
    timestamps = {topic: [] for topic in selected}
    current = []
    ground_truth = []
    navigation = []
    sensor_values = []
    while reader.has_next():
        topic, raw, bag_time = reader.read_next()
        if topic not in selected:
            continue
        msg = deserialize_message(raw, selected[topic])
        t = bag_time * 1e-9
        timestamps[topic].append(t)
        if topic == "/ocean_current":
            current.append([t, msg.x, msg.y, msg.z])
        elif topic == "/ground_truth/odometry":
            p = msg.pose.pose.position
            ground_truth.append([t, p.x, p.y, p.z])
        elif topic == "/navigation/status":
            navigation.append([
                t, msg.navigation_valid, msg.imu_ok, msg.dvl_ok, msg.pressure_ok,
            ])
        elif topic == "/imu/data":
            sensor_values.append([t, "imu_accel_x_mps2", msg.linear_acceleration.x])
            sensor_values.append([t, "imu_accel_y_mps2", msg.linear_acceleration.y])
            sensor_values.append([t, "imu_accel_z_mps2", msg.linear_acceleration.z])
            sensor_values.append([t, "imu_gyro_x_radps", msg.angular_velocity.x])
            sensor_values.append([t, "imu_gyro_y_radps", msg.angular_velocity.y])
            sensor_values.append([t, "imu_gyro_z_radps", msg.angular_velocity.z])
        elif topic == "/dvl/raw":
            speed = math.sqrt(
                msg.velocity.x * msg.velocity.x
                + msg.velocity.y * msg.velocity.y
                + msg.velocity.z * msg.velocity.z
            )
            sensor_values.append([t, "dvl_speed_mps", speed])
            sensor_values.append([t, "dvl_velocity_x_mps", msg.velocity.x])
            sensor_values.append([t, "dvl_velocity_y_mps", msg.velocity.y])
            sensor_values.append([t, "dvl_velocity_z_mps", msg.velocity.z])
            sensor_values.append([t, "dvl_altitude_m", msg.altitude])
            sensor_values.append([t, "dvl_good_beams", msg.num_good_beams])
        elif topic == "/dvl/quality_twist":
            velocity = msg.twist.twist.linear
            speed = math.sqrt(
                velocity.x * velocity.x
                + velocity.y * velocity.y
                + velocity.z * velocity.z
            )
            sensor_values.append([t, "dvl_quality_speed_mps", speed])
        elif topic == "/pressure/raw":
            sensor_values.append([t, "pressure_pa", msg.fluid_pressure])
        elif topic == "/pressure/depth_pose":
            sensor_values.append([t, "pressure_depth_m", -msg.pose.pose.position.z])
        elif topic == "/battery/state":
            sensor_values.append([t, "battery_voltage_v", msg.voltage])
            sensor_values.append([t, "battery_current_a", msg.current])
            sensor_values.append([t, "battery_percentage", msg.percentage])
    return (
        timestamps,
        pd.DataFrame(current, columns=["t", "x", "y", "z"]),
        pd.DataFrame(ground_truth, columns=["t", "x", "y", "z"]),
        pd.DataFrame(
            navigation,
            columns=["t", "navigation_valid", "imu_ok", "dvl_ok", "pressure_ok"],
        ),
        pd.DataFrame(sensor_values, columns=["t", "signal", "value"]),
    )


def topic_rate(values):
    if len(values) < 2:
        return 0.0
    duration = max(values[-1] - values[0], 1e-9)
    return (len(values) - 1) / duration


def write_markdown(frame, path):
    columns = list(frame.columns)
    lines = [
        "| " + " | ".join(columns) + " |",
        "|" + "|".join(["---"] * len(columns)) + "|",
    ]
    for row in frame.itertuples(index=False, name=None):
        values = [
            f"{value:.5f}" if isinstance(value, float) else str(value)
            for value in row
        ]
        lines.append("| " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def configure_plot():
    plt.rcParams.update({
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linestyle": "--",
        "font.family": "DejaVu Sans",
        "savefig.dpi": 240,
        "savefig.bbox": "tight",
    })


def sensor_health(timestamps, navigation, sensor_values, output):
    thresholds = {
        "/imu/data": 20.0,
        "/dvl/raw": 5.0,
        "/dvl/quality_twist": 5.0,
        "/pressure/raw": 5.0,
        "/pressure/depth_pose": 5.0,
        "/battery/state": 0.5,
        "/navigation/status": 10.0,
    }
    rows = []
    for topic, threshold in thresholds.items():
        rate = topic_rate(timestamps.get(topic, []))
        rows.append({
            "Topic": topic,
            "Mesaj sayısı": len(timestamps.get(topic, [])),
            "Ortalama hız (Hz)": rate,
            "Alt sınır (Hz)": threshold,
            "Sonuç": "KABUL" if rate >= threshold else "BAŞARISIZ",
        })
    table = pd.DataFrame(rows)
    health_ratio = {
        name: float(navigation[name].astype(bool).mean())
        if not navigation.empty else 0.0
        for name in ["navigation_valid", "imu_ok", "dvl_ok", "pressure_ok"]
    }
    accepted = (table["Sonuç"] == "KABUL").all() and min(health_ratio.values()) >= 0.95
    summary = pd.DataFrame([{
        "Doğrulama kararı": (
            "KABUL - sensör veri sürekliliği sağlandı"
            if accepted else "BAŞARISIZ - sensör veri sürekliliği sağlanmadı"
        ),
        **{f"{key} oranı": value for key, value in health_ratio.items()},
    }])
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)
    table.to_csv(metrics / "sensor_topic_rates.csv", index=False)
    summary.to_csv(metrics / "sensor_health_summary.csv", index=False)
    write_markdown(table, metrics / "sensor_topic_rates.md")
    write_markdown(summary, metrics / "sensor_health_summary.md")
    if not sensor_values.empty:
        values = sensor_values.copy()
        values["t"] -= values["t"].min()
        value_summary = values.groupby("signal")["value"].agg(
            ["count", "min", "mean", "max", "std"]
        ).reset_index()
        value_summary.rename(
            columns={
                "signal": "Ölçüm",
                "count": "Örnek sayısı",
                "min": "Minimum",
                "mean": "Ortalama",
                "max": "Maksimum",
                "std": "Standart sapma",
            },
            inplace=True,
        )
        values.to_csv(metrics / "sensor_values_timeseries.csv", index=False)
        value_summary.to_csv(metrics / "sensor_value_summary.csv", index=False)
        write_markdown(value_summary, metrics / "sensor_value_summary.md")

    configure_plot()
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    ax.bar(table["Topic"], table["Ortalama hız (Hz)"], color=BLUE)
    ax.scatter(table["Topic"], table["Alt sınır (Hz)"], color=MUTED_RED, label="Alt sınır")
    ax.set_ylabel("Hz")
    ax.tick_params(axis="x", rotation=30)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "sensor_topic_rates.png")
    plt.close(fig)
    if not sensor_values.empty:
        values = sensor_values.copy()
        values["t"] -= values["t"].min()

        def plot_sensor_group(filename, title, signals, ylabel):
            fig, ax = plt.subplots(figsize=(10.5, 5.2))
            colors = [DARK, CORAL, BLUE, MUTED_RED]
            has_data = False
            for index, signal in enumerate(signals):
                series = values[values["signal"] == signal]
                if series.empty:
                    continue
                has_data = True
                ax.plot(
                    series["t"],
                    series["value"],
                    label=signal,
                    color=colors[index % len(colors)],
                    linewidth=1.4,
                )
            if not has_data:
                plt.close(fig)
                return
            ax.set_title(title)
            ax.set_xlabel("Zaman (s)")
            ax.set_ylabel(ylabel)
            ax.legend(loc="best")
            fig.tight_layout()
            fig.savefig(figures / filename)
            plt.close(fig)

        # Sensör verileri tek ve sıkışık bir grafik yerine ayrı dosyalara yazılır.
        # Batarya ölçümleri özellikle grafik dışı bırakıldı.
        plot_sensor_group(
            "sensor_imu_acceleration.png",
            "IMU ivme ölçümleri",
            ["imu_accel_x_mps2", "imu_accel_y_mps2", "imu_accel_z_mps2"],
            "İvme (m/s²)",
        )
        plot_sensor_group(
            "sensor_imu_gyro.png",
            "IMU açısal hız ölçümleri",
            ["imu_gyro_x_radps", "imu_gyro_y_radps", "imu_gyro_z_radps"],
            "Açısal hız (rad/s)",
        )
        plot_sensor_group(
            "sensor_dvl_velocity.png",
            "DVL hız ölçümleri",
            [
                "dvl_speed_mps",
                "dvl_quality_speed_mps",
                "dvl_velocity_x_mps",
                "dvl_velocity_y_mps",
                "dvl_velocity_z_mps",
            ],
            "Hız (m/s)",
        )
        plot_sensor_group(
            "sensor_dvl_quality.png",
            "DVL kalite / irtifa ölçümleri",
            ["dvl_altitude_m", "dvl_good_beams"],
            "İrtifa (m) / iyi beam sayısı",
        )
        plot_sensor_group(
            "sensor_pressure_depth.png",
            "Basınç ve derinlik ölçümleri",
            ["pressure_depth_m", "pressure_pa"],
            "Derinlik (m) / basınç (Pa)",
        )
    print(summary.to_string(index=False))


def current_response(current, ground_truth, target, output):
    if current.empty or ground_truth.empty:
        raise RuntimeError("Akıntı doğrulaması için current ve ground-truth gerekli.")
    current = current.copy()
    ground_truth = ground_truth.copy()
    current["t"] -= current["t"].iloc[0]
    ground_truth["t"] -= ground_truth["t"].iloc[0]
    steady = current[current["t"] >= 0.5 * current["t"].max()]
    errors = {
        axis: float((steady[axis] - value).abs().mean())
        for axis, value in zip(["x", "y", "z"], target)
    }
    displacement = {
        axis: float(ground_truth[axis].iloc[-1] - ground_truth[axis].iloc[0])
        for axis in ["x", "y", "z"]
    }
    accepted = max(errors.values()) <= 0.03
    summary = pd.DataFrame([{
        "Doğrulama kararı": (
            "KABUL - akıntı hedefi yayınlandı"
            if accepted else "BAŞARISIZ - akıntı hedef hatası yüksek"
        ),
        "X ortalama hata (m/s)": errors["x"],
        "Y ortalama hata (m/s)": errors["y"],
        "Z ortalama hata (m/s)": errors["z"],
        "Araç X yer değiştirme (m)": displacement["x"],
        "Araç Y yer değiştirme (m)": displacement["y"],
        "Araç Z yer değiştirme (m)": displacement["z"],
        "Hedef akıntı büyüklüğü (m/s)": math.sqrt(sum(v * v for v in target)),
    }])
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)
    current.to_csv(metrics / "ocean_current_timeseries.csv", index=False)
    summary.to_csv(metrics / "ocean_current_summary.csv", index=False)
    write_markdown(summary, metrics / "ocean_current_summary.md")

    configure_plot()
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    for axis, value, color in zip(["x", "y", "z"], target, [DARK, CORAL, BLUE]):
        ax.plot(current["t"], current[axis], color=color, label=f"{axis.upper()} ölçüm")
        ax.axhline(value, color=color, linestyle="--", alpha=0.6)
    ax.set_xlabel("Zaman (s)")
    ax.set_ylabel("Akıntı (m/s)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "ocean_current_tracking.png")
    plt.close(fig)
    print(summary.to_string(index=False))

def current_services_activity(current, output):
    if current.empty:
        raise RuntimeError("Akıntı servis doğrulaması için /ocean_current gerekli.")
    current = current.copy()
    current["t"] -= current["t"].iloc[0]
    current["magnitude"] = (
        current["x"] ** 2 + current["y"] ** 2 + current["z"] ** 2
    ) ** 0.5
    summary = pd.DataFrame([{
        "Akıntı örnek sayısı": len(current),
        "Ortalama X (m/s)": float(current["x"].mean()),
        "Ortalama Y (m/s)": float(current["y"].mean()),
        "Ortalama Z (m/s)": float(current["z"].mean()),
        "Maksimum büyüklük (m/s)": float(current["magnitude"].max()),
    }])
    metrics = output / "metrics"
    metrics.mkdir(parents=True, exist_ok=True)
    current.to_csv(metrics / "ocean_current_service_timeseries.csv", index=False)
    summary.to_csv(metrics / "ocean_current_service_activity.csv", index=False)
    write_markdown(summary, metrics / "ocean_current_service_activity.md")

    configure_plot()
    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.0, 4.8))
    for axis, color in zip(["x", "y", "z"], [DARK, CORAL, BLUE]):
        ax.plot(current["t"], current[axis], color=color, label=f"{axis.upper()}")
    ax.plot(
        current["t"], current["magnitude"],
        color=MUTED_RED, linestyle="--", label="Büyüklük",
    )
    ax.set_xlabel("Zaman (s)")
    ax.set_ylabel("Akıntı (m/s)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "ocean_current_service_activity.png")
    plt.close(fig)
    print(summary.to_string(index=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--case",
        choices=[
            "sensor_health",
            "ocean_current_response",
            "ocean_current_services",
        ],
        required=True,
    )
    parser.add_argument("--current-x", type=float, default=0.4)
    parser.add_argument("--current-y", type=float, default=0.2)
    parser.add_argument("--current-z", type=float, default=0.0)
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    timestamps, current, ground_truth, navigation, sensor_values = read_bag(bag)
    if args.case == "sensor_health":
        sensor_health(timestamps, navigation, sensor_values, output)
    elif args.case == "ocean_current_response":
        current_response(
            current,
            ground_truth,
            (args.current_x, args.current_y, args.current_z),
            output,
        )
    else:
        current_services_activity(current, output)


if __name__ == "__main__":
    main()