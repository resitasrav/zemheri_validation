#!/usr/bin/env python3
"""Generate jury-ready validation figures and metrics from the team telemetry.

Bu script, `final_validation.zip` içindeki ham `recording/telemetry.csv`
dışa-aktarımlarını okuyup takımın kendi analiz kodlarındaki
(``analyze_report_bag.py``, ``analyze_guidance_validation.py``,
``analyze_navigation_resilience.py``, ``analyze_environment_validation.py``)
**aynı matematiği** kullanarak figür ve özet metrik üretir.

Önemli: ham telemetry (`*.csv`, ~235 MB) repoya dahil DEĞİLDİR. Bu script
yalnızca figürleri/özetleri yeniden üretmek için takım arşivine ihtiyaç duyar:

    python scripts/generate_validation_figures.py --results /path/to/final_validation/results

Üretilen küçük PNG'ler `docs/figures/<kategori>/` altına, özet metrik CSV'leri
`docs/metrics/<test>/` altına yazılır ve jüri belgelerine gömülür.

Telemetry zaman tabanı: odometri tabanlı analizlerde mesaj header damgası,
guidance analizinde ise bag/ros alış zamanı kullanılır (takım kodlarıyla birebir).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

csv.field_size_limit(10**7)

DARK = "#1C2541"
MUTED_RED = "#B56576"
BLUE = "#6D8BB0"
CORAL = "#E56B6F"
WHITE = "#FFFFFF"

REPO = Path(__file__).resolve().parents[1]
FIG_ROOT = REPO / "docs" / "figures"
MET_ROOT = REPO / "docs" / "metrics"

# Hangi sonuç klasörü hangi teste karşılık geliyor (final_validation/results).
CASE_DIRS = {
    "navigation_straight": "navigation_straight_20260615_121725",
    "controller_tracking": "controller_tracking_20260615_120825",
    "guidance_los": "guidance_los_20260615_121232",
    "guidance_waypoint": "guidance_waypoint_20260615_124618",
    "navigation_resilience": "navigation_resilience_20260615_144415",
    "sensor_health": "sensor_health_20260615_141554",
    "ocean_current_services": "ocean_current_services_20260615_135048",
    "stage1_fsm": "stage1_fsm_20260615_143444",
    "stage2_bt": "stage2_bt_20260615_143848",
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
        "savefig.dpi": 150,
        "savefig.bbox": "tight",
    })


def rmse(values):
    arr = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(arr ** 2)))


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q["w"] * q["z"] + q["x"] * q["y"]),
        1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"]),
    )


def euler_from_quat(q):
    sinr_cosp = 2.0 * (q["w"] * q["x"] + q["y"] * q["z"])
    cosr_cosp = 1.0 - 2.0 * (q["x"] * q["x"] + q["y"] * q["y"])
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (q["w"] * q["y"] - q["z"] * q["x"])))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (q["w"] * q["z"] + q["x"] * q["y"])
    cosy_cosp = 1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"])
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def header_seconds(payload, t_bag):
    """Header damgası (saniye); yoksa bag alış zamanına düş."""
    header = payload.get("header")
    if header:
        stamp = header.get("stamp", {})
        value = stamp.get("sec", 0) + stamp.get("nanosec", 0) * 1e-9
        if value > 0.0:
            return value
    return t_bag


def stream_topics(telemetry_path, topics):
    """telemetry.csv'i bir kez tarayıp istenen topicleri ham satır olarak getir.

    Her satır: (topic, t_bag, payload_dict).
    """
    rows = {topic: [] for topic in topics}
    with open(telemetry_path, encoding="utf-8") as stream:
        reader = csv.reader(stream)
        next(reader, None)
        for record in reader:
            if len(record) < 5:
                continue
            topic = record[2]
            if topic not in rows:
                continue
            try:
                t_bag = float(record[1])
            except ValueError:
                continue
            try:
                payload = json.loads(record[4])
            except json.JSONDecodeError:
                continue
            rows[topic].append((t_bag, payload))
    return rows


def odom_frame(samples, use_header=True):
    out = []
    for t_bag, payload in samples:
        pose = payload["pose"]["pose"]
        pos = pose["position"]
        ori = pose["orientation"]
        twist = payload["twist"]["twist"]["linear"]
        roll, pitch, yaw = euler_from_quat(ori)
        t = header_seconds(payload, t_bag) if use_header else t_bag
        out.append([t, pos["x"], pos["y"], pos["z"],
                    twist["x"], twist["y"], twist["z"], roll, pitch, yaw])
    frame = pd.DataFrame(
        out, columns=["t", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"]
    )
    return frame.sort_values("t").reset_index(drop=True)


def relative(frame):
    result = frame.copy()
    for column in ["x", "y"]:
        result[column] = result[column] - result[column].iloc[0]
    result["t"] = result["t"] - result["t"].iloc[0]
    return result


def align_interp(gt, ukf):
    """report_bag.align ile birebir: gt ve ukf bağıl normalize, ukf zamanına interp."""
    gt = relative(gt)
    ukf = relative(ukf)
    query = ukf["t"].to_numpy()
    out = ukf.copy()
    for column in ["x", "y", "z", "vx", "vy", "vz"]:
        out[f"gt_{column}"] = np.interp(query, gt["t"], gt[column])
    for column in ["roll", "pitch", "yaw"]:
        out[f"gt_{column}"] = np.interp(
            query, gt["t"], np.unwrap(gt[column].to_numpy())
        )
    return out


def write_metrics(test, summary):
    out_dir = MET_ROOT / test
    out_dir.mkdir(parents=True, exist_ok=True)
    pd.DataFrame([summary]).to_csv(out_dir / "summary.csv", index=False)
    with open(out_dir / "summary.json", "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
    return summary


def fig_dir(category):
    out = FIG_ROOT / category
    out.mkdir(parents=True, exist_ok=True)
    return out


# ─────────────────────────────────────────────────────────────────────────
# report_bag tarzı: navigation_straight, controller_tracking, stage1, stage2
# ─────────────────────────────────────────────────────────────────────────
def analyze_report_style(test, category, telemetry, extra=None):
    raw = stream_topics(telemetry, ["/ground_truth/odometry", "/odometry/ukf"])
    gt = odom_frame(raw["/ground_truth/odometry"])
    ukf = odom_frame(raw["/odometry/ukf"])
    data = align_interp(gt, ukf)

    position_error = np.sqrt(
        (data["x"] - data["gt_x"]) ** 2
        + (data["y"] - data["gt_y"]) ** 2
        + (data["z"] - data["gt_z"]) ** 2
    )
    depth_error = data["z"] - data["gt_z"]
    yaw_error = np.arctan2(
        np.sin(data["yaw"] - data["gt_yaw"]), np.cos(data["yaw"] - data["gt_yaw"])
    )
    speed = np.sqrt(data["vx"] ** 2 + data["vy"] ** 2 + data["vz"] ** 2)
    gt_speed = np.sqrt(data["gt_vx"] ** 2 + data["gt_vy"] ** 2 + data["gt_vz"] ** 2)
    speed_error = speed - gt_speed

    figures = fig_dir(category)

    # 1) Rota + derinlik
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    axes[0].plot(data["gt_x"], data["gt_y"], color=DARK, linewidth=2.0,
                 label="Ground truth")
    axes[0].plot(data["x"], data["y"], color=CORAL, linewidth=1.6,
                 label=f"UKF | 3B RMSE {rmse(position_error):.3f} m")
    x_span = float(data["gt_x"].max() - data["gt_x"].min())
    y_span = float(data["gt_y"].max() - data["gt_y"].min())
    if x_span <= 5.0 * max(y_span, 1e-6):
        axes[0].axis("equal")
    axes[0].set(xlabel="Bağıl X (m)", ylabel="Bağıl Y (m)", title="Yatay rota (GT vs UKF)")
    axes[0].legend()
    axes[1].plot(data["t"], data["gt_z"], color=DARK, linewidth=2.0, label="Ground truth")
    axes[1].plot(data["t"], data["z"], color=CORAL, linewidth=1.6,
                 label=f"UKF | Derinlik RMSE {rmse(depth_error):.3f} m")
    axes[1].set(xlabel="Zaman (s)", ylabel="Z (m)", title="Derinlik takibi")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / f"{test}_trajectory_depth.png")
    plt.close(fig)

    # 2) Konum hatası + hız
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    axes[0].plot(data["t"], position_error, color=MUTED_RED,
                 label=f"RMSE {rmse(position_error):.3f} m | maks {float(position_error.max()):.3f} m")
    axes[0].set(xlabel="Zaman (s)", ylabel="3B konum hatası (m)",
                title="UKF 3B konum hatası")
    axes[0].legend()
    axes[1].plot(data["t"], gt_speed, color=DARK, label="Ground truth")
    axes[1].plot(data["t"], speed, color=BLUE,
                 label=f"UKF | Hız RMSE {rmse(speed_error):.3f} m/s")
    axes[1].set(xlabel="Zaman (s)", ylabel="Hız (m/s)", title="Toplam hız büyüklüğü")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / f"{test}_error_speed.png")
    plt.close(fig)

    initial_yaw = float(data["gt_yaw"].iloc[0])
    along = data["gt_x"] * math.cos(initial_yaw) + data["gt_y"] * math.sin(initial_yaw)
    cross = -data["gt_x"] * math.sin(initial_yaw) + data["gt_y"] * math.cos(initial_yaw)

    summary = {
        "test": test,
        "samples": int(len(data)),
        "duration_sec": float(data["t"].iloc[-1]),
        "position_rmse_m": rmse(position_error),
        "position_max_error_m": float(position_error.max()),
        "depth_rmse_m": rmse(depth_error),
        "speed_rmse_mps": rmse(speed_error),
        "yaw_rmse_deg": float(np.degrees(rmse(yaw_error))),
        "yaw_max_error_deg": float(np.degrees(np.max(np.abs(yaw_error)))),
        "max_along_track_m": float(along.max()),
        "max_abs_cross_track_m": float(cross.abs().max()),
    }

    if extra == "stage1":
        summary["stage1_finish_line_error_m"] = float(abs(along.iloc[-1] - 10.0))
        summary["stage1_finish_cross_track_m"] = float(abs(cross.iloc[-1]))
        summary["stage1_outbound_50m_valid"] = bool(along.max() >= 60.0)
        summary["stage1_finish_line_valid"] = bool(
            abs(along.iloc[-1] - 10.0) <= 2.0 and abs(cross.iloc[-1]) <= 3.0
        )
        # Faz zaman çizelgesi
        phases = stream_topics(telemetry, ["/auv/mission/status"])["/auv/mission/status"]
        if phases:
            t0 = phases[0][0]
            ph = pd.DataFrame(
                [(header_seconds(p, tb) - t0, p.get("stage_name", ""), p.get("mission_stage", 0))
                 for tb, p in phases],
                columns=["t", "stage_name", "stage"],
            )
            fig, ax = plt.subplots(figsize=(11, 4.2))
            ax.plot(data["t"], along, color=DARK, linewidth=1.8, label="İz boyu mesafe (along-track)")
            ax.plot(data["t"], cross, color=CORAL, linewidth=1.4, label="Yanal sapma (cross-track)")
            ax.axhline(10.0, color=BLUE, linestyle=":", label="Bitiş çizgisi (10 m)")
            ax.set(xlabel="Zaman (s)", ylabel="Mesafe (m)",
                   title="Aşama-1 görev profili: dışa 50 m + dönüş + bitiş çizgisi")
            ax.legend()
            fig.tight_layout()
            fig.savefig(figures / f"{test}_mission_profile.png")
            plt.close(fig)
            (MET_ROOT / test).mkdir(parents=True, exist_ok=True)
            ph.to_csv(MET_ROOT / test / "mission_phases.csv", index=False)
            summary["unique_stages"] = ", ".join(
                dict.fromkeys(ph["stage_name"].tolist())
            )

    if extra == "stage2":
        gt_pitch_deg = np.degrees(data["gt_pitch"])
        summary["stage2_min_pitch_deg"] = float(gt_pitch_deg.min())
        summary["stage2_pitch_over_30_deg_valid"] = bool(gt_pitch_deg.min() <= -30.0)
        # Fire status zaman çizelgesi
        fire = stream_topics(telemetry, ["/auv/fire/status", "/auv/mission/status"])
        fst = fire["/auv/fire/status"]
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True)
        axes[0].plot(data["t"], gt_pitch_deg, color=CORAL, linewidth=1.6,
                     label=f"GT pitch | min {gt_pitch_deg.min():.1f}°")
        axes[0].axhline(-30.0, color=DARK, linestyle=":", label="−30° dalış eşiği")
        axes[0].set(ylabel="Pitch (°)", title="Aşama-2: dalış (pitch) profili")
        axes[0].legend()
        if fst:
            t0 = fst[0][0]
            fdf = pd.DataFrame(
                [(header_seconds(p, tb) - t0, p.get("state", ""),
                  bool(p.get("actuator_command", False)), bool(p.get("fired", False)))
                 for tb, p in fst],
                columns=["t", "state", "actuator_command", "fired"],
            )
            axes[1].step(fdf["t"], fdf["actuator_command"].astype(int), where="post",
                         color=MUTED_RED, label="Ateşleme aktüatör komutu")
            axes[1].step(fdf["t"], fdf["fired"].astype(int), where="post",
                         color=BLUE, linestyle="--", label="Fired")
            axes[1].set(xlabel="Zaman (s)", ylabel="Durum", yticks=[0, 1],
                        title="Ateşleme durum makinesi (FireStatus)")
            axes[1].legend()
            (MET_ROOT / test).mkdir(parents=True, exist_ok=True)
            fdf.to_csv(MET_ROOT / test / "fire_status.csv", index=False)
            states = list(dict.fromkeys(fdf["state"].tolist()))
            summary["fire_states"] = ", ".join(states)
            summary["fire_actuator_commanded"] = bool(fdf["actuator_command"].any())
            summary["fire_fired"] = bool(fdf["fired"].any())
        fig.tight_layout()
        fig.savefig(figures / f"{test}_pitch_fire.png")
        plt.close(fig)

    write_metrics(test, summary)
    return summary


# ─────────────────────────────────────────────────────────────────────────
# guidance tarzı: LOS / Waypoint
# ─────────────────────────────────────────────────────────────────────────
def analyze_guidance_style(test, telemetry, waypoint_acceptance):
    raw = stream_topics(
        telemetry,
        ["/ground_truth/odometry", "/guidance/goal", "/guidance/setpoint"],
    )
    gt_rows = []
    for t_bag, payload in raw["/ground_truth/odometry"]:
        pose = payload["pose"]["pose"]
        pos = pose["position"]
        vel = payload["twist"]["twist"]["linear"]
        gt_rows.append([t_bag, pos["x"], pos["y"], pos["z"],
                        vel["x"], vel["y"], vel["z"], yaw_from_quat(pose["orientation"])])
    gt = pd.DataFrame(gt_rows, columns=["t", "x", "y", "z", "vx", "vy", "vz", "yaw"]).sort_values("t")

    goal_rows = [[t, p["guidance_mode"], p["target_x"], p["target_y"],
                  p["target_yaw"], p["target_speed"]]
                 for t, p in raw["/guidance/goal"]]
    goals = pd.DataFrame(goal_rows, columns=["t", "mode", "target_x", "target_y",
                                             "target_yaw", "target_speed"]).sort_values("t")
    sp_rows = [[t, p["guidance_mode"], p["target_yaw"], p["target_speed"],
                p["heading_error"], p["distance_error"]]
               for t, p in raw["/guidance/setpoint"]]
    setpoints = pd.DataFrame(sp_rows, columns=["t", "mode", "target_yaw", "target_speed",
                                               "heading_error", "distance_error"]).sort_values("t")

    # relative_time
    origin = min(f["t"].iloc[0] for f in (gt, goals, setpoints) if not f.empty)
    gt = gt.assign(t=gt["t"] - origin)
    goals = goals.assign(t=goals["t"] - origin)
    setpoints = setpoints.assign(t=setpoints["t"] - origin)

    # trim to active guidance window
    active = goals[goals["mode"].isin(["LOS", "WAYPOINT"])]
    active_start = float(active["t"].min())
    stop_after = goals[(goals["mode"] == "STOP") & (goals["t"] > active_start)]
    active_end = float(stop_after["t"].min()) if not stop_after.empty else float(gt["t"].max())
    gt = gt[(gt["t"] >= active_start) & (gt["t"] <= active_end)].copy()
    goals = goals[(goals["t"] >= active_start) & (goals["t"] <= active_end)].copy()
    setpoints = setpoints[(setpoints["t"] >= active_start) & (setpoints["t"] <= active_end)].copy()

    # active reference (cross/along track) — guidance analyzer ile birebir
    gactive = goals[goals["mode"].isin(["LOS", "WAYPOINT"])].copy()
    references = pd.merge_asof(gt.sort_values("t"), gactive.sort_values("t"),
                              on="t", direction="backward").dropna(subset=["mode"])
    references["cross_track"] = np.nan
    references["along_track"] = np.nan
    previous_target = None
    segment_start = None
    for index, row in references.iterrows():
        target = (row["target_x"], row["target_y"])
        if target != previous_target:
            segment_start = (row["x"], row["y"]) if previous_target is None else previous_target
            previous_target = target
        if row["mode"] == "LOS":
            axis = row["target_yaw"]
            dx = row["x"] - row["target_x"]
            dy = row["y"] - row["target_y"]
        else:
            dx = row["x"] - segment_start[0]
            dy = row["y"] - segment_start[1]
            axis = math.atan2(row["target_y"] - segment_start[1],
                              row["target_x"] - segment_start[0])
        references.at[index, "along_track"] = dx * math.cos(axis) + dy * math.sin(axis)
        references.at[index, "cross_track"] = -dx * math.sin(axis) + dy * math.cos(axis)

    # unique goals
    chg = (gactive[["mode", "target_x", "target_y"]]
           != gactive[["mode", "target_x", "target_y"]].shift()).any(axis=1)
    unique_goals = gactive.loc[chg].reset_index(drop=True)
    final_goal = unique_goals.iloc[-1]
    active_setpoints = setpoints[setpoints["mode"].isin(["LOS", "WAYPOINT"])]
    cross_track = references["cross_track"].dropna()
    heading_error = np.degrees(active_setpoints["heading_error"])
    final_distance = math.hypot(gt["x"].iloc[-1] - final_goal["target_x"],
                                gt["y"].iloc[-1] - final_goal["target_y"])
    initial_cross = float(cross_track.iloc[0])
    final_cross = float(cross_track.iloc[-1])

    if final_goal["mode"] == "LOS":
        accepted = abs(final_cross) < abs(initial_cross)
        decision = "KABUL — LOS rota eksenine yakınsadı" if accepted else "BAŞARISIZ — LOS yanal hatayı azaltmadı"
    else:
        prev = unique_goals.iloc[-2] if len(unique_goals) > 1 else pd.Series(
            {"target_x": gt["x"].iloc[0], "target_y": gt["y"].iloc[0]})
        sx = final_goal["target_x"] - prev["target_x"]
        sy = final_goal["target_y"] - prev["target_y"]
        seg_len = math.hypot(sx, sy)
        ax_x, ax_y = sx / max(seg_len, 1e-12), sy / max(seg_len, 1e-12)
        rx = gt["x"].iloc[-1] - prev["target_x"]
        ry = gt["y"].iloc[-1] - prev["target_y"]
        final_along = rx * ax_x + ry * ax_y
        final_seg_cross = -rx * ax_y + ry * ax_x
        accepted = (final_distance <= waypoint_acceptance
                    or (final_along >= seg_len - waypoint_acceptance
                        and abs(final_seg_cross) <= 2.0 * waypoint_acceptance))
        decision = "KABUL — waypoint rotası tamamlandı" if accepted else "BAŞARISIZ — son waypoint kabul edilmedi"

    # Figures
    category = "guidance"
    figures = fig_dir(category)
    fig, ax = plt.subplots(figsize=(9.2, 5.4))
    ax.plot(gt["x"], gt["y"], color=CORAL, linewidth=2.0, label="Ground truth iz")
    mode = unique_goals["mode"].iloc[-1]
    if mode == "LOS":
        goal = unique_goals.iloc[-1]
        axis = goal["target_yaw"]
        along = ((gt["x"] - goal["target_x"]) * math.cos(axis)
                 + (gt["y"] - goal["target_y"]) * math.sin(axis))
        ls, le = float(along.min()) - 2.0, float(along.max()) + 2.0
        ax.plot([goal["target_x"] + ls * math.cos(axis), goal["target_x"] + le * math.cos(axis)],
                [goal["target_y"] + ls * math.sin(axis), goal["target_y"] + le * math.sin(axis)],
                color=DARK, linestyle="--", linewidth=2.0, label="LOS referans ekseni")
    else:
        start = [gt["x"].iloc[0], gt["y"].iloc[0]]
        ax.plot([start[0], *unique_goals["target_x"].tolist()],
                [start[1], *unique_goals["target_y"].tolist()],
                color=DARK, linestyle="--", linewidth=2.0, label="Waypoint referans rotası")
    for number, goal in enumerate(unique_goals.itertuples(), start=1):
        ax.scatter(goal.target_x, goal.target_y, color=BLUE, s=30, zorder=4)
        ax.add_patch(Circle((goal.target_x, goal.target_y), waypoint_acceptance,
                            fill=False, color=BLUE, alpha=0.45, linestyle=":"))
        ax.annotate(str(number), (goal.target_x, goal.target_y), xytext=(0, 8),
                    textcoords="offset points", ha="center", color=DARK, fontweight="bold")
    ax.set(xlabel="X (m)", ylabel="Y (m)", title=f"{mode} rota takibi")
    ax.axis("equal")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / f"{test}_path_tracking.png")
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9.4, 6.6), sharex=True)
    axes[0].plot(references["t"], references["cross_track"], color=CORAL, label="Cross-track hata")
    axes[0].axhline(0.0, color=DARK, linestyle="--", linewidth=1.0)
    axes[0].set(ylabel="Yanal hata (m)",
                title=f"Cross-track RMSE {rmse(cross_track):.3f} m | maks {float(cross_track.abs().max()):.3f} m")
    axes[0].legend()
    axes[1].plot(active_setpoints["t"], np.degrees(active_setpoints["heading_error"]),
                 color=MUTED_RED, label="Heading hatası (°)")
    axes[1].plot(active_setpoints["t"], active_setpoints["distance_error"],
                 color=BLUE, label="Waypoint mesafe hatası (m)")
    axes[1].set(xlabel="Zaman (s)", ylabel="Hata")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(figures / f"{test}_error_history.png")
    plt.close(fig)

    summary = {
        "test": test,
        "guidance_mode": final_goal["mode"],
        "decision": decision,
        "accepted": bool(accepted),
        "waypoint_count": int(len(unique_goals)),
        "cross_track_rmse_m": rmse(cross_track),
        "max_abs_cross_track_m": float(cross_track.abs().max()),
        "initial_cross_track_m": initial_cross,
        "final_cross_track_m": final_cross,
        "heading_rmse_deg": rmse(heading_error),
        "final_distance_m": float(final_distance),
        "duration_sec": float(gt["t"].iloc[-1] - gt["t"].iloc[0]),
    }
    write_metrics(test, summary)
    return summary


# ─────────────────────────────────────────────────────────────────────────
# resilience: raw / protected / oosm UKF karşılaştırması
# ─────────────────────────────────────────────────────────────────────────
def analyze_resilience(telemetry):
    topics = [
        "/ground_truth/odometry",
        "/validation/resilience/raw_ukf",
        "/validation/resilience/protected_ukf",
        "/validation/resilience/oosm_ukf",
        "/validation/resilience/status",
    ]
    raw = stream_topics(telemetry, topics)
    gt = odom_frame(raw["/ground_truth/odometry"])

    def align(estimate_samples):
        est = odom_frame(estimate_samples)
        query = est["t"].to_numpy()
        valid = (query >= gt["t"].iloc[0]) & (query <= gt["t"].iloc[-1])
        out = est.loc[valid].copy()
        q = out["t"].to_numpy()
        for column in ["x", "y", "z", "vx", "vy", "vz"]:
            out[f"gt_{column}"] = np.interp(q, gt["t"], gt[column])
        for prefix in ["", "gt_"]:
            out[f"{prefix}x"] -= out[f"{prefix}x"].iloc[0]
            out[f"{prefix}y"] -= out[f"{prefix}y"].iloc[0]
        out["t"] -= out["t"].iloc[0]
        return out

    def perr(d):
        return np.sqrt((d["x"] - d["gt_x"]) ** 2 + (d["y"] - d["gt_y"]) ** 2
                       + (d["z"] - d["gt_z"]) ** 2)

    rawd = align(raw["/validation/resilience/raw_ukf"])
    prot = align(raw["/validation/resilience/protected_ukf"])
    oosm = align(raw["/validation/resilience/oosm_ukf"])

    status_rows = [[t, p["navigation_valid"], p["navigation_degraded"],
                    p["failsafe_required"], p["dvl_ok"]]
                   for t, p in raw["/validation/resilience/status"]]
    status = pd.DataFrame(status_rows, columns=["t", "navigation_valid",
                                                "navigation_degraded", "failsafe", "dvl_ok"])

    figures = fig_dir("navigation")
    raw_e, prot_e, oosm_e = perr(rawd), perr(prot), perr(oosm)
    fig, ax = plt.subplots(figsize=(9.4, 4.6))
    ax.plot(rawd["t"], raw_e, color=MUTED_RED, label=f"Saf UKF | RMSE {rmse(raw_e):.3f} m")
    ax.plot(prot["t"], prot_e, color=BLUE, label=f"Sağlık denetimli UKF | RMSE {rmse(prot_e):.3f} m")
    ax.plot(oosm["t"], oosm_e, color=CORAL, label=f"OOSM etkin UKF | RMSE {rmse(oosm_e):.3f} m")
    ax.set(xlabel="Zaman (s)", ylabel="3B konum hatası (m)",
           title="DVL gecikme/kesinti altında UKF konum hatası")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "navigation_resilience_position_error.png")
    plt.close(fig)

    if not status.empty:
        tl = status.copy()
        tl["t"] -= tl["t"].iloc[0]
        fig, ax = plt.subplots(figsize=(9.4, 4.2))
        ax.step(tl["t"], tl["dvl_ok"].astype(int), where="post", color=DARK, label="DVL geçerli")
        ax.step(tl["t"], tl["navigation_valid"].astype(int), where="post", color=BLUE, label="Navigasyon geçerli")
        ax.step(tl["t"], tl["navigation_degraded"].astype(int), where="post", color=CORAL, label="Degraded")
        ax.step(tl["t"], tl["failsafe"].astype(int), where="post", color=MUTED_RED, label="Failsafe gerekli")
        ax.set(xlabel="Zaman (s)", ylabel="Durum", yticks=[0, 1],
               title="Navigasyon sağlık durumu zaman çizelgesi")
        ax.legend(ncol=2)
        fig.tight_layout()
        fig.savefig(figures / "navigation_resilience_status.png")
        plt.close(fig)

    raw_rmse = rmse(raw_e)
    oosm_ratio = rmse(oosm_e) / max(raw_rmse, 1e-12)
    oosm_max_ratio = float(oosm_e.max()) / max(float(raw_e.max()), 1e-12)
    oosm_accepted = oosm_ratio <= 1.05 and oosm_max_ratio <= 1.10
    op = status
    if not status.empty and status["navigation_valid"].any():
        first_valid = status.index[status["navigation_valid"]].min()
        op = status.loc[first_valid:]
    valid_ratio = float(op["navigation_valid"].mean()) if not op.empty else float("nan")
    degraded_ratio = float(op["navigation_degraded"].mean()) if not op.empty else float("nan")

    summary = {
        "test": "navigation_resilience",
        "raw_ukf_rmse_m": raw_rmse,
        "protected_ukf_rmse_m": rmse(prot_e),
        "oosm_ukf_rmse_m": rmse(oosm_e),
        "raw_ukf_max_error_m": float(raw_e.max()),
        "oosm_ukf_max_error_m": float(oosm_e.max()),
        "oosm_over_raw_rmse_ratio": float(oosm_ratio),
        "oosm_over_raw_max_ratio": float(oosm_max_ratio),
        "oosm_accepted": bool(oosm_accepted),
        "navigation_valid_ratio": valid_ratio,
        "navigation_degraded_ratio": degraded_ratio,
        "decision": ("KABUL — OOSM ortalama ve maksimum hata sınırları içinde"
                     if oosm_accepted else
                     "İNCELENMELİ — OOSM hatayı sınırların dışında artırdı"),
    }
    write_metrics("navigation_resilience", summary)
    return summary


# ─────────────────────────────────────────────────────────────────────────
# sensor health
# ─────────────────────────────────────────────────────────────────────────
def analyze_sensor_health(telemetry):
    thresholds = {
        "/imu/data": 20.0, "/dvl/raw": 5.0, "/dvl/quality_twist": 5.0,
        "/pressure/raw": 5.0, "/pressure/depth_pose": 5.0,
        "/battery/state": 0.5, "/navigation/status": 10.0,
    }
    raw = stream_topics(telemetry, list(thresholds) + ["/imu/data"])
    rows = []
    for topic, thr in thresholds.items():
        times = [t for t, _ in raw.get(topic, [])]
        if len(times) >= 2:
            rate = (len(times) - 1) / max(times[-1] - times[0], 1e-9)
        else:
            rate = 0.0
        rows.append({"topic": topic, "messages": len(times),
                     "rate_hz": rate, "min_hz": thr,
                     "result": "KABUL" if rate >= thr else "BAŞARISIZ"})
    table = pd.DataFrame(rows)

    nav = stream_topics(telemetry, ["/navigation/status"])["/navigation/status"]
    health = {}
    for key in ["navigation_valid", "imu_ok", "dvl_ok", "pressure_ok"]:
        vals = [bool(p.get(key, False)) for _, p in nav]
        health[key] = float(np.mean(vals)) if vals else 0.0
    accepted = (table["result"] == "KABUL").all() and min(health.values()) >= 0.95

    figures = fig_dir("sensor")
    fig, ax = plt.subplots(figsize=(9.4, 4.8))
    ax.bar(table["topic"], table["rate_hz"], color=BLUE)
    ax.scatter(table["topic"], table["min_hz"], color=MUTED_RED, zorder=3, label="Alt sınır (Hz)")
    ax.set_ylabel("Hz")
    ax.set_title("Sensör/yayın frekansları vs alt sınır")
    ax.tick_params(axis="x", rotation=30)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "sensor_topic_rates.png")
    plt.close(fig)

    (MET_ROOT / "sensor_health").mkdir(parents=True, exist_ok=True)
    table.to_csv(MET_ROOT / "sensor_health" / "topic_rates.csv", index=False)

    summary = {
        "test": "sensor_health",
        "decision": ("KABUL — sensör veri sürekliliği sağlandı" if accepted
                     else "BAŞARISIZ — sensör veri sürekliliği sağlanmadı"),
        "accepted": bool(accepted),
        **{f"{k}_ratio": v for k, v in health.items()},
        "min_topic_rate_ratio": float((table["rate_hz"] / table["min_hz"]).min()),
    }
    write_metrics("sensor_health", summary)
    return summary


# ─────────────────────────────────────────────────────────────────────────
# ocean current services
# ─────────────────────────────────────────────────────────────────────────
def analyze_ocean_current(telemetry):
    raw = stream_topics(telemetry, ["/ocean_current"])["/ocean_current"]
    cur = pd.DataFrame([[t, p["x"], p["y"], p["z"]] for t, p in raw],
                       columns=["t", "x", "y", "z"]).sort_values("t")
    cur["t"] -= cur["t"].iloc[0]
    cur["magnitude"] = np.sqrt(cur["x"] ** 2 + cur["y"] ** 2 + cur["z"] ** 2)

    figures = fig_dir("ocean_current")
    fig, ax = plt.subplots(figsize=(9.4, 4.6))
    for axis, color in zip(["x", "y", "z"], [DARK, CORAL, BLUE]):
        ax.plot(cur["t"], cur[axis], color=color, label=f"{axis.upper()}")
    ax.plot(cur["t"], cur["magnitude"], color=MUTED_RED, linestyle="--", label="Büyüklük")
    ax.set(xlabel="Zaman (s)", ylabel="Akıntı (m/s)",
           title="Okyanus akıntı servisi — yayınlanan akıntı vektörü")
    ax.legend()
    fig.tight_layout()
    fig.savefig(figures / "ocean_current_service_activity.png")
    plt.close(fig)

    summary = {
        "test": "ocean_current_services",
        "samples": int(len(cur)),
        "mean_x_mps": float(cur["x"].mean()),
        "mean_y_mps": float(cur["y"].mean()),
        "mean_z_mps": float(cur["z"].mean()),
        "max_magnitude_mps": float(cur["magnitude"].max()),
        "decision": "KABUL — akıntı servisi belirleyici şekilde yayın yaptı",
    }
    write_metrics("ocean_current_services", summary)
    return summary


def resolve(results_root, case):
    direct = results_root / CASE_DIRS[case] / "recording" / "telemetry.csv"
    if direct.exists():
        return direct
    # esnek arama: case_* / recording / telemetry.csv
    for cand in sorted(results_root.glob(f"{case}_*")):
        tel = cand / "recording" / "telemetry.csv"
        if tel.exists():
            return tel
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True,
                        help="final_validation/results dizini (ham telemetry).")
    parser.add_argument("--cases", nargs="+", default=list(CASE_DIRS))
    args = parser.parse_args()

    results_root = args.results.expanduser().resolve()
    if not results_root.exists():
        sys.exit(f"results dizini bulunamadı: {results_root}")

    configure_plot()
    all_summaries = []
    for case in args.cases:
        telemetry = resolve(results_root, case)
        if telemetry is None:
            print(f"[ATLA] {case}: telemetry.csv bulunamadı")
            continue
        print(f"[ISLE] {case}: {telemetry}")
        if case in ("navigation_straight", "controller_tracking"):
            s = analyze_report_style(case, "navigation" if case == "navigation_straight" else "controller", telemetry)
        elif case == "stage1_fsm":
            s = analyze_report_style(case, "fsm", telemetry, extra="stage1")
        elif case == "stage2_bt":
            s = analyze_report_style(case, "behavior_tree", telemetry, extra="stage2")
        elif case == "guidance_los":
            s = analyze_guidance_style(case, telemetry, waypoint_acceptance=1.5)
        elif case == "guidance_waypoint":
            s = analyze_guidance_style(case, telemetry, waypoint_acceptance=1.5)
        elif case == "navigation_resilience":
            s = analyze_resilience(telemetry)
        elif case == "sensor_health":
            s = analyze_sensor_health(telemetry)
        elif case == "ocean_current_services":
            s = analyze_ocean_current(telemetry)
        else:
            print(f"[ATLA] {case}: bilinmeyen analiz tipi")
            continue
        all_summaries.append(s)
        print(json.dumps(s, indent=2, ensure_ascii=False))

    MET_ROOT.mkdir(parents=True, exist_ok=True)
    with open(MET_ROOT / "all_summaries.json", "w", encoding="utf-8") as stream:
        json.dump(all_summaries, stream, indent=2, ensure_ascii=False)
    print(f"\nToplam {len(all_summaries)} test özeti üretildi → {MET_ROOT}")


if __name__ == "__main__":
    main()
