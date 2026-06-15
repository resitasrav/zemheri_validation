#!/usr/bin/env python3
"""
SARA - Doğrulama / RL figür üretici
===================================

Bir test çıktı klasöründeki kayıt (recording) CSV'lerini okuyup standart
figür setini üretir. RL episode klasörleri de aynı şemayı kullandığı için
bu araç RL telemetrisi için de doğrudan çalışır.

Okunan dosyalar (varsa, hepsi opsiyonel):
  recording/telemetry.csv                    -> GT/UKF odometri (trajectory, hata)
  recording/known_signal_timeseries.csv      -> derinlik / hız / komut sinyalleri
  metrics/ocean_current_service_timeseries.csv (veya benzeri) -> akıntı aktivitesi

Üretilen figürler (üretilebilenler):
  trajectory_xy.png            GT vs UKF yatay yörünge
  depth_speed_tracking.png     derinlik & hız + hedefleri
  ukf_position_error.png       UKF konum hatasının zamanla değişimi
  ocean_current_activity.png   akıntı x/y/z + büyüklük (env. validation)
  rl_summary_panel.png         (RL klasörlerinde) tek panelde özet

Kullanım:
  python3 plot_validation_figures.py <recording_or_case_dir> [--out OUT_DIR] [--title BAŞLIK]

Örnekler:
  # RL episode kaydı için (csv'leriniz buradaysa):
  python3 plot_validation_figures.py results/rl_policy_ep06_hard_cross_current_XXXX --title "RL ep06 hard_cross"
  # Ortam doğrulama kaydı için:
  python3 plot_validation_figures.py results/ocean_current_services_20260615_135048
"""

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams.update({"figure.dpi": 130, "font.size": 10, "axes.grid": True,
                     "grid.alpha": 0.3})


# --------------------------------------------------------------------------- #
# Dosya bulma
# --------------------------------------------------------------------------- #
def find_file(root: Path, names):
    """root altında verilen adlardan ilk bulunanı döndürür."""
    for name in names:
        hits = sorted(root.rglob(name))
        if hits:
            return hits[0]
    return None


def yaw_from_quat(x, y, z, w):
    """Quaternion -> yaw (derece)."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


# --------------------------------------------------------------------------- #
# telemetry.csv -> odometri zaman serileri
# --------------------------------------------------------------------------- #
def load_odometry(telemetry_path: Path):
    """telemetry.csv'den GT ve UKF odometriyi çıkarır."""
    series = {
        "/ground_truth/odometry": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
        "/odometry/ukf": {"t": [], "x": [], "y": [], "z": [], "yaw": []},
    }
    if telemetry_path is None or not telemetry_path.exists():
        return series

    wanted = set(series)
    with telemetry_path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            topic = row.get("topic")
            if topic not in wanted:
                continue
            try:
                data = json.loads(row["data_json"])
                pose = data["pose"]["pose"]
                pos = pose["position"]
                ori = pose["orientation"]
            except (KeyError, ValueError, TypeError):
                continue
            store = series[topic]
            store["t"].append(float(row["ros_time"]))
            store["x"].append(pos["x"])
            store["y"].append(pos["y"])
            store["z"].append(pos["z"])
            store["yaw"].append(yaw_from_quat(ori["x"], ori["y"], ori["z"], ori["w"]))

    for store in series.values():
        for key in store:
            store[key] = np.asarray(store[key], dtype=float)
    return series


# --------------------------------------------------------------------------- #
# known_signal_timeseries.csv -> derinlik/hız sinyalleri
# --------------------------------------------------------------------------- #
def load_known_signals(path: Path):
    cols = ["time", "depth", "speed", "target_depth", "target_speed",
            "cmd_forward", "cmd_vertical", "cmd_yaw_rate"]
    out = {c: [] for c in cols}
    if path is None or not path.exists():
        return {c: np.asarray([]) for c in cols}

    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            for c in cols:
                val = row.get(c, "")
                out[c].append(float(val) if val not in ("", None) else np.nan)
    arrays = {c: np.asarray(out[c], dtype=float) for c in cols}
    if arrays["time"].size:
        arrays["time"] = arrays["time"] - np.nanmin(arrays["time"])
    return arrays


def load_current(path: Path):
    cols = ["t", "x", "y", "z", "magnitude"]
    out = {c: [] for c in cols}
    if path is None or not path.exists():
        return {c: np.asarray([]) for c in cols}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            for c in cols:
                val = row.get(c, "")
                out[c].append(float(val) if val not in ("", None) else np.nan)
    return {c: np.asarray(out[c], dtype=float) for c in cols}


# --------------------------------------------------------------------------- #
# Figürler
# --------------------------------------------------------------------------- #
def fig_trajectory(series, out_dir, title):
    gt, ukf = series["/ground_truth/odometry"], series["/odometry/ukf"]
    if gt["x"].size == 0:
        return None
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(gt["x"], gt["y"], label="Ground truth", lw=2)
    if ukf["x"].size:
        ax.plot(ukf["x"], ukf["y"], "--", label="UKF tahmini", lw=1.6)
    ax.scatter(gt["x"][0], gt["y"][0], c="green", s=60, zorder=5, label="Başlangıç")
    ax.scatter(gt["x"][-1], gt["y"][-1], c="red", s=60, zorder=5, label="Bitiş")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Yatay yörünge — {title}")
    ax.axis("equal")
    ax.legend()
    path = out_dir / "trajectory_xy.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_position_error(series, out_dir, title):
    gt, ukf = series["/ground_truth/odometry"], series["/odometry/ukf"]
    if gt["x"].size == 0 or ukf["x"].size == 0:
        return None
    # UKF zamanını GT zamanına interpolasyon ile eşle
    t0 = gt["t"][0]
    gx = np.interp(ukf["t"], gt["t"], gt["x"])
    gy = np.interp(ukf["t"], gt["t"], gt["y"])
    gz = np.interp(ukf["t"], gt["t"], gt["z"])
    err = np.sqrt((ukf["x"] - gx) ** 2 + (ukf["y"] - gy) ** 2 + (ukf["z"] - gz) ** 2)
    rmse = float(np.sqrt(np.mean(err ** 2)))
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(ukf["t"] - t0, err, lw=1.4)
    ax.axhline(rmse, color="red", ls="--", lw=1, label=f"RMSE = {rmse:.3f} m")
    ax.set_xlabel("Süre (s)")
    ax.set_ylabel("UKF konum hatası (m)")
    ax.set_title(f"UKF konum hatası — {title}")
    ax.legend()
    path = out_dir / "ukf_position_error.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path, rmse


def fig_depth_speed(signals, out_dir, title):
    if signals["time"].size == 0:
        return None
    t = signals["time"]
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, signals["depth"], label="Derinlik (ölçülen)", lw=1.3)
    if np.isfinite(signals["target_depth"]).any():
        axes[0].plot(t, signals["target_depth"], "--", label="Derinlik hedefi", lw=1.3)
    axes[0].set_ylabel("Derinlik (m)")
    axes[0].legend()
    axes[1].plot(t, signals["speed"], label="Hız (ölçülen)", lw=1.3)
    if np.isfinite(signals["target_speed"]).any():
        axes[1].plot(t, signals["target_speed"], "--", label="Hız hedefi", lw=1.3)
    axes[1].set_ylabel("Hız (m/s)")
    axes[1].set_xlabel("Süre (s)")
    axes[1].legend()
    fig.suptitle(f"Derinlik & hız takibi — {title}")
    path = out_dir / "depth_speed_tracking.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_current(current, out_dir, title):
    if current["t"].size == 0:
        return None
    t = current["t"]
    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(t, current["x"], color="tab:blue")
    axes[0].set_ylabel("X (m/s)")
    axes[1].plot(t, current["y"], color="tab:orange")
    axes[1].set_ylabel("Y (m/s)")
    axes[2].plot(t, current["z"], color="tab:green")
    axes[2].set_ylabel("Z (m/s)")
    axes[3].plot(t, current["magnitude"], color="tab:red")
    axes[3].set_ylabel("|akıntı| (m/s)")
    axes[3].set_xlabel("Süre (s)")
    fig.suptitle(f"Okyanus akıntısı aktivitesi — {title}")
    path = out_dir / "ocean_current_activity.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def fig_rl_summary(series, signals, current, out_dir, title, rmse=None):
    """RL klasörleri için tek panelde özet (jüri-dostu)."""
    gt, ukf = series["/ground_truth/odometry"], series["/odometry/ukf"]
    if gt["x"].size == 0 and signals["time"].size == 0:
        return None
    fig = plt.figure(figsize=(12, 8))
    gs = fig.add_gridspec(2, 2)

    ax_traj = fig.add_subplot(gs[:, 0])
    if gt["x"].size:
        ax_traj.plot(gt["x"], gt["y"], label="GT", lw=2)
    if ukf["x"].size:
        ax_traj.plot(ukf["x"], ukf["y"], "--", label="UKF", lw=1.5)
    ax_traj.set_xlabel("x (m)")
    ax_traj.set_ylabel("y (m)")
    ax_traj.set_title("Yörünge")
    ax_traj.axis("equal")
    ax_traj.legend()

    ax_depth = fig.add_subplot(gs[0, 1])
    if signals["time"].size:
        ax_depth.plot(signals["time"], signals["depth"], label="derinlik")
        if np.isfinite(signals["target_depth"]).any():
            ax_depth.plot(signals["time"], signals["target_depth"], "--", label="hedef")
    ax_depth.set_ylabel("Derinlik (m)")
    ax_depth.legend()

    ax_cur = fig.add_subplot(gs[1, 1])
    if current["t"].size:
        ax_cur.plot(current["t"], current["magnitude"], color="tab:red")
        ax_cur.set_ylabel("|akıntı| (m/s)")
    elif signals["time"].size:
        ax_cur.plot(signals["time"], signals["speed"], color="tab:purple")
        ax_cur.set_ylabel("Hız (m/s)")
    ax_cur.set_xlabel("Süre (s)")

    subtitle = title if rmse is None else f"{title}  |  UKF RMSE = {rmse:.2f} m"
    fig.suptitle(f"RL özet — {subtitle}", fontsize=13)
    path = out_dir / "rl_summary_panel.png"
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("case_dir", type=Path, help="Test çıktı klasörü")
    parser.add_argument("--out", type=Path, default=None, help="Figür klasörü")
    parser.add_argument("--title", default=None, help="Başlık")
    parser.add_argument("--rl", action="store_true",
                        help="RL özet panelini de üret")
    args = parser.parse_args()

    case_dir = args.case_dir.resolve()
    if not case_dir.exists():
        sys.exit(f"Klasör bulunamadı: {case_dir}")

    title = args.title or case_dir.name
    out_dir = (args.out or (case_dir / "figures")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    telemetry = find_file(case_dir, ["telemetry.csv"])
    signals_csv = find_file(case_dir, ["known_signal_timeseries.csv"])
    current_csv = find_file(case_dir, [
        "ocean_current_service_timeseries.csv",
        "ocean_current_timeseries.csv",
    ])

    print(f"[i] case      : {case_dir}")
    print(f"[i] telemetry : {telemetry}")
    print(f"[i] signals   : {signals_csv}")
    print(f"[i] current   : {current_csv}")

    series = load_odometry(telemetry)
    signals = load_known_signals(signals_csv)
    current = load_current(current_csv)

    produced = []
    rmse = None
    p = fig_trajectory(series, out_dir, title)
    if p:
        produced.append(p)
    res = fig_position_error(series, out_dir, title)
    if res:
        produced.append(res[0])
        rmse = res[1]
    p = fig_depth_speed(signals, out_dir, title)
    if p:
        produced.append(p)
    p = fig_current(current, out_dir, title)
    if p:
        produced.append(p)
    if args.rl:
        p = fig_rl_summary(series, signals, current, out_dir, title, rmse)
        if p:
            produced.append(p)

    print("\n[✓] Üretilen figürler:")
    for path in produced:
        print(f"    - {path}")
    if not produced:
        print("    (uygun CSV bulunamadı — telemetry / known_signal / current CSV gerekli)")


if __name__ == "__main__":
    main()
