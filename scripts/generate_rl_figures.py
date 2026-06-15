#!/usr/bin/env python3
"""
SARA — RL doğrulama figür üreticisi (corrected, raw-telemetry tabanlı)
======================================================================

Bu script RL figürlerini repo içindeki *düzeltilmiş* özet CSV'den üretir; trajectory
overlay'i ise (varsa) harici ham telemetriden çizer.

Girdi (repoda mevcut):
  docs/diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv

Opsiyonel (harici, repoda DEĞİL — büyük ham telemetri):
  --results <final_validation/results>   trajectory overlay için ham telemetry.csv

Üretilen figürler -> docs/figures/rl/ :
  rl_episode_comparison_matrix.png   senaryo x {UKF aligned RMSE, depth RMSE} + karar
  rl_current_robustness.png          ordinal akıntı şiddeti vs cross-track RMSE
  rl_ukf_raw_vs_aligned_rmse.png     raw vs başlangıç-hizalı UKF RMSE
  rl_trajectory_overlay.png          GT + UKF (gerçek telemetri) + referans hat

KABUL KRİTERİ (gerçek, rl_policy_validation.py'den):
  progress >= 0.90*target  AND  depth_rmse <= 0.35  AND  max_speed <= 2.5  AND  nav_valid >= 0.95
"""
import argparse
import csv
import json
import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DIAG = ROOT / "docs" / "diagnostics" / "rl_ukf"
OUT = ROOT / "docs" / "figures" / "rl"
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({"figure.dpi": 130, "font.size": 10,
                     "axes.grid": True, "grid.alpha": 0.3})

SEVERITY = {
    "no_current": 0, "following_current": 1, "cross_current": 2,
    "diagonal_current": 3, "reverse_current": 4, "hard_cross_current": 5,
}
DEPTH_RMSE_THRESHOLD = 0.35     # rl_policy_validation.py kabul eşiği

GT_TOPIC = "/ground_truth/odometry"
UKF_TOPIC = "/odometry/ukf"


def load_summary():
    df = pd.read_csv(DIAG / "corrected_rl_ukf_summary_from_raw_telemetry.csv")
    df["severity"] = df["episode"].map(SEVERITY)
    return df.sort_values("severity").reset_index(drop=True)


def status_label(row):
    """Gerçek kabul kriterinin belirleyici bileşeni: depth RMSE <= 0.35 m.
    Hepsi eşiği aştığı için aday politika 'eşik altı' (WIP)."""
    return "PASS" if row["depth_rmse_m"] <= DEPTH_RMSE_THRESHOLD else "below thr."


def fig_comparison_matrix(df):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    labels = [e.replace("_current", "").replace("_", " ") for e in df["episode"]]
    x = np.arange(len(df))

    axes[0].bar(x, df["ukf_aligned_rmse_m"], color="#548235")
    axes[0].set_xticks(x); axes[0].set_xticklabels(labels, rotation=30, ha="right")
    axes[0].set_ylabel("UKF konum RMSE (başlangıç hizalı) [m]")
    axes[0].set_title("UKF–GT konum RMSE (ham telemetriden, hizalı)")
    for i, v in enumerate(df["ukf_aligned_rmse_m"]):
        axes[0].text(i, v, f"{v:.2f}", ha="center", va="bottom", fontsize=8)

    axes[1].bar(x, df["depth_rmse_m"], color="#1F4E79")
    axes[1].axhline(DEPTH_RMSE_THRESHOLD, color="#C00000", ls="--", lw=1.2,
                    label=f"kabul eşiği {DEPTH_RMSE_THRESHOLD} m")
    axes[1].set_xticks(x); axes[1].set_xticklabels(labels, rotation=30, ha="right")
    axes[1].set_ylabel("Derinlik RMSE [m]")
    axes[1].set_title("Derinlik RMSE — aday politika kabul belirleyicisi")
    axes[1].legend()
    for i, row in df.iterrows():
        axes[1].text(i, row["depth_rmse_m"], f"{row['depth_rmse_m']:.2f}\n{status_label(row)}",
                     ha="center", va="bottom", fontsize=8)

    fig.suptitle("RL aday politikası — akıntı senaryosu karşılaştırma matrisi\n"
                 "(UKF RMSE düzeltilmiş; kabul derinlik RMSE ≤ 0.35 m kriterine bağlı — "
                 "6 senaryoda da eşik altı. Eğitilmiş SAC değildir.)", fontsize=11)
    fig.tight_layout()
    p = OUT / "rl_episode_comparison_matrix.png"
    fig.savefig(p); plt.close(fig)
    return p


def fig_current_robustness(df):
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(df["severity"], df["cross_track_rmse_m"], "o-", color="#C00000", lw=1.8, ms=8)
    for _, row in df.iterrows():
        ax.annotate(row["episode"].replace("_current", ""),
                    (row["severity"], row["cross_track_rmse_m"]),
                    textcoords="offset points", xytext=(6, 6), fontsize=8)
    ax.set_xlabel("Ordinal akıntı şiddeti (senaryo sırası, fiziksel büyüklük değil)")
    ax.set_ylabel("Cross-track RMSE [m]")
    ax.set_title("Akıntı dayanıklılığı — ordinal şiddet vs yana sapma RMSE")
    ax.set_xticks(list(SEVERITY.values()))
    ax.set_xticklabels([k.replace("_current", "") for k in SEVERITY], rotation=20, ha="right")
    fig.tight_layout()
    p = OUT / "rl_current_robustness.png"
    fig.savefig(p); plt.close(fig)
    return p


def fig_raw_vs_aligned(df):
    fig, ax = plt.subplots(figsize=(9, 5))
    labels = [e.replace("_current", "") for e in df["episode"]]
    x = np.arange(len(df)); w = 0.38
    ax.bar(x - w / 2, df["ukf_raw_rmse_m"], w, label="Raw RMSE (origin farkı dahil)", color="#F4B183")
    ax.bar(x + w / 2, df["ukf_aligned_rmse_m"], w, label="Aligned RMSE (başlangıç çıkarıldı)", color="#548235")
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel("UKF–GT konum RMSE [m]")
    ax.set_title("UKF RMSE: raw vs başlangıç-hizalı (ham telemetriden yeniden hesap)")
    ax.legend()
    for i, row in df.iterrows():
        ax.text(i - w / 2, row["ukf_raw_rmse_m"], f"{row['ukf_raw_rmse_m']:.2f}", ha="center", va="bottom", fontsize=7)
        ax.text(i + w / 2, row["ukf_aligned_rmse_m"], f"{row['ukf_aligned_rmse_m']:.2f}", ha="center", va="bottom", fontsize=7)
    fig.tight_layout()
    p = OUT / "rl_ukf_raw_vs_aligned_rmse.png"
    fig.savefig(p); plt.close(fig)
    return p


def _load_telemetry(path, topics):
    out = {t: [] for t in topics}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            topic = row.get("topic")
            if topic not in topics:
                continue
            try:
                data = json.loads(row["data_json"])
                if topic in (GT_TOPIC, UKF_TOPIC):
                    pos = data["pose"]["pose"]["position"]
                    out[topic].append((float(row["ros_time"]), pos["x"], pos["y"]))
                else:  # /guidance/goal
                    out[topic].append((float(row["ros_time"]), data.get("target_x"), data.get("target_y")))
            except (KeyError, ValueError, TypeError):
                continue
    return {t: np.asarray(v, dtype=float) for t, v in out.items()}


def fig_trajectory_overlay(results_dir, scenario_dir="rl_policy_20260615_140215",
                           scenario="no_current"):
    """GT + UKF (gerçek telemetri) + referans hat. Başlangıçlar hizalanır."""
    tel = None
    if results_dir:
        cand = Path(results_dir) / scenario_dir / "recording" / "telemetry.csv"
        if cand.exists():
            tel = cand
    fig, ax = plt.subplots(figsize=(9, 5.4))
    if tel is not None:
        data = _load_telemetry(tel, {GT_TOPIC, UKF_TOPIC, "/guidance/goal"})
        gt, ukf, goal = data[GT_TOPIC], data[UKF_TOPIC], data["/guidance/goal"]
        gx, gy = gt[:, 1] - gt[0, 1], gt[:, 2] - gt[0, 2]
        ux, uy = ukf[:, 1] - ukf[0, 1], ukf[:, 2] - ukf[0, 2]
        ax.plot(gx, gy, color="#E56B6F", lw=2.2, label="Ground truth (= politikanın sürdüğü iz)")
        ax.plot(ux, uy, color="#6D8BB0", lw=1.5, ls="--", label="UKF kestirimi (gerçek telemetri)")
        if len(goal):
            tx = float(goal[-1, 1]) - gt[0, 1]
            ty = float(goal[-1, 2]) - gt[0, 2]
            ax.plot([0, tx], [0, ty], color="#1C2541", lw=1.5, ls=":", label="Referans hat (LOS hedefi)")
        ax.scatter([0], [0], c="green", s=55, zorder=5, label="Başlangıç")
        ax.scatter([gx[-1]], [gy[-1]], c="red", s=55, zorder=5, label="Bitiş")
        ax.set_title(f"Yörünge overlay — {scenario} (ham telemetriden)\n"
                     "Not: 'RL path' ayrı bir kanal değildir; politikanın sürdüğü iz = Ground truth.")
        src = "ham telemetri"
    else:
        epi = ROOT / "data" / "episodes" / "sara_best_episode.csv"
        df = pd.read_csv(epi)
        ax.plot(df["x"], df["y"], lw=2, color="#1F4E79", label="AUV izi (calm episode, sim state)")
        ax.plot([0, 50], [0, 0], "--", color="#7F7F7F", lw=1.4, label="Referans hat (y=0)")
        ax.set_title("Yörünge — tek episode (calm, sim state).\n"
                     "Not: ham telemetri verilmedi; ayrı GT/UKF kanalları gösterilemez.")
        src = "episode CSV (fallback)"
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(fontsize=8)
    fig.tight_layout()
    p = OUT / "rl_trajectory_overlay.png"
    fig.savefig(p); plt.close(fig)
    print(f"[i] trajectory overlay kaynağı: {src}")
    return p


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", default=None,
                        help="final_validation/results (trajectory overlay için ham telemetri)")
    args = parser.parse_args()

    df = load_summary()
    produced = [fig_comparison_matrix(df), fig_current_robustness(df),
                fig_raw_vs_aligned(df), fig_trajectory_overlay(args.results)]
    print("[OK] Uretilen figurler:")
    for p in produced:
        print("    -", p.relative_to(ROOT))
    cols = ["episode", "progress_m", "depth_rmse_m", "cross_track_rmse_m",
            "ukf_raw_rmse_m", "ukf_aligned_rmse_m", "nav_valid_ratio"]
    print("\n[i] Duzeltilmis RL ozeti:")
    print(df[cols].to_string(index=False))


if __name__ == "__main__":
    main()
