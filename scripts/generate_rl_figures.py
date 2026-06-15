#!/usr/bin/env python3
"""
SARA — RL doğrulama figür üreticisi (corrected, raw-telemetry tabanlı)
======================================================================

Bu script, RL UKF/GT teşhisinden çıkan *düzeltilmiş* özet CSV'sini ve tek
episode telemetri kaydını kullanarak README/Wiki için RL figürlerini üretir.

Girdi dosyaları (repo içinde mevcut):
  docs/diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv
  docs/diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv
  data/episodes/sara_best_episode.csv        (tek episode, calm/no-current sim state)

Üretilen figürler -> docs/figures/rl/ :
  rl_episode_comparison_matrix.png   senaryo x {UKF aligned RMSE, cross-track RMSE} + durum
  rl_current_robustness.png          ordinal akıntı şiddeti vs cross-track RMSE
  rl_ukf_raw_vs_aligned_rmse.png     raw vs başlangıç-hizalı UKF RMSE (CSV'den yeniden)
  rl_trajectory_overlay.png          AUV yörüngesi (calm episode) vs referans hat

DÜRÜSTLÜK NOTU:
  - UKF RMSE değerleri ham `/odometry/ukf` kaydından *başlangıç hizalı* yeniden
    hesaplanmıştır (teşhis paketi). Bu repo bundle'ı ham per-step telemetry
    CSV'lerini İÇERMEZ; bu yüzden figürler teşhis özet CSV'sinden üretilir.
  - trajectory_overlay yalnızca tek episode'un (calm) sim durumunu çizer; ayrı
    GT/UKF/RL kanalları bu bundle'da olmadığından 4 ayrı iz ÇİZİLEMEZ. Bu durum
    figür başlığında ve README'de açıkça belirtilmiştir.
"""
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

# Ordinal senaryo şiddeti (fiziksel büyüklükle birebir değil — bkz README)
SEVERITY = {
    "no_current": 0, "following_current": 1, "cross_current": 2,
    "diagonal_current": 3, "reverse_current": 4, "hard_cross_current": 5,
}
PROGRESS_TARGET_M = 50.0       # SARA görevi: 50 m ileri
FINAL_CROSS_TOL_M = 1.0        # hedefe oturma toleransı


def load_summary():
    df = pd.read_csv(DIAG / "corrected_rl_ukf_summary_from_raw_telemetry.csv")
    df["severity"] = df["episode"].map(SEVERITY)
    df = df.sort_values("severity").reset_index(drop=True)
    return df


def status_label(row):
    """Şeffaf kriter: hedefe ulaştı mı? (progress>=50 m ve |final_cross|<1 m).
    Bu yalnızca aday politikanın hedefe oturmasını ölçer; eğitilmiş SAC değildir."""
    reached = (row["progress_m"] >= PROGRESS_TARGET_M
               and abs(row["final_cross_m"]) < FINAL_CROSS_TOL_M)
    return "PASS*" if reached else "WIP"


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

    axes[1].bar(x, df["cross_track_rmse_m"], color="#1F4E79")
    axes[1].set_xticks(x); axes[1].set_xticklabels(labels, rotation=30, ha="right")
    axes[1].set_ylabel("Cross-track RMSE [m]")
    axes[1].set_title("Yana sapma (cross-track) RMSE")
    for i, row in df.iterrows():
        st = status_label(row)
        axes[1].text(i, row["cross_track_rmse_m"], f"{row['cross_track_rmse_m']:.2f}\n{st}",
                     ha="center", va="bottom", fontsize=8)

    fig.suptitle("RL aday politikası — akıntı senaryosu karşılaştırma matrisi\n"
                 "(*PASS = aday politika hedefe ulaştı: progress≥50 m & |final cross|<1 m; "
                 "eğitilmiş SAC değildir)", fontsize=11)
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


def fig_trajectory_overlay():
    epi = ROOT / "data" / "episodes" / "sara_best_episode.csv"
    if not epi.exists():
        return None
    df = pd.read_csv(epi)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(df["x"], df["y"], lw=2, color="#1F4E79", label="AUV yörüngesi (calm episode, sim state)")
    ax.plot([0, PROGRESS_TARGET_M], [0, 0], "--", color="#7F7F7F", lw=1.4, label="Referans hat (y=0, 0→50 m)")
    ax.scatter([df["x"].iloc[0]], [df["y"].iloc[0]], c="green", s=60, zorder=5, label="Başlangıç")
    ax.scatter([df["x"].iloc[-1]], [df["y"].iloc[-1]], c="red", s=60, zorder=5, label="Bitiş")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Yörünge — tek episode (calm).\n"
                 "Not: bu bundle'da ayrı GT/UKF/RL kanalları yok; tek sim-state izi gösterilir.")
    ax.legend(fontsize=8)
    fig.tight_layout()
    p = OUT / "rl_trajectory_overlay.png"
    fig.savefig(p); plt.close(fig)
    return p


def main():
    df = load_summary()
    produced = [fig_comparison_matrix(df), fig_current_robustness(df),
                fig_raw_vs_aligned(df), fig_trajectory_overlay()]
    print("[✓] Üretilen figürler:")
    for p in produced:
        if p:
            print("    -", p.relative_to(ROOT))
    # Konsola özet doğrulama tablosu
    print("\n[i] Düzeltilmiş RL özeti (ham telemetriden):")
    cols = ["episode", "progress_m", "final_cross_m", "cross_track_rmse_m",
            "ukf_raw_rmse_m", "ukf_aligned_rmse_m", "nav_valid_ratio"]
    print(df[cols].to_string(index=False))


if __name__ == "__main__":
    main()
