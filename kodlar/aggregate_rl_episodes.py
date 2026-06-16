#!/usr/bin/env python3
"""Aggregate the latest isolated RL policy validation episodes."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DARK = "#1C2541"
MUTED_RED = "#B56576"
BLUE = "#6D8BB0"
CORAL = "#E56B6F"
GREEN = "#4D9078"
PURPLE = "#8E6C88"

EPISODES = [
    ("rl_policy", "Akıntısız"),
    ("rl_policy_following_current", "Takip eden"),
    ("rl_policy_cross_current", "Çapraz"),
    ("rl_policy_diagonal_current", "Diyagonal"),
    ("rl_policy_reverse_current", "Ters"),
    ("rl_policy_hard_cross_current", "Güçlü çapraz"),
]


def configure_plot():
    plt.rcParams.update({
        "axes.grid": True,
        "grid.alpha": 0.25,
        "grid.linestyle": "--",
        "font.family": "DejaVu Sans",
        "savefig.dpi": 240,
        "savefig.bbox": "tight",
    })


def latest_outputs(results_root):
    latest = {}
    for directory in results_root.iterdir():
        manifest_path = directory / "test_manifest.yaml"
        summary_path = directory / "metrics" / "rl_policy_summary.csv"
        timeline_path = directory / "metrics" / "rl_policy_timeseries.csv"
        if not directory.is_dir() or not (
            manifest_path.exists() and summary_path.exists() and timeline_path.exists()
        ):
            continue
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
        case = manifest.get("case")
        if case not in dict(EPISODES):
            continue
        if case not in latest or directory.name > latest[case][0].name:
            latest[case] = (directory, manifest)
    missing = [case for case, _label in EPISODES if case not in latest]
    if missing:
        raise RuntimeError("Eksik RL episode sonuçları: " + ", ".join(missing))
    return latest


def load_episodes(results_root):
    outputs = latest_outputs(results_root)
    rows = []
    timelines = []
    for case, label in EPISODES:
        directory, manifest = outputs[case]
        summary = pd.read_csv(
            directory / "metrics" / "rl_policy_summary.csv"
        ).iloc[0]
        timeline = pd.read_csv(
            directory / "metrics" / "rl_policy_timeseries.csv"
        )
        current = manifest["runner_arguments"]["current_target_mps"]
        target_distance = float(summary["Hedef mesafe (m)"])
        progress = float(summary["İlerleme (m)"])
        rows.append({
            "Vaka": case,
            "Episode": label,
            "Politika kararı": summary["Doğrulama kararı"],
            "Durdurma nedeni": manifest.get("stop_reason", ""),
            "Akıntı X (m/s)": float(current[0]),
            "Akıntı Y (m/s)": float(current[1]),
            "Test süresi (s)": float(summary["Test süresi (s)"]),
            "Hedef mesafe (m)": target_distance,
            "İlerleme (m)": progress,
            "İlerleme oranı": progress / max(target_distance, 1e-9),
            "Cross-track RMSE (m)": float(summary["Cross-track RMSE (m)"]),
            "Derinlik RMSE (m)": float(summary["Derinlik RMSE (m)"]),
            "UKF konum RMSE (m)": float(summary["UKF konum RMSE (m)"]),
            "UKF zaman eşleme ortalama hatası (ms)": float(
                summary["UKF zaman eşleme ortalama hatası (ms)"]
            ),
            "UKF zaman eşleme maksimum hatası (ms)": float(
                summary["UKF zaman eşleme maksimum hatası (ms)"]
            ),
            "Maksimum hız (m/s)": float(summary["Maksimum hız (m/s)"]),
            "DVL hız sınırı ihlal sayısı": int(
                summary["DVL hız sınırı ihlal sayısı"]
            ),
            "Navigasyon geçerli oranı": float(
                summary["Navigasyon geçerli oranı"]
            ),
            "Navigasyon degraded oranı": float(
                summary["Navigasyon degraded oranı"]
            ),
            "Çıktı klasörü": directory.name,
        })
        timelines.append((label, timeline))
    return pd.DataFrame(rows), timelines


def write_markdown(table, path):
    columns = list(table.columns)
    lines = [
        "| " + " | ".join(columns) + " |",
        "|" + "|".join(["---"] * len(columns)) + "|",
    ]
    for row in table.itertuples(index=False, name=None):
        values = [
            f"{value:.5f}" if isinstance(value, float) else str(value)
            for value in row
        ]
        lines.append("| " + " | ".join(values) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_episode_bars(table, output):
    labels = table["Episode"]
    x = np.arange(len(labels))
    colors = [DARK, BLUE, CORAL, MUTED_RED, GREEN, PURPLE]
    width = 0.36

    fig, axes = plt.subplots(3, 1, figsize=(10.0, 10.0), sharex=True)
    axes[0].bar(x, table["İlerleme oranı"], color=colors)
    axes[0].axhline(0.90, color=DARK, linestyle="--", label="Kabul eşiği")
    axes[0].set_ylabel("İlerleme oranı")
    axes[0].legend()

    axes[1].bar(
        x - width / 2, table["Cross-track RMSE (m)"], width,
        color=CORAL, label="Cross-track RMSE",
    )
    axes[1].bar(
        x + width / 2, table["Derinlik RMSE (m)"], width,
        color=BLUE, label="Derinlik RMSE",
    )
    axes[1].axhline(0.35, color=DARK, linestyle="--", label="Derinlik eşiği")
    axes[1].axhline(2.0, color=MUTED_RED, linestyle=":", label="Cross-track eşiği")
    axes[1].set_ylabel("RMSE (m)")
    axes[1].legend()

    axes[2].bar(x, table["UKF konum RMSE (m)"], color=colors)
    axes[2].set_ylabel("UKF RMSE (m)")
    axes[2].set_xticks(x, labels, rotation=20, ha="right")
    fig.tight_layout()
    fig.savefig(output / "rl_episode_performance_bars.png")
    plt.close(fig)


def save_combined_trajectory(timelines, output):
    fig, ax = plt.subplots(figsize=(9.5, 5.8))
    colors = [DARK, BLUE, CORAL, MUTED_RED, GREEN, PURPLE]
    for (name, timeline), color in zip(timelines, colors):
        ax.plot(
            timeline["relative_x"], timeline["relative_y"],
            color=color, linewidth=1.6, label=name,
        )
    ax.axhline(
        0.0, color=DARK, linestyle="--", linewidth=1.0, label="Referans rota"
    )
    ax.set_xlabel("Yerel X (m)")
    ax.set_ylabel("Yerel Y (m)")
    ax.legend(ncol=2)
    ax.axis("equal")
    fig.tight_layout()
    fig.savefig(output / "rl_episode_trajectories.png")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("analysis/final_validation/results"),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    results_root = args.results_root.expanduser().resolve()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    table, timelines = load_episodes(results_root)
    configure_plot()
    table.to_csv(output / "rl_episode_comparison.csv", index=False)
    write_markdown(table, output / "rl_episode_comparison.md")
    save_episode_bars(table, output)
    save_combined_trajectory(timelines, output)

    summary = pd.DataFrame([{
        "Episode sayısı": len(table),
        "Kabul edilen episode sayısı": int(
            table["Politika kararı"].astype(str).str.startswith("KABUL").sum()
        ),
        "Hedefe ulaşan episode sayısı": int(
            (table["Durdurma nedeni"] == "rl_target_reached").sum()
        ),
        "DVL hız sınırı toplam ihlal sayısı": int(
            table["DVL hız sınırı ihlal sayısı"].sum()
        ),
        "Ortalama cross-track RMSE (m)": float(
            table["Cross-track RMSE (m)"].mean()
        ),
        "Ortalama derinlik RMSE (m)": float(table["Derinlik RMSE (m)"].mean()),
        "Ortalama UKF konum RMSE (m)": float(
            table["UKF konum RMSE (m)"].mean()
        ),
    }])
    summary.to_csv(output / "rl_episode_aggregate_summary.csv", index=False)
    write_markdown(summary, output / "rl_episode_aggregate_summary.md")


if __name__ == "__main__":
    main()
