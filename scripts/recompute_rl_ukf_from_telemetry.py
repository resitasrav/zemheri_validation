#!/usr/bin/env python3
"""
RL UKF–GT RMSE'yi ham telemetriden yeniden hesaplar (DÜZELTİLMİŞ hizalama).
==========================================================================

Bu script, `final_validation` sonuç klasöründeki RL episode'larının ham
`recording/telemetry.csv` kayıtlarını okuyup UKF–GT konum hatasını **doğru zaman
hizalamasıyla** yeniden hesaplar. Üretici scriptteki (`rl_policy_validation.py`)
donmuş-UKF-kolon hatasından bağımsızdır.

Üretici hatası (kök neden):
  `rl_policy_validation.py` içinde `ukf` DataFrame'i, başlangıç zamanı (start)
  çıkarma döngüsünden ÖNCE kopyalanır; döngü yalnızca `frames` sözlüğündeki
  orijinalleri kaydırır, `ukf` kopyasını kaydırmaz. Böylece `merge_asof(nearest)`
  her GT satırını İLK UKF örneğine eşler → x_ukf/y_ukf/z_ukf episode boyunca DONAR
  → position_error ~ görev mesafesi (~50 m) kadar büyür (sahte ~35 m UKF RMSE).

Bu script telemetriyi doğrudan okuduğu için bu hatadan etkilenmez.

Kullanım:
  python scripts/recompute_rl_ukf_from_telemetry.py <final_validation/results> [--out OUT.csv]

NOT: Ham telemetry.csv dosyaları büyüktür (~50–65 MB) ve REPOYA EKLENMEZ. Bu script
sadece harici sonuç klasörünü işler; çıktısı küçük bir özet CSV'dir.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

GT_TOPIC = "/ground_truth/odometry"
UKF_TOPIC = "/odometry/ukf"

# results klasör adı -> senaryo etiketi
SCENARIO_BY_DIR = {
    "rl_policy_20260615_140215": "no_current",
    "rl_policy_20260615_140405": "following_current",
    "rl_policy_20260615_140556": "cross_current",
    "rl_policy_20260615_140746": "diagonal_current",
    "rl_policy_20260615_140938": "reverse_current",
    "rl_policy_20260615_141143": "hard_cross_current",
}


def load_odom(telemetry_path):
    """telemetry.csv'den GT ve UKF (t, x, y, z) dizilerini çıkarır."""
    gt, ukf = [], []
    with open(telemetry_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            topic = row.get("topic")
            if topic not in (GT_TOPIC, UKF_TOPIC):
                continue
            try:
                data = json.loads(row["data_json"])
                pos = data["pose"]["pose"]["position"]
                t = float(row["ros_time"])
            except (KeyError, ValueError, TypeError):
                continue
            (gt if topic == GT_TOPIC else ukf).append((t, pos["x"], pos["y"], pos["z"]))
    return np.asarray(gt, dtype=float), np.asarray(ukf, dtype=float)


def nearest_align(gt, ukf):
    """UKF'i GT zaman tabanına en yakın komşu ile hizalar (doğru ortak taban)."""
    gt = gt[np.argsort(gt[:, 0])]
    ukf = ukf[np.argsort(ukf[:, 0])]
    idx = np.searchsorted(ukf[:, 0], gt[:, 0])
    idx = np.clip(idx, 1, len(ukf) - 1)
    left = ukf[idx - 1]
    right = ukf[idx]
    choose_left = np.abs(gt[:, 0] - left[:, 0]) <= np.abs(gt[:, 0] - right[:, 0])
    ukf_al = np.where(choose_left[:, None], left, right)
    return gt[:, 1:4], ukf_al[:, 1:4]


def compute(gt_xyz, ukf_xyz):
    err_raw = np.linalg.norm(gt_xyz - ukf_xyz, axis=1)
    rmse_raw = float(np.sqrt(np.mean(err_raw ** 2)))
    gt_a = gt_xyz - gt_xyz[0]
    ukf_a = ukf_xyz - ukf_xyz[0]
    err_al = np.linalg.norm(gt_a - ukf_a, axis=1)
    rmse_al = float(np.sqrt(np.mean(err_al ** 2)))
    final_al = float(err_al[-1])
    progress = float(np.linalg.norm(gt_xyz[-1, :2] - gt_xyz[0, :2]))
    return rmse_raw, rmse_al, final_al, progress


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path, help="final_validation/results klasörü")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    rows = []
    for dname, scenario in SCENARIO_BY_DIR.items():
        tel = args.results / dname / "recording" / "telemetry.csv"
        if not tel.exists():
            print(f"[!] {scenario}: telemetry yok ({tel})")
            continue
        gt, ukf = load_odom(tel)
        if len(gt) == 0 or len(ukf) == 0:
            print(f"[!] {scenario}: GT/UKF örneği yok")
            continue
        gt_xyz, ukf_xyz = nearest_align(gt, ukf)
        rmse_raw, rmse_al, final_al, progress = compute(gt_xyz, ukf_xyz)
        ukf_x_span = float(ukf[:, 1].max() - ukf[:, 1].min())
        rows.append((scenario, dname, len(gt), len(ukf), progress,
                     rmse_raw, rmse_al, final_al, ukf_x_span))
        print(f"{scenario:20s} raw={rmse_raw:6.3f}  aligned={rmse_al:6.3f}  "
              f"final_aligned={final_al:6.3f}  progress={progress:6.2f}  "
              f"ukf_x_span={ukf_x_span:6.2f}")

    if args.out and rows:
        with open(args.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["episode", "dir", "gt_samples", "ukf_samples", "progress_m",
                        "ukf_raw_rmse_m", "ukf_aligned_rmse_m",
                        "ukf_final_aligned_error_m", "raw_telemetry_ukf_x_span_m"])
            w.writerows(rows)
        print(f"\n[✓] yazıldı: {args.out}")


if __name__ == "__main__":
    main()
