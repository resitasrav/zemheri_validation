# RL UKF–GT Diagnosis

[← README](../../README.md) · Kaynak teşhis notu: [docs/diagnostics/rl_ukf/RL_UKF_GT_DIAGNOSIS.md](../diagnostics/rl_ukf/RL_UKF_GT_DIAGNOSIS.md)

## Table of Contents
- [Sorun](#sorun)
- [Kanıt (gerçek dosyalardan doğrulanmış)](#kanıt-gerçek-dosyalardan-doğrulanmış)
- [Kök neden — koddaki tam hata](#kök-neden--koddaki-tam-hata)
- [Düzeltme](#düzeltme)
- [Düzeltilmiş sonuçlar (bağımsız yeniden hesap)](#düzeltilmiş-sonuçlar-bağımsız-yeniden-hesap)
- [Karar mantığı: BAŞARISIZ kararı UKF hatasından bağımsızdır](#karar-mantığı-başarısız-kararı-ukf-hatasından-bağımsızdır)
- [Evidence Files](#evidence-files)

## Sorun
İlk RL metrik çıktılarında **UKF konum RMSE ≈ 30–46 m** raporlanmıştı (README'de `hard_cross_current`
için 35.09 m). Bu değer diğer testlerdeki konum RMSE'leriyle (navigation straight ≈0.82 m, controller
≈0.20 m) iki kat büyüklük tutarsızdır ve şüphelidir.

## Kanıt (gerçek dosyalardan doğrulanmış)

`final_validation` paketindeki gerçek üretilmiş `metrics/rl_policy_timeseries.csv` (no_current)
dosyasından okunan değerler:

| Kolon | span (max−min) | ilk | son |
|---|---:|---:|---:|
| `x` (GT) | 50.11 m | 4.11 | 54.22 |
| `x_ukf` | **0.0000** | 0.1133 | 0.1133 |
| `y_ukf` | **0.0000** | 0.0469 | 0.0469 |
| `z_ukf` | **0.0000** | −0.1025 | −0.1025 |
| `position_error` | — | 0.0 | **50.14** |

`x_ukf`/`y_ukf`/`z_ukf` ilk örnekte **donmuş**; `position_error` GT ilerledikçe ~50 m'ye büyüyor →
sahte ~35 m RMSE. Aynı episode'un ham `recording/telemetry.csv` kaydında `/odometry/ukf` x değeri
**50.36 m boyunca normal ilerliyor** (UKF gerçekte akıyor).

## Kök neden — koddaki tam hata

Üretici: [docs/diagnostics/rl_ukf/legacy/rl_policy_validation_BUGGY.py](../diagnostics/rl_ukf/legacy/rl_policy_validation_BUGGY.py)
(`final_validation/test_codes/rl_policy_validation.py` ile birebir).

```python
# build_timeline() içinde:
ukf = frames["/odometry/ukf"].copy()            # satır 139: start ÇIKARILMADAN ÖNCE kopyalanır
...
start = max(gt["t"].min(), ukf["t"].min(), active_goals["t"].min())
gt = gt[gt["t"] >= start].copy()
gt["t"] -= start                                # gt kaydırılır
for frame in frames.values():
    frame["t"] -= start                         # frames sözlüğündeki ORİJİNALLER kaydırılır
                                                 #   ama satır 139'daki `ukf` KOPYASI değil!
...
timeline = nearest(gt, ukf, "ukf")              # satır 152: gt.t ∈ [0,T], ukf.t ∈ [~1.78e9, ...]
```

`nearest()` içinde `pd.merge_asof(direction="nearest")` çağrılır. `gt["t"]` görece zamanda
([0, T]) iken `ukf["t"]` hâlâ **mutlak epoch saniyesindedir** (`t = bag_time*1e-9 ≈ 1.78e9`,
read_bag satır 69). Tüm UKF zamanları tüm GT zamanlarından çok büyük olduğundan, merge_asof her GT
satırını **ilk UKF örneğine** eşler → `x_ukf/y_ukf/z_ukf` donar.

> **Özet:** Zaman tabanı uyuşmazlığı (UKF kopyası `start` ile normalize edilmedi) → `merge_asof`
> tüm UKF değerlerini ilk örneğe sabitledi → donmuş UKF kolonu → sahte 30–46 m RMSE.

## Düzeltme

[docs/diagnostics/rl_ukf/rl_policy_validation_fixed.py](../diagnostics/rl_ukf/rl_policy_validation_fixed.py)
— tek satırlık düzeltme (gt ile aynı normalizasyon):

```python
gt["t"] -= start
for frame in frames.values():
    frame["t"] -= start
ukf = ukf[ukf["t"] >= start].copy()
ukf["t"] -= start   # FIX: ukf kopyası da start ile normalize edilir
active_goals["t"] -= start
```

> Bu exporter ROS 2 (`rosbag2_py`) ve `.db3` bag gerektirir; ham bag'ler jüri reposuna dahil edilmez.
> ROS-bağımsız, doğrudan telemetriden çalışan doğrulama için
> [scripts/recompute_rl_ukf_from_telemetry.py](../../scripts/recompute_rl_ukf_from_telemetry.py).

## Düzeltilmiş sonuçlar (bağımsız yeniden hesap)

Ham `recording/telemetry.csv` dosyalarından **bu depodaki script ile bağımsız** yeniden hesaplanmıştır
([recomputed_rl_ukf_from_telemetry_verification.csv](../diagnostics/rl_ukf/recomputed_rl_ukf_from_telemetry_verification.csv)):

| Senaryo | Eski (buggy metrics) | Raw RMSE | **Aligned RMSE** | raw telemetry UKF x span |
|---|---:|---:|---:|---:|
| no_current | 30.34 m | 3.86 m | **0.73 m** | 50.36 m |
| following_current | 36.98 m | 4.13 m | **0.16 m** | 56.63 m |
| cross_current | 31.91 m | 4.01 m | **0.19 m** | 53.38 m |
| diagonal_current | 46.03 m | 4.12 m | **0.26 m** | 81.43 m |
| reverse_current | 32.88 m | 4.00 m | **0.09 m** | 50.86 m |
| hard_cross_current | 35.09 m | 3.94 m | **0.16 m** | 56.15 m |

Bu değerler teşhis paketinin
[corrected_rl_ukf_summary_from_raw_telemetry.csv](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv)
çıktısıyla **eşleşmektedir.** Başlangıç-hizalı RMSE **0.09–0.73 m** bandında ve diğer testlerle
tutarlıdır. Sonuç: eski 35 m değeri **gerçek UKF çökmesi değil, donmuş-kolon analiz artefaktıdır.**

<img src="../figures/rl/rl_ukf_raw_vs_aligned_rmse.png" width="780">
<img src="../diagnostics/rl_ukf/hard_cross_gt_ukf_alignment.png" width="780">

### RMSE tanımları
```python
error_raw     = norm(gt_xyz - ukf_xyz, axis=1);           rmse_raw     = sqrt(mean(error_raw**2))
gt_a, ukf_a   = gt_xyz - gt_xyz[0], ukf_xyz - ukf_xyz[0]
error_aligned = norm(gt_a - ukf_a, axis=1);               rmse_aligned = sqrt(mean(error_aligned**2))
```

## Karar mantığı: BAŞARISIZ kararı UKF hatasından bağımsızdır

Eski per-episode **BAŞARISIZ** kararı (6 senaryoda da), donmuş-UKF hatasından **etkilenmez.** Kabul
kriteri (`rl_policy_validation.py:300-305`) UKF konum hatasını kullanmaz:

```python
accepted = (final_progress >= 0.90 * target_distance
            and rmse(depth_error) <= 0.35
            and speed_max <= 2.50
            and navigation_valid_ratio >= 0.95)
```

Tüm senaryolarda **derinlik RMSE 0.79–1.68 m (> 0.35 m eşiği)** olduğundan karar gerçekten "eşik
altı"dır. Bu, **aday politikanın gerçek bir sonucudur** (derinlik takibi eşiği), UKF artefaktı değildir.
Navigasyon altyapısı tüm senaryolarda geçerlidir (`nav_valid_ratio = 1.0`).

## Evidence Files
- [RL_UKF_GT_DIAGNOSIS.md](../diagnostics/rl_ukf/RL_UKF_GT_DIAGNOSIS.md)
- [corrected_rl_ukf_summary_from_raw_telemetry.csv](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv)
- [recomputed_rl_ukf_from_telemetry_verification.csv](../diagnostics/rl_ukf/recomputed_rl_ukf_from_telemetry_verification.csv)
- [metrics_vs_raw_telemetry_ukf_span_check.csv](../diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv)
- [legacy/rl_policy_validation_BUGGY.py](../diagnostics/rl_ukf/legacy/rl_policy_validation_BUGGY.py) · [rl_policy_validation_fixed.py](../diagnostics/rl_ukf/rl_policy_validation_fixed.py)
- [legacy/legacy_rl_metrics_buggy_ukf_rmse.csv](../diagnostics/rl_ukf/legacy/legacy_rl_metrics_buggy_ukf_rmse.csv)

## Limitations
Ham `recording/telemetry.csv` ve `.db3` bag'ler büyüktür ve **repoya dahil edilmez**; yeniden hesap
harici `final_validation/results` klasöründen yapılır. Düzeltilmiş exporter (`rl_policy_validation_fixed.py`)
burada **çalıştırılmamıştır** çünkü ROS 2 + .db3 gerektirir; bunun yerine ROS-bağımsız
`recompute_rl_ukf_from_telemetry.py` ile sonuç bağımsız doğrulanmıştır.
