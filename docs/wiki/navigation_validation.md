# Navigation Validation

[← README](../../README.md)

## Table of Contents
- [Purpose](#purpose)
- [Methodology](#methodology)
- [Inputs](#inputs)
- [Execution / Commands](#execution--commands)
- [Logs](#logs)
- [Results](#results)
- [Figures](#figures)
- [Decision](#decision)
- [Evidence Files](#evidence-files)
- [Limitations](#limitations)

## Purpose
Düz hatta UKF konum/derinlik/yaw kestiriminin gerçek harekete (ground truth) tutarlılığını; ve DVL
gecikme/kesintisi altında navigasyon sağlık denetiminin (valid/degraded/failsafe) çökmediğini doğrulamak.

## Methodology
- **Straight Line:** `control_backend:=ros`, warmup 5 s, hedef derinlik 2 m. UKF (`/odometry/ukf`) ile
  GT (`/ground_truth/odometry`), takımın `analyze_report_bag.py` interp-hizalama matematiğiyle karşılaştırılır.
- **Resilience:** DVL bozulması (gecikme + periyodik kesinti) altında üç paralel UKF dalı: **saf**
  robot_localization, **sağlık-denetimli** ve **OOSM**. OOSM kabul ölçütü: RMSE oranı ≤ 1.05 **ve**
  maks-hata oranı ≤ 1.10 (`analyze_navigation_resilience.py`).

## Inputs
Gazebo `buoyant_sara.world`; DVL/IMU/basınç köprüleri; `ukf_node`; `navigation_health_node`. Resilience'da
ek `/validation/resilience/{raw,protected,oosm}_ukf` ve `/validation/resilience/status`.

## Execution / Commands
```bash
python src/validation/run_final_validation.py --cases navigation_straight
python src/validation/run_final_validation.py --cases navigation_resilience
python scripts/generate_validation_figures.py --results <results> --cases navigation_straight navigation_resilience
```

## Logs
Özet metrikler: [navigation_straight/summary.json](../metrics/navigation_straight/summary.json) ·
[navigation_resilience/summary.json](../metrics/navigation_resilience/summary.json).

## Results

**Straight Line (gerçek telemetriden):**
| Metrik | Değer |
|---|---:|
| 3B konum RMSE | 0.217 m |
| Maks. 3B hata | 1.082 m |
| Derinlik RMSE | 0.0012 m |
| Maks. cross-track | 0.158 m |
| Yaw RMSE | 0.011° |
| Örnek / süre | 1379 / 45.9 s |

**Resilience:**
| UKF dalı | 3B konum RMSE | Maks. hata |
|---|---:|---:|
| Saf robot_localization | 1.106 m | 3.65 m |
| Sağlık denetimli | 1.106 m | — |
| OOSM etkin | 1.325 m | 4.51 m |

`navigation_valid` = 1.0 · `degraded` = 0.308 · `failsafe` hiç gerekmedi · OOSM/saf RMSE oranı 1.198.

## Figures

<img src="../figures/navigation/navigation_straight_trajectory_depth.png" width="900">

*Düz hat: GT vs UKF yatay rota (3B RMSE 0.217 m) ve derinlik takibi (RMSE 0.0012 m).*

<img src="../figures/navigation/navigation_straight_error_speed.png" width="900">

*UKF 3B konum hatası ve toplam hız büyüklüğü (GT vs UKF).*

<img src="../figures/navigation/navigation_resilience_position_error.png" width="820">

*Saf / sağlık-denetimli / OOSM UKF dallarının GT'ye göre 3B konum hatası.*

<img src="../figures/navigation/navigation_resilience_status.png" width="820">

*Sağlık durumu: DVL kesintilerinde `degraded`, `navigation_valid` korunuyor, `failsafe` gerekmiyor.*

## Decision
- **Straight Line → PASS** — UKF kestirimi gerçek harekete çok yakın (derinlik RMSE mm seviyesi, cross-track < 0.16 m).
- **Resilience → KISMİ** — sağlık/degraded yönetimi çalışıyor (valid 1.0, failsafe yok); OOSM doğruluk
  kazanımı bu koşumda gösterilemedi (RMSE oranı 1.20 > 1.05) → doğruluk iyileştirmesi olarak raporlanmamalı.

## Evidence Files
- [navigation_straight/summary.csv](../metrics/navigation_straight/summary.csv) ·
  [navigation_resilience/summary.csv](../metrics/navigation_resilience/summary.csv)
- [analyze_report_bag.py](../../src/validation/analyze_report_bag.py) ·
  [analyze_navigation_resilience.py](../../src/validation/analyze_navigation_resilience.py)

## Limitations
Ham per-test rosbag repoda değil (boyut); metrikler ham telemetriden yeniden üretilebilir. OOSM sonucu
yalnız test edilen gecikme/kesinti koşulu için geçerlidir.
