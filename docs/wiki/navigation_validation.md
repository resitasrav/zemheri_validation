# Navigation Validation

[← README](../../README.md)

## İçindekiler
- [Purpose](#purpose)
- [Methodology](#methodology)
- [Inputs](#inputs)
- [Metrics](#metrics)
- [Results](#results)
- [Decision](#decision)
- [Evidence Files](#evidence-files)
- [Limitations](#limitations)

## Purpose
UKF tabanlı durum kestiriminin (a) düz hatta gerçek harekete tutarlılığını, (b) DVL gecikme/kesintisi
altında dayanıklılığını doğrulamak.

## Methodology
- **Straight Line:** `control_backend:=ros`, 50 s, warmup 5 s, hedef derinlik 2 m.
- **Resilience:** ek bozucu düğümler — `resilience_dvl_delay_node` (0.6 s gecikme),
  `resilience_dvl_dropout_node` (pass 12 s / dropout 4 s), üç paralel UKF dalı (ham / korumalı / OOSM).

## Inputs
Gazebo `buoyant_sara.world`, ros_gz köprüleri, `dvl_quality_gate_node`, `ukf_node`
(robot_localization), `navigation_health_node`.

## Metrics
| Metrik | Straight | Resilience |
|---|---:|---:|
| Konum RMSE | 0.824 m | korumalı vs ham karşılaştırması |
| Konum maks. hata | 1.029 m | — |
| Derinlik RMSE | 0.051 m | — |
| Hız RMSE | 0.234 m/s | — |
| Yaw RMSE / maks. | 1.51° / 2.61° | — |
| Maks. cross-track | 0.158 m | — |
| Kayıt süresi | 45.4 s | 70.4 s |
| Mesaj / topic | — | 134.587 / 31 |

## Results
Düz hatta cross-track 16 cm, derinlik hatası 5 cm seviyesinde; rota tutuşu sağlam. Resilience testi
DVL gecikme+kesinti yükü altında `[COMPLETE]` ile temiz tamamlandı (`dropped=0`), 16 WARN logu kasıtlı
kesinti olaylarıyla tutarlı.

## Decision
**PASS** — kontrol zinciri kararlı; navigasyon sensör bozulması altında ayakta kaldı.

## Evidence Files
- [tests/01_navigation_straight.md](../../tests/01_navigation_straight.md)
- [tests/05_navigation_resilience.md](../../tests/05_navigation_resilience.md)
- Örnek figürler: [trajectory_xy](../../figures/ornek_trajectory_xy.png) · [ukf_position_error](../../figures/ornek_ukf_position_error.png)

## Limitations
Ham rosbag/CSV/PNG çıktıları (14 PNG · 10 CSV · 1 rosbag straight; 9 PNG · 13 CSV · 1 rosbag
resilience) boyut nedeniyle bu bundle'da **değildir**; metrikler analiz manifestlerinden taşınmıştır.
Bazı testlerde kapanış (SIGINT) anomalileri vardır; ölçümleri etkilemez.
