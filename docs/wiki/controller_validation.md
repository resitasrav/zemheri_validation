# Controller Validation

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
Hız (0.8 m/s) ve derinlik (2 m) referansları verildiğinde setpoint/velocity kontrolcülerinin takip
başarımını ölçmek.

## Methodology
`control_backend:=ros`, 55 s, warmup 5 s, mesafe 35 m, derinlik 2 m, hedef hız 0.8 m/s.

## Inputs
`control_setpoint_node` (PID setpoint), `velocity_controller`, UKF state.

## Metrics
| Metrik | Değer |
|---|---:|
| Örnek sayısı | 1478 |
| Konum RMSE | 0.200 m |
| Konum maks. hata | 1.129 m |
| Derinlik RMSE | 0.0011 m |
| Hız RMSE | 0.152 m/s |
| Roll / Pitch RMSE | 0.006° / 0.005° |
| Yaw RMSE / maks. | 0.011° / 0.026° |

## Results
Derinlik ve tutum (roll/pitch/yaw) hataları milimetre/yüzde-derece seviyesinde — kontrolcünün derinlik
ve tutum tutuşu çok iyi. Büyük cross-track (6.59 m) komutla yapılan manevradan kaynaklanır; bu test
**referans takibini** ölçer, cross-track'i değil.

## Decision
**PASS** — referans takibi yüksek doğrulukta.

## Evidence Files
- [tests/02_controller_tracking.md](../../tests/02_controller_tracking.md)
- Örnek figür: [depth_speed_tracking](../../figures/ornek_depth_speed_tracking.png)

## Limitations
Hedef hız/derinlik/yaw referansına göre ayrı bir **kontrol-hatası** analizi (mevcut analiz aracı
öncelikle GT/UKF doğruluğunu raporlar) bir sonraki geliştirme adımıdır. Ham çıktılar (14 PNG · 10 CSV ·
1 rosbag) bu bundle'da değildir.
