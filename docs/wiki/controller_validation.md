# Controller Validation

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
Hedef hız (0.8 m/s) ve hedef derinlik (2 m) altında kontrol zincirinin gerçekleşen hareketini, derinlik
tutumunu ve UKF kestirim tutarlılığını ölçmek.

## Methodology
`control_backend:=ros`, 35 m düz seyir, 2 m hedef derinlik ve 0.8 m/s hedef hız kullanıldı. Analiz,
takımın [analyze_report_bag.py](../../src/validation/analyze_report_bag.py) koduyla GT ve UKF zaman
serilerini hizalayarak konum, derinlik, hız ve yaw metriklerini çıkardı.

Bu sayfa **validation/simulation** kapsamını raporlar: `guidance_node` çıktısı ROS sim kontrol zincirinde
`/sara_uuv/cmd_vel` ve sim aktüatör topic'lerine gider. Runtime/gerçek araç zinciri
`control_setpoint_bridge_node → /control/setpoint → Pixhawk/ArduPilot` olarak ayrıdır ve bu koşumda
doğrudan performans kanıtı olarak kullanılmamıştır.

## Inputs
`control_setpoint_node` / `setpoint_controller`, `velocity_controller`, `guidance_node`, UKF
(`/odometry/ukf`), GT odometri ve final_validation gerçek telemetri kaydı.

## Execution / Commands
```bash
python src/validation/run_final_validation.py --cases controller_tracking
python scripts/generate_validation_figures.py --results <final_validation/results> --cases controller_tracking
```

## Logs
Özet metrik dosyaları:
[summary.csv](../metrics/controller_tracking/summary.csv) ·
[summary.json](../metrics/controller_tracking/summary.json).

## Results
| Metrik | Değer |
|---|---:|
| Örnek sayısı | 1509 |
| Süre | 50.265 s |
| 3B konum RMSE | 0.201 m |
| Maks. 3B hata | 1.134 m |
| Derinlik RMSE | 0.0012 m |
| Hız RMSE | 0.150 m/s |
| Yaw RMSE / maks. | 0.022° / 0.048° |
| İz boyu mesafe | 36.260 m |
| Maks. cross-track | 6.587 m |

Derinlik ve yaw kestirim hataları çok düşüktür. Maksimum cross-track değeri bu koşumda yapılan manevradan
gelir; bu sayfa yanal rota tutma başarısını değil, kontrol zinciri ve UKF/GT tutarlılığını raporlar.

## Figures
<img src="../figures/controller/controller_tracking_trajectory_depth.png" width="900">

*Controller tracking: GT ve UKF yatay rota karşılaştırması ile derinlik takibi.*

<img src="../figures/controller/controller_tracking_error_speed.png" width="900">

*Controller tracking: UKF 3B konum hatası ve toplam hız büyüklüğü; hedef seyir hızı korunuyor.*

## Decision
**PASS** — Derinlik RMSE 0.0012 m, yaw RMSE 0.022° ve hız RMSE 0.150 m/s seviyesinde kaldı. Test,
ArduPilot/MAVLink arka ucu yerine ROS kontrol arka ucunu doğrular.

## Evidence Files
- [docs/metrics/controller_tracking/summary.csv](../metrics/controller_tracking/summary.csv)
- [docs/figures/controller/](../figures/controller/)
- [src/validation/analyze_report_bag.py](../../src/validation/analyze_report_bag.py)
- [src/validation/report_test_runner.py](../../src/validation/report_test_runner.py)

## Limitations
Mevcut özet, actuator saturasyonu veya ayrı PID iç hata kanallarını raporlamaz; final_validation içinde bu
sayfada sunulabilecek izole actuator-range kanıtı yoktur. Ham rosbag/telemetry dosyaları repoya dahil
edilmez. `/control/setpoint` ve Pixhawk/ArduPilot gerçek araç performansı için bu pakette doğrudan kanıt
yoktur.
