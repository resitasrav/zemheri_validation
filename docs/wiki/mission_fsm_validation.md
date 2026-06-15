# Mission FSM Validation (Stage 1)

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
Yarışma **Aşama-1** görevini bir sonlu durum makinesi (FSM) ile baştan sona koşturmak ve aracın görev
senaryosunda kararlı ilerlediğini GT/UKF üzerinden doğrulamak.

## Methodology
`control_backend:=ros`, görev süresi 240 s'ye kadar; tam görev düğüm yığını (guidance,
setpoint/velocity kontrolcüleri, failsafe yöneticisi, ocean current, sensör köprüleri, UKF).

## Inputs
`mission_manager_node` (FSM), `guidance_node`, kontrolcüler, `safety_monitor_node`,
`ocean_current_node`, UKF.

## Metrics
| Metrik | Değer |
|---|---:|
| Örnek sayısı | 3226 |
| Süre | 107.5 s |
| Konum RMSE | 1.34 m |
| Derinlik RMSE | 0.075 m |
| Hız RMSE | 0.170 m/s |
| Yaw RMSE / maks. | 3.38° / 6.51° |
| İz boyu mesafe | 73.84 m |
| Maks. cross-track | 32.60 m |

## Results
Aşama-1 görevi 73.8 m'lik iz boyunca kesintisiz yürüdü; derinlik ve yönelim hataları düşük. Büyük
cross-track (32.6 m) görevin çoklu manevra/dönüş geometrisinden kaynaklanır — bir hata değil.

## Decision
**PASS** — görev FSM'i uçtan uca kararlı çalıştı.

## Evidence Files
- [tests/08_stage1_fsm.md](../../tests/08_stage1_fsm.md)

## Limitations
Ham çıktılar (14 PNG · 10 CSV · 1 rosbag) bu bundle'da değildir.
