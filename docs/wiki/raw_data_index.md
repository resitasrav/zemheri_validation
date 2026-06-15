# Raw Data Index

[← README](../../README.md)

Bu sayfa, depodaki tüm doğrulanabilir veri/artefakt dosyalarını ve **bu bundle'da bulunmayan** ham
kayıtları listeler.

## İçindekiler
- [Bu depoda mevcut veri](#bu-depoda-mevcut-veri)
- [Bu bundle'da olmayan ham kayıtlar](#bu-bundleda-olmayan-ham-kayıtlar)
- [Doğrulanan episode değerleri](#doğrulanan-episode-değerleri)

## Bu depoda mevcut veri

| Dosya | Açıklama | Şema / boyut |
|---|---|---|
| [data/episodes/sara_best_episode.csv](../../data/episodes/sara_best_episode.csv) | En iyi episode tam telemetri (sim) | 34 kolon × 662 satır |
| [docs/diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv) | 6 senaryo düzeltilmiş RL/UKF özeti | 14 kolon × 6 satır |
| [docs/diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv](../diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv) | Donmuş-kolon kanıtı | 5 kolon × 6 satır |
| [docs/architecture/SARA_Sistem_Mimarisi.csv](../architecture/SARA_Sistem_Mimarisi.csv) | 23 mimari düğüm | draw.io CSV |
| [docs/architecture/SARA_Baglanti_Listesi.csv](../architecture/SARA_Baglanti_Listesi.csv) | 28 mimari bağlantı | kaynak→hedef→veri→tip |
| [reports/sara_mission_report.html](../../reports/sara_mission_report.html) | Görev raporu (HTML) | local asset: 2 PNG + 1 MP4 |
| [reports/sara_best_episode.png](../../reports/sara_best_episode.png) | Episode görseli | PNG |
| [reports/sara_episode_summary.png](../../reports/sara_episode_summary.png) | Episode özeti | PNG |
| [reports/sara_mission_video.mp4](../../reports/sara_mission_video.mp4) | Görev videosu | MP4 (~0.12 MB) |
| [figures/ocean_current_service_activity.png](../../figures/ocean_current_service_activity.png) | Akıntı aktivitesi (yeniden üretildi) | PNG, 4 panel |
| [figures/ornek_*.png](../../figures/) | Plot aracının örnek çıktıları | 3 PNG |
| [docs/figures/rl/*.png](../figures/rl/) | Üretilen RL figürleri | 4 PNG |

## sara_best_episode.csv kolon şeması (34)
```
t, step, x, y, z, u, yaw, pitch, ex, ey, ez, eu,
throttle, pitch_fin, yaw_fin, esc_pwm, pitch_pwm, yaw_pwm,
current_n, current_e, current_d, current_mode, energy_wh, reward, done, truncated,
reward_progress, reward_depth, reward_cross, reward_energy, reward_fin, reward_time,
reward_safety, reward_terminal
```

## Bu bundle'da olmayan ham kayıtlar

| Veri | Neden yok | Etkisi |
|---|---|---|
| Per-test rosbag + CSV + PNG (nav/guidance/controller/FSM/BT/sensor) | Boyut | Bu metrikler depodan bağımsız yeniden hesaplanamaz; analiz manifestlerinden taşınmıştır. |
| RL `recording/telemetry.csv` (per episode) | `final_validation` arşivi bundle'da değil | UKF RMSE bağımsız yeniden hesaplanamaz; teşhis paketinin çıktısı kullanılır. |
| RL `metrics/rl_policy_timeseries.csv` | aynı | Donmuş kolon yalnızca span-check üzerinden doğrulanır. |
| RL metrics export scripti | repoda yok | Hatalı kod satırı gösterilemez. |

## Doğrulanan episode değerleri
`python scripts/verify_validation_artifacts.py` ile dosyadan doğrulanmıştır:

| Ölçüt | Dosya değeri |
|---|---:|
| Final x | 50.037 m (≥ 50 ✓) |
| Final derinlik z | 1.984 m (~2 m ✓) |
| Final cross-track y | 0.029 m |
| Energy | 7.274 Wh |
| Toplam reward (Σ) | 932.45 |
| Adım sayısı | 662 |
| done / truncated | True / False ✓ |
| Kütle (sim) | 15.85 kg (`sim/sara_sedaa.py:74`) |
