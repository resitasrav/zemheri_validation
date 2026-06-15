# CLAUDE.md — SARA Validation Repository Context

Bu dosya, Claude'un (veya başka bir geliştiricinin) bu depoyu tekrar açtığında bağlamı hızlıca
anlaması için hazırlanmıştır. Wiki formatındadır.

## Current Project State
- **Repo amacı:** SARA otonom sualtı aracı yazılım zincirinin (sensör → UKF → guidance → kontrolcü →
  görev FSM/BT) Gazebo Harmonic'te doğrulanmasını **belgelemek**. Bu bir dokümantasyon/teşhis/figür
  paketidir; ROS 2 paket kaynak kodunun tamamı bu depoda **yoktur**.
- **Hedef GitHub deposu başta boştu** (`MelikeBeyazli/zemheri_validation`); içerik yerel artefaktlardan
  inşa edildi.
- Aktif branch: `final-sara-wiki-rl-diagnosis`. Main'e doğrudan push yapılmamalıdır.

## Repository Structure
```
tests/         10 test analiz sayfası (01..10)
figures/       yeniden üretilen + örnek doğrulama figürleri (ornek_*, ocean_current_*)
rl_tools/      plot_validation_figures.py (genel figür üreticisi) + README_RL.md
docs/architecture/      SARA mimari (png/pdf/drawio + 2 CSV)
docs/diagnostics/rl_ukf/ RL UKF/GT teşhis paketi (MD + 2 CSV + 2 PNG)
docs/figures/rl/        üretilen RL figürleri (4 PNG)
docs/wiki/              9 wiki sayfası
data/episodes/          sara_best_episode.csv (.xls'ten dönüştürüldü; gerçekte CSV)
reports/                HTML rapor, episode görselleri, görev videosu
sim/                    sara_sedaa.py (RL/sim ortamı)
scripts/                verify_validation_artifacts.py + generate_rl_figures.py
```

## System Architecture Summary
ROS 2 seyir/görev zinciri: `dvl_quality_gate_node → ukf_node → navigation_health_node →
auv_state_publisher → mission_manager_node (FSM + Aşama-2 BT) → guidance_node → control_setpoint_node →
mavlink_bridge_node → ArduPilot → PWM/Servo`. Failsafe: ROS tarafında `safety_monitor_node`
(heartbeat/AI-disable/fire-inhibit), Pixhawk tarafında bağımsız hold + `reconnect_sync` ile görev
kurtarma. Makine-okunur kaynak: `docs/architecture/SARA_Sistem_Mimarisi.csv` (23 düğüm) +
`SARA_Baglanti_Listesi.csv` (28 bağlantı) — **tutarlı** (tüm bağlantı uçları düğüm listesinde).

## Validation Layers
| Katman | Test | Durum |
|---|---|---|
| Navigation | Straight / Resilience | PASS / PASS |
| Guidance | LOS / Waypoint | PASS / PASS |
| Controller | Tracking | PASS |
| Sensor | Health | PASS |
| FSM | Stage 1 | PASS |
| BT | Stage 2 mission / Fire decision | PASS / **Needs Evidence** |
| Ocean Current | Services (8/8) | PASS |
| RL | Policy candidate | **WIP** |

> **Önemli:** Navigation/Guidance/Controller/FSM/BT/Sensor metrikleri test analiz manifestlerinden
> **taşınmıştır**; ham rosbag/CSV/PNG bu bundle'da yok → bağımsız yeniden hesaplanamaz.

## RL / Policy Candidate Validation
- Bu çalışma **eğitilmiş bir SAC ajanı DEĞİLDİR.** Checkpoint, öğrenme eğrisi, değerlendirme protokolü,
  seed/hiperparametre kaydı yok → **policy candidate** olarak etiketlenir.
- 6 akıntı senaryosu; zincir hepsinde geçerli (`nav_valid_ratio = 1.0`). `reverse_current` progress
  47.68 m (<50 m hedef) ve `hard_cross_current` cross-track RMSE 4.79 m olduğundan genel durum **WIP**.

## UKF-RL Diagnosis Summary
- Eski RL metrik CSV'lerinde (`metrics/rl_policy_timeseries.csv`) UKF kolonları (`x_ukf`,`y_ukf`,`z_ukf`)
  episode boyunca **sabit (span ≈ 0)** kalmış olabilir.
- Ham `recording/telemetry.csv` içinde `/odometry/ukf` **normal ilerliyor** (x span ≈ 50–81 m).
- Bu yüzden eski **30–46 m UKF RMSE** değerleri doğrudan UKF çökmesi olarak yorumlanmamalıdır; bu bir
  **analiz/export artefaktıdır.**
- Ham telemetriden yeniden hesaplanan **başlangıç-hizalı** UKF-GT RMSE ≈ **0.09–0.73 m** bandında
  (diğer testlerle tutarlı).
- **Doğrulama durumu:** Semptom ve kök-neden sınıfı `metrics_vs_raw_telemetry_ukf_span_check.csv`
  üzerinden depo içinde doğrulandı. Ham telemetry ve üretici script depoda **olmadığından** hatalı kod
  satırı gösterilemiyor; README'ye yalnızca doğrulanmış sonuç yazıldı. Detay:
  `docs/wiki/rl_ukf_diagnosis.md`.

## Important Files
- `scripts/verify_validation_artifacts.py` — teslim öncesi tüm tutarlılık kontrolleri (30 kontrol).
- `scripts/generate_rl_figures.py` — RL figürlerini düzeltilmiş özet CSV'den üretir.
- `data/episodes/sara_best_episode.csv` — doğrulanan tek-episode telemetrisi.
- `docs/diagnostics/rl_ukf/` — UKF teşhisinin tüm kanıtı.
- `sim/sara_sedaa.py` — RL/sim ortamı (kütle = 15.8454 kg, satır 74).

## How to Reproduce
```bash
python scripts/verify_validation_artifacts.py     # tutarlılık denetimi (çıkış 0 = geçti)
python scripts/generate_rl_figures.py             # RL figürlerini yeniden üret
python sim/sara_sedaa.py                           # RL/sim deneyleri (seed=42, 16 episode)
```

## Verification Checklist
- [x] README ve wiki linkleri kırık değil (verify script kontrol ediyor).
- [x] sara_best_episode.csv 34 kolon, done=True, truncated=False, x≥50, z~2, Σreward≈932.4.
- [x] Diagnosis span-check tutarlı (metrics span≈0 vs raw span≈50–81 m).
- [x] Aligned UKF RMSE 0.09–0.73 m bandında.
- [x] 4 RL figürü + 2 teşhis PNG'si mevcut.
- [x] Mimari CSV tutarlı (23 düğüm / 28 bağlantı, yetim referans yok).
- [x] HTML raporun local assetleri mevcut.
- [x] Notebook geçerli JSON.

## Known Metrics (dosyadan doğrulanmış)
| Ölçüt | Değer | Kaynak |
|---|---:|---|
| Final forward x | 50.037 m | sara_best_episode.csv |
| Final depth z | 1.984 m | sara_best_episode.csv |
| Final cross-track y | 0.029 m | sara_best_episode.csv |
| Energy | 7.274 Wh | sara_best_episode.csv |
| Toplam reward | 932.45 | sara_best_episode.csv (Σ) |
| Adım | 662 | sara_best_episode.csv |
| Kütle | 15.85 kg | sara_sedaa.py:74 |
| Aligned UKF RMSE | 0.09–0.73 m | corrected_rl_ukf_summary CSV |

## Known Limitations
- Ham per-test kayıtları yok → o metrikler bağımsız doğrulanamaz.
- RL metrics üretici script yok → hatalı kod satırı gösterilemez.
- RL = policy candidate, trained SAC değil.
- Fire decision logic izole test edilmedi.
- ArduPilot kontrol arka ucu performans doğrulamasına dahil değil.

## Commit and Push Notes
- Branch: `final-sara-wiki-rl-diagnosis`. **Main'e doğrudan push YOK.**
- Push öncesi `scripts/verify_validation_artifacts.py` çıkışı **0** olmalı; değilse push etme.
- Büyük dosya: `reports/sara_mission_video.mp4` ≈ 0.12 MB (GitHub limitinin çok altında, LFS gerekmez).
- Yeni `final_validation` arşivi veya RL üretici script eklenirse: ham telemetriden bağımsız
  yeniden hesaplama yap, hatalı satırı düzelt, eski metrikleri `legacy/` altında etiketle.
