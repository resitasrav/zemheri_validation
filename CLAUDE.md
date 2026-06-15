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
- **Kök neden bulundu ve düzeltildi (kod seviyesinde, `final_validation` arşivinden doğrulandı).**
- Üretilen gerçek `metrics/rl_policy_timeseries.csv`'de UKF kolonları (`x_ukf/y_ukf/z_ukf`) episode
  boyunca **donmuş (span = 0)**; ham `recording/telemetry.csv`'de `/odometry/ukf` 50–81 m ilerliyor.
- **Hatalı kod:** `rl_policy_validation.py` `build_timeline()` içinde `ukf` DataFrame'i başlangıç-zamanı
  normalizasyonundan önce kopyalanıyor ve normalize edilmiyor → `merge_asof(nearest)` tüm UKF değerlerini
  ilk örneğe sabitliyor → donmuş kolon → sahte 30–46 m RMSE.
- **Düzeltme:** tek satır (`ukf["t"] -= start`) — `docs/diagnostics/rl_ukf/rl_policy_validation_fixed.py`.
  Hatalı orijinal `docs/diagnostics/rl_ukf/legacy/` altında "old artifact" olarak etiketli.
- **Bağımsız doğrulama:** `scripts/recompute_rl_ukf_from_telemetry.py` ham telemetriden yeniden hesapladı;
  başlangıç-hizalı UKF-GT RMSE ≈ **0.09–0.73 m** (teşhis paketiyle eşleşiyor, diğer testlerle tutarlı).
- **Karar üzerine etkisi yok:** accept/reject kriteri (`rl_policy_validation.py:300-305`) UKF konum
  hatasını kullanmaz; derinlik RMSE ≤ 0.35 m vb. bakar. 6 senaryoda da derinlik RMSE 0.79–1.68 m > 0.35
  → aday politika gerçekten eşik altı (UKF artefaktından bağımsız).
- Detay: `docs/wiki/rl_ukf_diagnosis.md`.

## Important Files
- `scripts/verify_validation_artifacts.py` — teslim öncesi tüm tutarlılık kontrolleri.
- `scripts/generate_rl_figures.py` — RL figürleri (trajectory overlay için opsiyonel `--results`).
- `scripts/recompute_rl_ukf_from_telemetry.py` — ROS-bağımsız UKF RMSE yeniden hesabı (harici results).
- `data/episodes/sara_best_episode.csv` — doğrulanan tek-episode telemetrisi.
- `docs/diagnostics/rl_ukf/` — UKF teşhisi: corrected + verification CSV, span-check, fixed exporter,
  `legacy/` (buggy exporter + eski değerler).
- `sim/sara_sedaa.py` — RL/sim ortamı (kütle = 15.8454 kg, satır 74).

> **Kaynak veri:** `final_validation.zip` (235 MB, repo dışı) — 6 RL episode'unun ham
> `recording/telemetry.csv` + `metrics/` + test kodları. Repoya YALNIZCA küçük curated özetler alındı;
> ham `.db3`/büyük telemetry `.gitignore` ile dışarıda.

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

## Commit Notes
- Bu repo **jüri incelemesi** için hazırlanmıştır; RL sonuçları **policy candidate validation**
  olarak sunulur (eğitilmiş SAC değil).
- Branch: `final-sara-wiki-rl-diagnosis`. **Main'e dokunulmadı.**
- **Push YAPILMADI ve YAPILMAYACAK — kullanıcı kendisi pushlayacaktır.** Remote değiştirilmedi.
- Push öncesi `scripts/verify_validation_artifacts.py` çıkışı **0** olmalı.
- `.gitignore` ham/büyük/arşiv dosyaları (`*.zip`, `*.rar`, `*.bundle`, `*.db3`, `recording/`,
  `build/`, `install/`, `log/`) repo dışında tutar. `reports/sara_mission_video.mp4` ≈ 0.12 MB
  curated istisnadır (`!reports/*.mp4`).
