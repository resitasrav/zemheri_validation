# `src/validation/` — Takımın Gerçek Doğrulama Kodu

Bu klasördeki dosyalar **takımın kendi yazdığı** ROS 2 doğrulama/analiz
zincirinden gelir (`final_validation.zip` arşivi). Jüri "hangi kodu çalıştırıyoruz,
testler nasıl işliyor" sorusunu burada görür. Bu dosyalar **Claude tarafından
üretilmemiştir**; oldukları gibi (provenance korunarak) eklenmiştir.

> Bu kodlar ROS 2 / Gazebo ortamında (`rosbag2_py`, `rclpy`, `zemheri_interfaces`)
> çalışır. Repo, jüri incelemesi için kanıt + kod paketidir; tam ROS 2 çalışma
> alanı (`zemheri_ws`) ayrı tutulur.

## Test çalıştırıcılar (orchestration)

| Dosya | Görev |
|---|---|
| `run_final_validation.py` | Tüm final testleri Gazebo'da izole oturumlarla sırayla koşturur; her test için ayrı rosbag kaydı alır, analizi çalıştırır, tek bir indeks üretir. Test parametreleri (mesafe, süre, akıntı) burada tanımlıdır. |
| `report_test_runner.py` | Tek bir rapor-doğrulama senaryosunu (`--case ...`) ROS 2 üzerinde koşturur ve verisini kaydeder. |
| `run_algorithm_validation_suite.py` | Algoritma doğrulama senaryolarını topluca çalıştırıp artefaktları özetler. |
| `competition_mission_runner.py` | Aşama-1/Aşama-2 yarışma görev akışını (FSM + BT) sürücü olarak yürütür. |

## Analiz (rosbag → metrik + figür)

| Dosya | Üretilen test kanıtı | Kabul ölçütü (kod içinde) |
|---|---|---|
| `analyze_report_bag.py` | navigation_straight, controller_tracking, stage1_fsm, stage2_bt için GT↔UKF doğruluk figürleri ve `summary.csv` | konum/derinlik/hız/yaw RMSE; stage1 bitiş çizgisi (`along≈10 m`, `cross≤3 m`); stage2 dalış (`pitch≤−30°`) |
| `analyze_guidance_validation.py` | LOS / Waypoint cross-track, heading, rota figürleri | LOS: son yanal hata < başlangıç; Waypoint: son mesafe ≤ kabul yarıçapı (1.5 m) |
| `analyze_navigation_resilience.py` | Saf / sağlık-denetimli / OOSM UKF karşılaştırması + sağlık durum çizelgesi | OOSM: RMSE oranı ≤ 1.05 **ve** maks-hata oranı ≤ 1.10 |
| `analyze_environment_validation.py` | sensor_health (yayın frekansları) ve ocean_current servis figürleri | tüm topic'ler alt-frekans sınırının üstünde; sağlık oranları ≥ 0.95 |
| `rl_policy_validation.py` | RL **policy candidate** episode metrikleri (gerçek Gazebo/UKF zinciri) | derinlik RMSE ≤ 0.35 m vb. — **NOT:** bu dosya UKF zaman-tabanı (`merge_asof`) hatasını içerir; bkz. [RL UKF teşhisi](../../docs/diagnostics/rl_ukf/RL_UKF_GT_DIAGNOSIS.md) |

## Önemli notlar

- **`rl_policy_validation.py` içindeki hata:** `build_timeline()` fonksiyonunda UKF
  DataFrame'i başlangıç-zamanı normalizasyonundan önce kopyalanıp normalize
  edilmediği için `merge_asof(nearest)` tüm UKF örneklerini ilk değere sabitler
  (donmuş kolon) → sahte ~30–46 m UKF RMSE. Düzeltilmiş sürüm ve kök-neden analizi
  `docs/diagnostics/rl_ukf/` altındadır. Bu hata kabul/ret kararını **değiştirmez**
  (karar derinlik RMSE'ye bakar).
- Bu klasördeki analiz kodları **rosbag (.db3)** okur. Jüri belgelerindeki figürler,
  aynı matematik ile ham `recording/telemetry.csv` dışa-aktarımlarından
  `scripts/generate_validation_figures.py` (Claude-üretimi yardımcı) tarafından
  yeniden üretilmiştir; ham telemetry (~235 MB) repoya dahil değildir.
