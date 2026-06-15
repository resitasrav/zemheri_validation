# RL Policy Validation

[← README](../../README.md)

> **This section validates a policy candidate / RL-style control candidate. It should not be presented as a fully trained SAC agent unless training checkpoints, training curves and the evaluation protocol are included.**
>
> Bu bölüm, eğitilmiş SAC ajanı iddiası değil; farklı akıntı senaryolarında test edilmiş bir **policy
> candidate** değerlendirmesidir.

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
Seçilen politika adayının tam ROS/Gazebo zincirinde (UKF + guidance + kontrolcü) altı akıntı senaryosunda
ilerleme, yanal hata, derinlik takibi ve navigation-valid durumunu ölçmek.

## Methodology
Altı episode matrisi gerçek final_validation kayıtlarından analiz edildi. UKF-GT konum hatası ham
`recording/telemetry.csv` üzerinden hem **raw** hem de **başlangıç-hizalı (aligned)** olarak bağımsız yeniden
hesaplandı. Kabul kararı, takımın [rl_policy_validation.py](../../src/validation/rl_policy_validation.py)
mantığındaki progress, depth RMSE, hız ve nav_valid eşiklerine göre yorumlandı.

Ek kaynak ayrımı:

- `rl.zip / rl_prevalidation`: kontrollü/basit görevde aday politikanın çalışabildiğini gösteren prevalidation
  çıktısıdır; bu bölüm tek başına final ROS/Gazebo kabul kanıtı değildir.
- `rl.zip / rl_summary` ve `final_validation1.zip`: altı akıntı senaryosundaki final ROS/Gazebo policy
  karşılaştırmasını verir. Bu karşılaştırmada kabul edilen episode yoktur.
- `docs/diagnostics/rl_ukf/`: eski UKF exporter artefaktını açıklayan ve ham telemetriden düzeltilmiş UKF
  karşılaştırmasını veren ana tanı paketidir.

## Inputs
- [data/episodes/sara_best_episode.csv](../../data/episodes/sara_best_episode.csv) — calm episode özeti, 34 kolon, 662 adım.
- [validation_cases/rl_policy](../validation_cases/rl_policy/) — `rl.zip`, `final_validation1.zip` ve düzeltilmiş UKF tanısının küçük kanıt düzeni.
- [rl_zip_episode_comparison.csv](../validation_cases/rl_policy/metrics/rl_zip_episode_comparison.csv)
- [corrected_rl_ukf_summary_from_raw_telemetry.csv](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv)
- [metrics_vs_raw_telemetry_ukf_span_check.csv](../diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv)
- [src/validation/rl_policy_validation.py](../../src/validation/rl_policy_validation.py)

## Execution / Commands
```bash
python scripts/recompute_rl_ukf_from_telemetry.py <final_validation/results> --out out.csv
python scripts/generate_rl_figures.py --results <final_validation/results>
```

## Logs
Düzeltilmiş RL/UKF özetleri:
[corrected summary](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv) ·
[span check](../diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv).

## Results
### Prevalidation (`rl.zip`)

`rl_prevalidation/sara_episode_summary.csv` kontrollü/basit 50 m görevde 16 episode'un tamamını `success=True`
olarak raporlar. En iyi kayıt episode 12'dir: `50.04 m` ileri ilerleme, `1.98 m` derinlik, `0.03 m`
cross-track ve `932.4` reward. Bu sonuç aday politikanın basit görevde çalışabildiğini gösterir; final
akıntı senaryolarındaki kabul kararının yerine geçmez.

### Final ROS/Gazebo Akıntı Senaryoları

| Senaryo | Progress | Cross-track RMSE | Depth RMSE | Raw UKF RMSE | Aligned UKF RMSE | max speed | nav_valid |
|---|---:|---:|---:|---:|---:|---:|---:|
| no_current | 50.12 m | 0.54 m | 1.10 m | 3.86 m | 0.73 m | 1.03 m/s | 1.0 |
| following_current | 56.82 m | 0.42 m | 1.09 m | 4.13 m | 0.16 m | 1.28 m/s | 1.0 |
| cross_current | 53.77 m | 1.27 m | 1.21 m | 4.01 m | 0.19 m | 0.98 m/s | 1.0 |
| diagonal_current | 81.55 m | 0.81 m | 0.79 m | 4.12 m | 0.27 m | 1.25 m/s | 1.0 |
| reverse_current | 47.68 m | 0.34 m | 1.48 m | 4.00 m | 0.09 m | 0.83 m/s | 1.0 |
| hard_cross_current | 58.07 m | 4.79 m | 1.68 m | 3.94 m | 0.16 m | 0.95 m/s | 1.0 |

Kabul eşiği içinde kritik başarısızlık derinlik takibidir: tüm senaryolarda depth RMSE 0.79-1.68 m
aralığında ve `0.35 m` eşiğinin üzerindedir. `reverse_current` ayrıca 50 m ilerleme altında kalır;
`hard_cross_current` ise 4.79 m cross-track RMSE ile yanal hata açısından zayıftır.

## Figures
<img src="../validation_cases/rl_policy/figures/rl_zip_episode_performance_bars.png" width="820">

*`rl.zip` kaynaklı Türkçe performans paneli: 6 final episode'un puan, cross-track/depth RMSE ve ilerleme
oranı özeti. Bu panelde de kabul edilen episode yoktur.*

<img src="../figures/rl/rl_episode_comparison_matrix.png" width="900">

*Düzeltilmiş RL/policy candidate episode matrisi: başlangıç-hizalı UKF RMSE ve derinlik RMSE birlikte.*

<img src="../figures/rl/rl_current_robustness.png" width="700">

*Akıntı senaryosu sıralamasına göre robustness görünümü; bu grafik policy candidate değerlendirmesidir.*

<img src="../figures/rl/rl_trajectory_overlay.png" width="780">

*GT ve UKF rota overlay'i; politika izinin gerçek hareket karşılığı ground-truth rotadır.*

<img src="../figures/rl/rl_ukf_raw_vs_aligned_rmse.png" width="780">

*Raw ve başlangıç-hizalı UKF RMSE karşılaştırması; eski 30-46 m metrik artefaktı kullanılmaz.*

## Decision
**WIP** — Zincir altı senaryoda da çalıştı ve `nav_valid_ratio=1.0` kaldı. Ancak aday politika hiçbir
senaryoda derinlik RMSE kabul eşiğini sağlamadı; bu nedenle "trained SAC agent PASS" gibi sunulmamalıdır.

## Evidence Files
- [docs/validation_cases/rl_policy/](../validation_cases/rl_policy/)
- [docs/diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv](../diagnostics/rl_ukf/corrected_rl_ukf_summary_from_raw_telemetry.csv)
- [docs/diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv](../diagnostics/rl_ukf/metrics_vs_raw_telemetry_ukf_span_check.csv)
- [docs/figures/rl/](../figures/rl/)
- [scripts/recompute_rl_ukf_from_telemetry.py](../../scripts/recompute_rl_ukf_from_telemetry.py)
- [scripts/generate_rl_figures.py](../../scripts/generate_rl_figures.py)
- [src/validation/rl_policy_validation.py](../../src/validation/rl_policy_validation.py)
- [RL UKF Diagnosis](rl_ukf_diagnosis.md)

## Limitations
Eğitim checkpoint'i, öğrenme eğrisi, seed/env config ve resmi değerlendirme protokolü bu depoda yoktur.
Per-episode ham telemetry büyük olduğu için repoya alınmaz; sunulan metrikler curated diagnosis ve yeniden
hesap CSV'lerinden gelir.
