# Ocean Current Validation

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
RL ve navigasyon dayanıklılığı senaryolarının dayandığı okyanus akıntısı servislerinin deterministik biçimde
akıntı vektörü yayınladığını doğrulamak.

## Methodology
Okyanus akıntısı servisleri programatik olarak çağrıldı; `/ocean_current` zaman serisi kaydedildi ve
[analyze_environment_validation.py](../../src/validation/analyze_environment_validation.py) ile özetlendi.
Bu sayfa servis yayınını doğrular; RL sayfasındaki senaryo karşılaştırmalarıyla karıştırılmamalıdır.

## Inputs
`ocean_current_node`, `/ocean_current` telemetrisi ve final_validation gerçek kayıtlarından üretilen
`ocean_current_services` summary dosyaları.

## Execution / Commands
```bash
python src/validation/run_final_validation.py --cases ocean_current_services
python scripts/generate_validation_figures.py --results <final_validation/results> --cases ocean_current_services
```

## Logs
Özet metrik dosyaları:
[summary.csv](../metrics/ocean_current_services/summary.csv) ·
[summary.json](../metrics/ocean_current_services/summary.json).

## Results
| Metrik | Değer |
|---|---:|
| Örnek sayısı | 871 |
| Ortalama X | 0.336 m/s |
| Ortalama Y | 0.050 m/s |
| Ortalama Z | 0.006 m/s |
| Maks. büyüklük | 0.557 m/s |
| Karar | KABUL |

Ölçülen akıntı vektörü servis komutlarıyla tutarlı yayın yaptı. Bu test RL politika kabul kararı vermez;
yalnızca akıntı altyapısının gerçek koşumda veri ürettiğini kanıtlar.

## Figures
<img src="../figures/ocean_current/ocean_current_service_activity.png" width="820">

*Ocean current service: X/Y/Z bileşenleri ve toplam akıntı büyüklüğü zaman serisi.*

## Decision
**PASS** — Akıntı servisi gerçek telemetry koşumunda deterministik biçimde yayın yaptı; maksimum büyüklük
0.557 m/s olarak ölçüldü.

## Evidence Files
- [docs/metrics/ocean_current_services/summary.csv](../metrics/ocean_current_services/summary.csv)
- [docs/figures/ocean_current/ocean_current_service_activity.png](../figures/ocean_current/ocean_current_service_activity.png)
- [src/validation/analyze_environment_validation.py](../../src/validation/analyze_environment_validation.py)
- [src/validation/report_test_runner.py](../../src/validation/report_test_runner.py)

## Limitations
Bu sayfa akıntı servis yayınına odaklanır. `no_current`, `following_current`, `cross_current`,
`diagonal_current`, `reverse_current` ve `hard_cross_current` politika performansı
[RL Policy Validation](rl_policy_validation.md) sayfasında WIP olarak raporlanır.
