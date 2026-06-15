# Ocean Current Validation

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
RL ve dayanıklılık senaryolarının dayandığı okyanus akıntısı servislerinin çağrılara doğru yanıt
verdiğini ve gerçek etki ürettiğini doğrulamak.

## Methodology
8 servise programatik çağrı; servis dönüşleri ile ölçülen akıntı zaman serisinin karşılaştırılması.

## Inputs
`ocean_current_node`, `metrics/ocean_current_service_timeseries.csv`.

## Metrics
| Servis | Sonuç |
|---|:---:|
| set_mode_constant | ✅ |
| set_target | ✅ |
| trigger_gust | ✅ |
| schedule_event | ✅ |
| clear_schedule | ✅ |
| set_preset | ✅ |
| load_built_in_scenario | ✅ |
| reset_episode | ✅ |

**8 / 8** · Hedef akıntı `(0.4, 0.2, 0.0)` m/s; ölçülen ort. X≈0.34, Y≈0.05, Z≈0.006 m/s; anlık maks.
büyüklük **0.557 m/s**.

## Results
Sekiz servisin tamamı yanıt verdi; ölçülen akıntı zaman serisi servis komutlarıyla tutarlı. Akıntı
düğümü esinti/dalgalanma ürettiği için anlık büyüklük 0.56 m/s'e kadar çıkıyor.

<img src="../../figures/ocean_current_service_activity.png" width="780">

## Decision
**PASS** — ortam akıntı altyapısı çalışıyor (RL/resilience senaryolarının temeli).

## Evidence Files
- [tests/07_ocean_current_services.md](../../tests/07_ocean_current_services.md)
- [figures/ocean_current_service_activity.png](../../figures/ocean_current_service_activity.png) — 0 baytlık
  orijinalin yerine kayıtlı CSV'den **yeniden üretildi** (4 panel: X/Y/Z + büyüklük).

## Limitations
Diğer ham çıktılar (7 PNG · 11 CSV · 1 rosbag) bu bundle'da değildir.
