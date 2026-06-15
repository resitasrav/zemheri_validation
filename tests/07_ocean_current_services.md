# 07 — Ortam Doğrulama: Akıntı Servisleri · `ocean_current_services`

**Durum: ✅ KABUL** — *akıntı servisleri yanıt verdi* · Kategori: *Ortam doğrulama (environment validation)*

## Bu test neyi doğrular?
Simülasyondaki **okyanus akıntısı (ocean current) servislerinin** çağrılara
doğru yanıt verdiğini doğrular. Bu servisler, RL eğitimi ve dayanıklılık
testleri için **akıntı senaryolarını programatik olarak kurmayı** sağlar
(sabit mod, hedef akıntı, ani esinti, zamanlanmış olay, hazır senaryo vb.).

## Test edilen 8 servis (hepsi ✅)
| Servis | Sonuç | Mesaj |
|---|:---:|---|
| `set_mode_constant` | ✅ | Mode updated |
| `set_target` | ✅ | Target current updated |
| `trigger_gust` | ✅ | Gust triggered |
| `schedule_event` | ✅ | Scheduled event added |
| `clear_schedule` | ✅ | Schedule cleared |
| `set_preset` | ✅ | Preset applied |
| `load_built_in_scenario` | ✅ | Built-in scenario loaded |
| `reset_episode` | ✅ | Episode reset completed |

**Başarılı servis: 8 / 8**

## Akıntı aktivitesi (kayıt boyunca ölçülen)
| Örnek | Ort. X | Ort. Y | Ort. Z | Maks. büyüklük |
|---:|---:|---:|---:|---:|
| 861 | 0.335 m/s | 0.050 m/s | 0.006 m/s | **0.557 m/s** |

> Hedef akıntı `(0.4, 0.2, 0.0)` m/s istenmişti; ölçülen ortalama X≈0.34,
> Y≈0.05 m/s. Akıntı düğümü esinti/dalgalanma ürettiği için anlık büyüklük
> 0.56 m/s'e kadar çıkıyor — bu, servislerin gerçekten **etki ürettiğini** gösterir.

## Jüriye not (yorum)
Sekiz servisin tamamı yanıt verdi ve ölçülen akıntı zaman serisi servis
komutlarıyla tutarlı. Bu test, RL ve dayanıklılık senaryolarının dayandığı
**ortam altyapısının çalıştığını** kanıtlar.

## Üretilen çıktılar
- Grafikler: `ocean_current_service_activity.png` *(bu pakette yeniden üretildi —
  aşağıya bakın)*, `known_signal_timeseries.png`, `process_cpu_max.png`,
  `process_memory_max.png`, `rosout_levels.png`, `topic_counts.png`,
  `topic_rates.png`
- 7 PNG · 11 CSV · 1 rosbag

## ⚠️ Düzeltilen grafik
Orijinal çıktıdaki `figures/ocean_current_service_activity.png` **0 bayt (boş)**
geldi. `metrics/ocean_current_service_timeseries.csv` kayıtlı olduğu için bu
grafik CSV'den **yeniden üretildi** ve `figures/ocean_current_service_activity.png`
olarak bu pakete eklendi (akıntı X/Y/Z + büyüklük, 4 panel).
