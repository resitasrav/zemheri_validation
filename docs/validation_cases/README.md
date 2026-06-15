# Validation Case Kanıt Düzeni

[← README](../../README.md)

Bu klasör, `final_validation/results/<test_adi>/figures|metrics` düzenini referans alır; ancak ham kayıtları
taşımaz. Her testin küçük ve işlenebilir kanıtları kendi klasöründe tutulur, böylece figürler ve CSV özetleri
birbirine karışmaz.

## Klasör Mantığı

```text
docs/validation_cases/
  controller_tracking/
    figures/
    metrics/
  navigation_straight/
    figures/
    metrics/
  guidance_los/
    figures/
    metrics/
  guidance_waypoint/
    figures/
    metrics/
  navigation_resilience/
    figures/
    metrics/
  ocean_current_services/
    figures/
    metrics/
  sensor_health/
    figures/
    metrics/
  stage1_fsm/
    figures/
    metrics/
  stage2_bt/
    figures/
    metrics/
  rl_policy/
    figures/
    metrics/
```

## Kaynak Ayrımı

| Kaynak | Kullanım | Repo içine alınan kısım |
|---|---|---|
| `final_validation1.zip` | Ana final validation arşivi | Kısa RL episode summary CSV'leri |
| `final_validation.zip` | Ham final validation arşivi ve önceki paket | Repo içinde yalnız curated türevler |
| `rl.zip` | RL prevalidation ve RL episode karşılaştırma paketi | Küçük CSV/MD özetleri ve Türkçe RL figürleri |
| `algorithm_io_dataflow.md` | Runtime veri akışı sözleşmesi | README/wiki/mimari kapsam notları |

Ham `recording/telemetry.csv`, `.db3` bag, büyük timeseries CSV, raw log dump, zip/rar/bundle ve build/cache
çıktıları burada tutulmaz. Bunlar dış arşivde kalır; repo tarafında sadece jüriye okunabilir kısa kanıtlar
bulunur.

## Case İndeksi

| Case | Figürler | Metrikler | Not |
|---|---|---|---|
| Controller Tracking | [figures](controller_tracking/figures/) | [metrics](controller_tracking/metrics/) | ROS simulation kontrol zinciri kanıtı |
| Navigation Straight | [figures](navigation_straight/figures/) | [metrics](navigation_straight/metrics/) | Düz hat UKF/navigasyon kanıtı |
| Guidance LOS | [figures](guidance_los/figures/) | [metrics](guidance_los/metrics/) | LOS güdüm kanıtı |
| Guidance Waypoint | [figures](guidance_waypoint/figures/) | [metrics](guidance_waypoint/metrics/) | Waypoint güdüm kanıtı |
| Navigation Resilience | [figures](navigation_resilience/figures/) | [metrics](navigation_resilience/metrics/) | DVL/OOSM dayanıklılık kanıtı |
| Ocean Current Services | [figures](ocean_current_services/figures/) | [metrics](ocean_current_services/metrics/) | Akıntı servisleri kanıtı |
| Sensor Health | [figures](sensor_health/figures/) | [metrics](sensor_health/metrics/) | Sensör konu hızı ve sağlık kanıtı |
| Stage 1 FSM | [figures](stage1_fsm/figures/) | [metrics](stage1_fsm/metrics/) | Görev FSM kanıtı |
| Stage 2 BT | [figures](stage2_bt/figures/) | [metrics](stage2_bt/metrics/) | BT/fire-status akış kanıtı |
| RL Policy | [figures](rl_policy/figures/) | [metrics](rl_policy/metrics/) | Prevalidation PASS; final ROS/Gazebo policy WIP/FAIL |

## RL İçin Okuma Notu

`rl.zip` içindeki prevalidation çıktısı kontrollü/basit görevde aday politikanın çalışabildiğini gösterir.
`rl_summary` ve `final_validation1` episode özetleri ise altı akıntı senaryosunun kabul kriterlerini
geçmediğini gösterir. UKF hareketiyle ilgili eski "geçersiz" işareti tek başına final karar kanıtı değildir;
güncel yorum için `rl_policy/metrics/corrected_rl_ukf_summary_from_raw_telemetry.csv` ve
`../diagnostics/rl_ukf/` tanı dosyaları esas alınır.
