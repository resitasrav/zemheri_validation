# Legacy — Buggy RL UKF Metrics (audit only, DO NOT USE)

Bu klasör, RL doğrulamasındaki **hatalı** UKF konum RMSE çıktısının kaynağını ve eski değerlerini
denetim/şeffaflık amacıyla saklar. Bu değerler **jüri raporunda kullanılmaz**; doğru değerler için bkz.
[../corrected_rl_ukf_summary_from_raw_telemetry.csv](../corrected_rl_ukf_summary_from_raw_telemetry.csv)
ve [../../wiki/rl_ukf_diagnosis.md](../../wiki/rl_ukf_diagnosis.md).

## Dosyalar
| Dosya | İçerik |
|---|---|
| `rl_policy_validation_BUGGY.py` | Hatayı üreten orijinal exporter (final_validation/test_codes'tan birebir). |
| `legacy_rl_metrics_buggy_ukf_rmse.csv` | Eski (hatalı) UKF konum RMSE değerleri (30–46 m). |

## Hata (kök neden — koddan doğrulanmış)

`rl_policy_validation_BUGGY.py` içinde:

- **Satır 139:** `ukf = frames["/odometry/ukf"].copy()` — UKF DataFrame'i, başlangıç zamanı
  normalizasyonundan **ÖNCE** kopyalanır.
- **Satır 146–149:** `gt["t"] -= start` ve `for frame in frames.values(): frame["t"] -= start` —
  bu döngü `frames` sözlüğündeki orijinalleri kaydırır, ancak satır 139'da alınan `ukf` **kopyasını
  kaydırmaz.**
- **Satır 152:** `timeline = nearest(gt, ukf, "ukf")` — burada `gt["t"]` [0, T] aralığında iken
  `ukf["t"]` hâlâ mutlak epoch saniyesindedir (~1.78e9). `merge_asof(direction="nearest")` bu yüzden
  **her GT satırını ilk UKF örneğine** eşler → `x_ukf`, `y_ukf`, `z_ukf` episode boyunca **DONAR**.
- Sonuç: `position_error` (satır 185–192) GT'nin sabit bir noktadan uzaklığını ölçer; episode sonunda
  ~görev mesafesi (~50 m) kadar büyür → sahte **~30–46 m UKF RMSE**.

### Gerçek üretilmiş kanıt
`metrics/rl_policy_timeseries.csv` (no_current): `x_ukf` span = **0.0** (ilk değerde donmuş 0.1133),
GT `x` span = 50.11 m, `position_error` 0 → 50.14 m. Ham `recording/telemetry.csv` içinde aynı
episode'da `/odometry/ukf` x değeri 50.36 m boyunca normal ilerler.

## Düzeltme

Bkz. [../rl_policy_validation_fixed.py](../rl_policy_validation_fixed.py) — `ukf` kopyası da `start`
ile normalize edilir (tek satırlık düzeltme), böylece `merge_asof` doğru zaman tabanında çalışır.

## Önemli not (karar mantığı)

Eski per-episode **BAŞARISIZ** kararı (her 6 senaryoda) bu UKF hatasından **bağımsızdır.** Kabul
kriteri (`rl_policy_validation_BUGGY.py:300-305`) UKF konum hatasını **kullanmaz**; derinlik RMSE
≤ 0.35 m, ilerleme ≥ %90 hedef, hız ≤ 2.5 m/s ve nav_valid ≥ 0.95 koşullarına bakar. Tüm senaryolarda
derinlik RMSE 0.79–1.68 m (> 0.35 m) olduğu için karar gerçekten "eşik altı"dır — bu, aday politikanın
gerçek bir sonucudur, UKF artefaktı değildir.
