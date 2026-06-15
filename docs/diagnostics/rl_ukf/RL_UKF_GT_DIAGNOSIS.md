# RL UKF/GT Teşhis Notu

Bu not, `final_validation(1).zip` içindeki RL episode klasörlerinin **ham `recording/telemetry.csv` kayıtlarından** yeniden hesaplanmıştır.

## Ana bulgu

Önceki README paketindeki RL tarafına **tam güvenmemek gerekir**. Klasörlerdeki `metrics/rl_policy_timeseries.csv` dosyalarında `x_ukf`, `y_ukf`, `z_ukf` değerleri episode boyunca sabit kalmış görünüyor. Buna karşılık aynı episode'un ham `recording/telemetry.csv` kaydında `/odometry/ukf` mesajları episode boyunca normal şekilde ilerliyor.

Bu yüzden README'deki `UKF konum RMSE = 30–46 m` değerleri büyük olasılıkla gerçek UKF çökmesi değil, RL analiz çıktısı üretimindeki eşleştirme/kayıt artefaktıdır.

## Kanıt

`metrics/rl_policy_timeseries.csv` dosyasında UKF x spanı 0 m iken, ham telemetry içinde aynı episode'da UKF x yaklaşık 47–56 m arası ilerliyor. Bu, metrik dosyasının UKF kolonlarını doğru güncellemediğini gösterir.

## Ham telemetry'den yeniden hesaplanan UKF sonuçları

| Senaryo | Metrics UKF RMSE (README'deki) | Ham telemetry raw RMSE | Başlangıç hizalı RMSE |
|---|---:|---:|---:|
| no_current | 30.34 m | 3.86 m | 0.73 m |
| following_current | 36.98 m | 4.13 m | 0.16 m |
| cross_current | 31.91 m | 4.01 m | 0.19 m |
| diagonal_current | 46.03 m | 4.12 m | 0.27 m |
| reverse_current | 32.88 m | 4.00 m | 0.09 m |
| hard_cross_current | 35.09 m | 3.94 m | 0.16 m |

## Yorum

- UKF ham telemetry üzerinde gerçek zamanlı olarak akıyor; RL episode'larında UKF tamamen donmuş değil.
- Başlangıç hizalaması yapıldığında UKF/GT hata seviyesi yaklaşık **0.09–0.73 m** bandına düşüyor.
- Bu seviye diğer navigation/controller testleriyle uyumlu; örneğin navigation straight testinde konum RMSE ~0.82 m, controller tracking testinde ~0.20 m.
- Dolayısıyla RL bölümünde 35 m değerini jüriye doğrudan basmak riskli ve teknik olarak yanıltıcı olur.

## README için önerilen ifade

> RL doğrulama senaryolarında ilk analiz çıktılarında görülen yüksek UKF konum RMSE değerlerinin, ham `/odometry/ukf` kayıtları ile yapılan yeniden hesaplamada analiz/eşleştirme artefaktı olduğu görülmüştür. Ham telemetry üzerinden başlangıç hizalı UKF-GT konum RMSE değerleri 0.09–0.73 m aralığındadır. Bu nedenle RL sonuçları politika başarımı açısından ayrıca değerlendirilmiş; navigasyon altyapısının episode boyunca geçerli kaldığı doğrulanmıştır.

## Üretilen dosyalar

- `corrected_rl_ukf_summary_from_raw_telemetry.csv`
- `metrics_vs_raw_telemetry_ukf_span_check.csv`
- `rl_ukf_raw_vs_aligned_rmse.png`
- `hard_cross_gt_ukf_alignment.png`
