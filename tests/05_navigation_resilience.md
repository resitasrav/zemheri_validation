# 05 — Navigasyon Dayanıklılığı · `navigation_resilience`

**Durum: ✅ TAMAMLANDI** · Kategori: *Navigasyon dayanıklılığı (resilience)*

## Bu test neyi doğrular?
DVL (hız sensörü) verisi **gecikirse veya kesilirse** navigasyonun çökmediğini,
"korumalı" UKF dalının bozulmaya karşı dayanıklı kaldığını doğrular. Yani gerçek
deniz koşullarında sensör arızası simüle edilir ve navigasyonun ayakta kalıp
kalmadığına bakılır.

## Nasıl kuruldu? (en ayırt edici test)
Bu test, diğerlerinden farklı olarak özel **bozucu (disturbance) düğümler**
çalıştırır:
- `resilience_dvl_delay_node` — DVL teslimini **0.6 s geciktirir** (ölçüm zaman
  damgaları korunur).
- `resilience_dvl_dropout_node` — DVL'i periyodik olarak **keser** (pass 12 s /
  dropout 4 s, harekete göre senkron, tekrarlı).
- Üç paralel UKF örneği: **ham (raw)**, **korumalı (protected)** ve **OOSM**
  (out-of-sequence measurement) dalları.
- `navigation_health_node` durumu raporlar.

Çalışma sırasında loglarda görülen tetiklenmeler (örnek):
`DVL delivery intentionally interrupted` → `DVL delivery restored` döngüleri,
gecikme kuyruğu (`queue`) yönetimi `dropped=0` ile.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Kayıt süresi | 70.4 s |
| Mesaj sayısı | 134.587 |
| Topic sayısı | 31 |
| Kaynak örneği | 1952 |
| ROS log (INFO/WARN) | 73 / 16 |
| Bag boyutu | 24.87 MB |
| Tahmini 1 saatlik bag | ~1272 MB |

## Jüriye not (yorum)
Test, DVL gecikme + kesinti yükü altında **temiz tamamlandı** (`[COMPLETE]`).
16 WARN seviyesi log, kasıtlı kesinti/gecikme olaylarının sayısıyla tutarlı.
Korumalı vs. ham UKF dallarının karşılaştırması `position_error_comparison.png`
ve `protected_navigation_status.png` grafiklerinde sunulur.

## Üretilen çıktılar
- Grafikler: `position_error_comparison.png`, `protected_navigation_status.png`,
  `trajectory_and_depth_comparison.png`, `known_signal_timeseries.png`
- 9 PNG · 13 CSV · 1 rosbag

## Bilinen kapanış anomalisi (engelleyici değil)
Simülasyon kapanışında `guidance_node` SIGINT sırasında segfault (exit −11) verdi.
Bu, test bittikten sonra süreçler kapatılırken oluştu; kayıt ve analiz
tamamlandığı için sonuçları etkilemez. Yine de **kapanış sırasını sağlamlaştırmak**
ileride bir iyileştirme maddesi olarak not edilmelidir.
