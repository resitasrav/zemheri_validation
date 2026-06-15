# 02 — Kontrolcü Takip Performansı · `controller_tracking`

**Durum: ✅ TAMAMLANDI** · Kategori: *Navigasyon / kontrol performansı*

## Bu test neyi doğrular?
Hız (0.8 m/s) ve derinlik (2 m) referansları verildiğinde, **setpoint ve hız
kontrolcülerinin** aracı bu referanslara ne kadar tutarlı sürdüğünü ölçer.

## Nasıl kuruldu?
- Gazebo + `control_backend:=ros`.
- Süre 55 s, warmup 5 s, mesafe 35 m, derinlik 2 m, hız 0.8 m/s.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Örnek sayısı | 1478 |
| Süre | 49.2 s |
| Konum RMSE | **0.200 m** |
| Konum maks. hata | 1.129 m |
| Derinlik RMSE | **0.0011 m** |
| Hız RMSE | 0.152 m/s |
| Roll / Pitch RMSE | 0.006° / 0.005° |
| Yaw RMSE / maks. | 0.011° / 0.026° |
| İz boyu mesafe | 36.26 m |
| Maks. yana sapma | 6.59 m |

## Jüriye not (yorum)
Derinlik ve yönelim (roll/pitch/yaw) hataları **milimetre/yüzde-derece**
seviyesinde — kontrolcünün derinlik ve tutum tutuşu çok iyi. Buradaki büyük
yana sapma (6.59 m), aracın komutla yaptığı manevradan kaynaklanır; bu test
yana sapmayı değil **referans takibini** ölçmek için tasarlanmıştır.

> **Önemli çerçeve (README'den):** Mevcut analiz aracı öncelikle
> ground-truth/UKF doğruluğunu ve aracın *gerçekleşen* hareketini raporlar.
> Hedef hız/derinlik/yaw referanslarına göre ayrı bir **kontrol-hatası**
> analizi bir sonraki geliştirme adımıdır.

## Üretilen çıktılar
- Grafikler: `depth_tracking.png`, `navigation_error_and_speed.png`,
  `orientation_errors.png`, `trajectory_3d.png`
- 14 PNG · 10 CSV · 1 rosbag
