# 01 — Navigasyon Performansı (Düz Hat) · `navigation_straight`

**Durum: ✅ TAMAMLANDI** · Kategori: *Navigasyon / kontrol performansı*

## Bu test neyi doğrular?
Araç sabit derinlikte düz bir hat boyunca ilerlerken, **navigasyon kestiriminin
(UKF) ve gerçekleşen hareketin** ne kadar tutarlı olduğunu ölçer. Yani "araç
gitmesi gereken çizgide mi kaldı, kestirim gerçeği ne kadar iyi takip etti?"
sorusunu yanıtlar.

## Nasıl kuruldu?
- Gazebo simülasyonu, `control_backend:=ros` (ROS kontrol zinciri).
- Süre 50 s, 5 s navigasyon filtresi ısınması (warmup), hedef derinlik 2 m.
- Pipeline: Gazebo → DVL/IMU/basınç köprüleri → DVL kalite kapısı → **UKF** →
  navigasyon sağlık düğümü → guidance → setpoint/velocity kontrolcüleri.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Örnek sayısı | 1363 |
| Süre | 45.4 s |
| Konum RMSE | **0.824 m** |
| Konum maks. hata | 1.029 m |
| Derinlik RMSE | **0.051 m** |
| Hız RMSE | 0.234 m/s |
| Roll / Pitch RMSE | 0.36° / 0.45° |
| Yaw RMSE / maks. | 1.51° / 2.61° |
| İz boyu mesafe (along-track) | 32.59 m |
| Maks. yana sapma (cross-track) | **0.158 m** |

## Jüriye not (yorum)
Yana sapmanın 16 cm'de kalması ve derinlik hatasının 5 cm seviyesinde olması,
düz hat görevinde **kontrol zincirinin kararlı** çalıştığını gösterir. Yaw hatası
1.5° gibi düşük bir bandda; rota tutuşu sağlam.

## Üretilen çıktılar
- Grafikler: `depth_tracking.png`, `navigation_error_and_speed.png`,
  `orientation_errors.png`, `trajectory_3d.png`
- 14 PNG · 10 CSV · 1 rosbag

## Bilinen kayıt anomalisi (engelleyici değil)
Runner kayıt dosyasının sonunda, test bittikten **sonra** kapanış aşamasında bir
`RCLError: publisher's context is invalid` izi var. Bu, analiz tamamlandıktan
sonra yayıncı kapanırken oluşan bir temizlik (shutdown) hatasıdır; ölçümleri ve
analiz çıktısını etkilemez (manifest `analysis.status: completed`).
