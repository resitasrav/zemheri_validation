# 06 — Sensör Sağlığı · `sensor_health`

**Durum: ✅ KABUL** — *sensör veri sürekliliği sağlandı* · Kategori: *Sensör sağlığı*

## Bu test neyi doğrular?
IMU, DVL ve basınç sensörlerinin **kesintisiz ve geçerli** veri akıttığını,
navigasyonun bu veriyle sürekli "geçerli" durumda kaldığını doğrular.

## Nasıl kuruldu?
- Gazebo + `control_backend:=ros`, süre 35 s, warmup 5 s, derinlik 2 m.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Doğrulama kararı | **KABUL** — sensör veri sürekliliği sağlandı |
| `navigation_valid` oranı | **1.0** |
| `imu_ok` oranı | **1.0** |
| `dvl_ok` oranı | **1.0** |
| `pressure_ok` oranı | **1.0** |
| Kayıt süresi | 44.4 s |
| Mesaj sayısı | 81.871 |
| Topic sayısı | 26 |
| ROS log (INFO/WARN) | 52 / 1 |
| Bag boyutu | 12.57 MB |

## Jüriye not (yorum)
Dört sağlık göstergesinin de oranı **1.0** — yani test boyunca hiçbir sensör
kanalında süreklilik kaybı yaşanmadı. Navigasyon %100 "geçerli" durumda kaldı.

## Üretilen çıktılar
- Grafikler: `sensor_dvl_quality.png`, `sensor_dvl_velocity.png`,
  `sensor_imu_acceleration.png`, `sensor_imu_gyro.png`
- 12 PNG · 11 CSV · 1 rosbag

## Bilinen kapanış anomalisi (engelleyici değil)
Simülasyon kapanışında `failsafe_manager_node` SIGINT sırasında bir traceback
verdi. Test/kayıt tamamlandıktan sonraki kapanış anına ait olduğu için
ölçümleri etkilemez.
