# 09 — Yarışma Görevi Aşama 2 (Davranış Ağacı) · `stage2_bt`

**Durum: ✅ TAMAMLANDI** · Kategori: *Yarışma görev koşucusu (competition mission runner)*

## Bu test neyi doğrular?
Yarışmanın **2. aşama görevini**, bir **davranış ağacı (Behavior Tree, BT)** ile
uçtan uca koşturur. Aşama 1'deki FSM'e göre daha modüler bir görev kurgusunun
çalıştığını doğrular.

## Nasıl kuruldu?
- Gazebo + `control_backend:=ros`, görev süresi 100 s'ye kadar.
- Tam görev düğüm yığını (guidance, kontrolcüler, failsafe, sensörler, UKF).

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Örnek sayısı | 1138 |
| Süre | 37.9 s |
| Konum RMSE | 0.722 m |
| Konum maks. hata | 0.988 m |
| Derinlik RMSE | 0.178 m |
| Hız RMSE | 0.191 m/s |
| Roll / Pitch RMSE | 0.51° / **2.58°** |
| Yaw RMSE / maks. | 1.94° / 5.16° |
| Maks. roll / pitch (GT) | 10.05° / **29.09°** |
| İz boyu mesafe | 42.73 m |
| Maks. yana sapma | 0.43 m |

## Jüriye not (yorum)
BT görevi düşük yana sapma (0.43 m) ile tamamlandı. Pitch ekseninde tepe değer
29° gibi belirgin bir manevra var (dalış/çıkış davranışı); pitch RMSE 2.58° ile
diğer testlerden yüksek ama görev senaryosunun gerektirdiği manevrayla tutarlı.
İleride pitch geçişlerinin yumuşatılması bir iyileştirme adayı olabilir.

## Üretilen çıktılar
- Grafikler: `depth_tracking.png`, `navigation_error_and_speed.png`,
  `orientation_errors.png`, `trajectory_3d.png`
- 14 PNG · 10 CSV · 1 rosbag
