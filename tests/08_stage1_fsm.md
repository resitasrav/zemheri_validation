# 08 — Yarışma Görevi Aşama 1 (FSM) · `stage1_fsm`

**Durum: ✅ TAMAMLANDI** · Kategori: *Yarışma görev koşucusu (competition mission runner)*

## Bu test neyi doğrular?
Yarışmanın **1. aşama görevini**, bir **sonlu durum makinesi (FSM)** ile baştan
sona koşturur ve aracın bu görev senaryosunda kararlı şekilde ilerlediğini
ground-truth/UKF üzerinden doğrular. Bu, "uçtan uca görev çalışıyor mu?"
sorusunun yarışma senaryosundaki cevabıdır.

## Nasıl kuruldu?
- Gazebo + `control_backend:=ros`, görev süresi 240 s'ye kadar.
- Tam görev düğüm yığını: guidance, setpoint/velocity kontrolcüleri,
  failsafe yöneticisi, ocean current, sensör köprüleri, UKF.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Örnek sayısı | 3226 |
| Süre | 107.5 s |
| Konum RMSE | 1.34 m |
| Konum maks. hata | 1.64 m |
| Derinlik RMSE | 0.075 m |
| Hız RMSE | 0.170 m/s |
| Roll / Pitch RMSE | 0.47° / 0.87° |
| Yaw RMSE / maks. | 3.38° / 6.51° |
| Maks. roll / pitch (GT) | 10.38° / 9.73° |
| İz boyu mesafe | 73.84 m |
| Maks. yana sapma | 32.60 m |

## Jüriye not (yorum)
Aşama 1 görevi 73.8 m'lik bir iz boyunca koşturuldu; derinlik ve yönelim
hataları düşük kaldı. Buradaki büyük yana sapma (32.6 m), görevin **çoklu
manevra/dönüş** içermesinden kaynaklanır — bu bir hata değil, görev rotasının
geometrisidir. Görev FSM'i baştan sona kesintisiz yürüdü.

## Üretilen çıktılar
- Grafikler: `depth_tracking.png`, `navigation_error_and_speed.png`,
  `orientation_errors.png`, `trajectory_3d.png`
- 14 PNG · 10 CSV · 1 rosbag
