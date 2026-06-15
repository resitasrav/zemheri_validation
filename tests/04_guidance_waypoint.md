# 04 — Güdüm Doğrulama: Waypoint · `guidance_waypoint`

**Durum: ✅ KABUL** — *waypoint rotası tamamlandı* · Kategori: *Güdüm (guidance) doğrulama*

## Bu test neyi doğrular?
Çoklu hedef noktasından (waypoint) oluşan bir rotanın, güdüm tarafından
**sırayla ve kabul yarıçapı içinde** tamamlanıp tamamlanmadığını doğrular.

## Nasıl kuruldu?
- Güdüm modu: **WAYPOINT**, 4 waypoint.
- Süre 150 s, warmup 5 s, mesafe 60 m, yana sapma 3 m, waypoint kabul 1.5 m,
  derinlik 2 m.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Doğrulama kararı | **KABUL** — waypoint rotası tamamlandı |
| Waypoint sayısı | 4 |
| Yana sapma RMSE | **0.580 m** |
| Maks. yana sapma | 1.18 m |
| Son yana sapma | 1.15 m |
| Heading hata RMSE / maks. | 5.42° / 15.90° |
| Son waypoint mesafesi | 2.54 m |
| Test süresi | 76.3 s |

## Jüriye not (yorum)
Dört waypoint'lik rota boyunca yana sapma RMSE'si 0.58 m'de kalmış; rota düzgün
şekilde tamamlanmış. Heading hatası waypoint geçişlerinde (dönüş anlarında) bir
miktar yükseliyor, bu çoklu-waypoint güdümünde beklenen davranıştır.

## Üretilen çıktılar
- Grafikler: `guidance_command_tracking.png`, `guidance_error_history.png`,
  `guidance_path_tracking.png`, `known_signal_timeseries.png`
- 9 PNG · 10 CSV · 1 rosbag
