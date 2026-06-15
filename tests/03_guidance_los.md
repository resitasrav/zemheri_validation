# 03 — Güdüm Doğrulama: LOS · `guidance_los`

**Durum: ✅ KABUL** — *LOS rota eksenine yakınsadı* · Kategori: *Güdüm (guidance) doğrulama*

## Bu test neyi doğrular?
**Line-of-Sight (LOS)** güdümünün, rotadan sapmış bir başlangıçtan yola
çıkıldığında aracı **rota eksenine geri çekip çekmediğini** doğrular. Klasik
"hattı yakala ve üstünde kal" davranışı.

## Nasıl kuruldu?
- Güdüm modu: **LOS**, 1 waypoint.
- Süre 65 s, warmup 5 s, mesafe 40 m, başlangıç yana sapması 5 m, derinlik 2 m.
- LOS ileri-bakış (lookahead) 5 m, maks. heading düzeltmesi 12°.

## Sonuçlar (final)
| Ölçüt | Değer |
|---|---:|
| Doğrulama kararı | **KABUL** — LOS rota eksenine yakınsadı |
| Başlangıç yana sapması | −4.97 m |
| Son yana sapma | **0.0013 m** |
| Yana sapma azalma oranı | **%99.97** |
| Yana sapma RMSE | 2.16 m |
| Maks. yana sapma | 4.97 m |
| Heading hata RMSE / maks. | 3.64° / 12.55° |
| Son waypoint mesafesi | 4.62 m |
| Test süresi | 64.7 s |

## Jüriye not (yorum)
Başlangıçta ~5 m sapmış araç, test sonunda rota eksenine **fiilen sıfır** sapma
ile oturmuş (azalma %99.97). Bu, LOS güdümünün hattı doğru yakaladığının net
kanıtıdır.

## Üretilen çıktılar
- Grafikler: `guidance_command_tracking.png`, `guidance_error_history.png`,
  `guidance_path_tracking.png`, `known_signal_timeseries.png`
- 9 PNG · 10 CSV · 1 rosbag
