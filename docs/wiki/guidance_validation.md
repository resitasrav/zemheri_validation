# Guidance Validation

[← README](../../README.md)

## İçindekiler
- [Purpose](#purpose)
- [Methodology](#methodology)
- [Inputs](#inputs)
- [Metrics](#metrics)
- [Results](#results)
- [Decision](#decision)
- [Evidence Files](#evidence-files)
- [Limitations](#limitations)

## Purpose
Line-of-Sight (LOS) güdümünün rota eksenini yakalamasını ve çoklu-waypoint rotasının kabul yarıçapı
içinde sırayla tamamlanmasını doğrulamak.

## Methodology
- **LOS:** 1 waypoint, başlangıç sapması 5 m, lookahead 5 m, maks. heading düzeltmesi 12°, 65 s.
- **Waypoint:** 4 waypoint, kabul yarıçapı 1.5 m, mesafe 60 m, 150 s'ye kadar.

## Inputs
`guidance_node` (LOS / Waypoint modu), `control_setpoint_node`, UKF state (`/odometry/ukf`).

## Metrics
| Metrik | LOS | Waypoint |
|---|---:|---:|
| Doğrulama kararı | KABUL | KABUL |
| Başlangıç cross-track | −4.97 m | — |
| Son cross-track | 0.0013 m | 1.15 m |
| Cross-track azalma | %99.97 | — |
| Cross-track RMSE | 2.16 m | 0.580 m |
| Heading RMSE / maks. | 3.64° / 12.55° | 5.42° / 15.90° |
| Son waypoint mesafesi | 4.62 m | 2.54 m |

## Results
LOS: ~5 m sapmış araç rota eksenine fiilen sıfır sapma ile oturdu (%99.97 azalma). Waypoint: 4-nokta
rotası 0.58 m cross-track RMSE ile düzgün tamamlandı; heading hatası dönüş anlarında beklenen biçimde
yükseldi.

## Decision
**PASS** — her iki güdüm modu da kabul kriterlerini sağladı.

## Evidence Files
- [tests/03_guidance_los.md](../../tests/03_guidance_los.md)
- [tests/04_guidance_waypoint.md](../../tests/04_guidance_waypoint.md)

## Limitations
Ham çıktılar (her test: 9 PNG · 10 CSV · 1 rosbag) bu bundle'da değildir; metrikler analiz
manifestlerinden taşınmıştır.
