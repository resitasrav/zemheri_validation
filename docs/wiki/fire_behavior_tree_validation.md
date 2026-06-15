# Fire Behavior Tree Validation (Stage 2)

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
Yarışma **Aşama-2** görevini (yaklaşım → pitch-up → güvenli ateşleme → roket ayrılması) bir Davranış
Ağacı (Behavior Tree, BT) ile uçtan uca koşturmak ve modüler görev kurgusunun çalıştığını doğrulamak.

## Methodology
`control_backend:=ros`, görev süresi 100 s'ye kadar; tam görev düğüm yığını.

## Inputs
`mission_manager_node` (Aşama-2 BT), `safety_monitor_node` (heartbeat · AI disable · **fire inhibit**),
kontrolcüler, UKF.

## Metrics
| Metrik | Değer |
|---|---:|
| Örnek sayısı | 1138 |
| Süre | 37.9 s |
| Konum RMSE | 0.722 m |
| Derinlik RMSE | 0.178 m |
| Roll / Pitch RMSE | 0.51° / 2.58° |
| Maks. pitch (GT) | 29.09° |
| İz boyu mesafe | 42.73 m |
| Maks. cross-track | 0.43 m |

## Results
BT görevi düşük cross-track (0.43 m) ile tamamlandı. Pitch ekseninde tepe değer 29° (dalış/çıkış
manevrası); pitch RMSE 2.58° görev senaryosunun gerektirdiği manevrayla tutarlı.

## Decision
- **PASS (görev yürütme):** Aşama-2 BT'si uçtan uca, düşük cross-track ile çalıştı.
- **Needs Evidence (ateşleme karar mantığı):** Ateşleme inhibit/permit kararı mimaride
  `safety_monitor_node` üzerinde tanımlıdır (bkz. [architecture](../architecture/SARA_Baglanti_Listesi.csv),
  `safety_monitor_node → mavlink_bridge_node: failsafe / surface`). Ancak bu bundle'da ateşleme karar
  koşullarını (güvenli açı/derinlik/mesafe penceresi, inhibit tetikleri) **izole eden bir test ve kanıt
  dosyası yoktur**. Ateşleme mantığının ayrı doğrulanması bir sonraki adımdır.

## Evidence Files
- [tests/09_stage2_bt.md](../../tests/09_stage2_bt.md)

## Limitations
Ham çıktılar (14 PNG · 10 CSV · 1 rosbag) bu bundle'da değildir. Ateşleme-karar mantığı için doğrudan
dosya kanıtı yoktur (yalnızca mimari tanımı mevcut).
