# RL Araçları — Eksik Grafikleri Üretme + Per-Episode Raporlama

Bu klasör, RL politika doğrulamasındaki **iki boşluğu** kapatmak içindir:
1. Eksik RL grafiklerini **kayıtlı CSV'lerden** üretmek.
2. Episode matrisini **her senaryoyu ayrı raporlayacak** şekilde koşmak.

---

## 1) `plot_validation_figures.py` — RL grafik üreticisi

### Ne yapar?
Bir test/episode çıktı klasörünü okuyup standart figür setini üretir:

| Figür | Kaynak CSV | Açıklama |
|---|---|---|
| `trajectory_xy.png` | `recording/telemetry.csv` | GT vs UKF yatay yörünge |
| `ukf_position_error.png` | `recording/telemetry.csv` | UKF konum hatası (RMSE çizgili) |
| `depth_speed_tracking.png` | `recording/known_signal_timeseries.csv` | derinlik & hız + hedefleri |
| `ocean_current_activity.png` | `…ocean_current…timeseries.csv` | akıntı X/Y/Z + büyüklük |
| `rl_summary_panel.png` | yukarıdakiler | `--rl` ile tek panel jüri özeti |

Bütün CSV'ler **opsiyonel**: hangisi varsa ona ait figürü üretir.

### Kullanım
```bash
# RL episode kaydınızın olduğu klasörü verin:
python3 plot_validation_figures.py results/rl_policy_ep06_hard_cross_current_XXXX \
        --title "RL ep06 hard_cross" --rl

# Çıktılar varsayılan olarak <klasör>/figures/ altına yazılır.
# Farklı bir yere yazmak için: --out figures_ep06
```

### Bu araç **gerçek veriyle test edildi**
Bu pakette RL çıktı klasörleri (CSV) bulunmadığından, araç aynı şemayı kullanan
**ortam-doğrulama kaydı** (`ocean_current_services_20260615_135048`) üzerinde
koşturuldu ve şu figürler üretildi (`../figures/` altında, `ornek_` önekiyle):
`ornek_trajectory_xy.png`, `ornek_ukf_position_error.png`,
`ornek_depth_speed_tracking.png`. Yani **boru hattı çalışıyor** — RL CSV'lerini
verdiğin an aynı figürleri RL için üretecek.

### RL CSV'lerini nereden alacaksın?
Her test koşumu `results/<case>_<zaman>/recording/` altına `telemetry.csv` ve
`known_signal_timeseries.csv` yazıyor (ortam testlerinde gördüğümüz şema). RL
episode klasörleri de aynı şemada. O klasörü bu araca vermen yeterli.

> Eğer Melike RL için **ek/özel CSV** (ör. `rl_policy_summary.csv`,
> `rl_episode_summary.csv`) üretmişse, kolonlarını bana ilet; üreticiye o
> kolonlara özel paneller (ödül eğrisi, cross-track geçmişi vb.) eklerim.

---

## 2) Per-episode raporlama düzeltmesi (öneri)

### Sorun
`test_scripts/run_final_validation.py` RL matrisini koşarken her episode'u
**aynı `rl_policy` satırına** yazıp üzerine biniyor. README sadece **son
episode'u** gösteriyor; senaryo karşılaştırması kayboluyor.

### Kök neden
`existing_results()` ve `write_index()`, RL satırını tek bir `rl_policy` anahtarı
üzerinden tutuyor. Oysa koşum sırasında her episode `rl_policy_ep01…ep06` diye
ayrı klasör/satır üretiyor (kodda `episode_case = f"rl_policy_ep{idx:02d}_{ad}"`).
Sorun **özet/index tarafında** birleştirme/üzerine yazma mantığında.

### En küçük düzeltme
README'ye her RL episode'unu **ayrı satır** olarak yansıtmak. Pratikte:
- `write_index` zaten `rows` listesindeki her satırı yazıyor; RL episode satırları
  `rl_policy_ep*` adıyla zaten ekleniyor. Index'te tek satır görünüyorsa, eski
  `rl_policy` satırının temizlenip episode satırlarının korunduğundan emin ol
  (ana döngüdeki `rows = [row for row in rows if not row['case'].startswith('rl_policy_ep')]`
  filtresinin episode'ları **silmediğini** doğrula).
- Ayrıca her episode için bir **özet CSV** (`rl_episode_summary.csv`) yazıp, hepsini
  toplayan küçük bir karşılaştırma tablosu (senaryo × UKF RMSE × cross-track RMSE ×
  karar) üretmek en temiz çözüm.

> Kodun bütününü paylaşırsan (özellikle `report_test_runner.py` ve RL analiz
> scripti), bu düzeltmeyi doğrudan diff olarak hazırlarım.

---

## Özet
- Grafik boru hattı **hazır ve kanıtlanmış** → RL CSV'leri gelince tek komut.
- Per-episode kaybı **tanımlandı** → düzeltme net; kod gelince diff çıkarılır.
