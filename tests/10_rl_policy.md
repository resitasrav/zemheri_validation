# 10 — RL Politika Doğrulama · `rl_policy`

**Durum: ❌ BAŞARISIZ** — *ROS/Gazebo politika adayı koşulları sağlanmadı*
· Kategori: *RL politika doğrulama*

> **Önce çerçeve (jüri için kritik):** Bu test bir **eğitilmiş SAC ajanını**
> değerlendirmez. README'nin açıkça belirttiği gibi, *"RL policy validation
> gerçek Gazebo modeli, UKF, sensör, guidance ve controller zinciri üzerinden
> seçilmiş politika **adayını** doğrular. Bu sonuç eğitilmiş bir SAC ajanı
> sonucu değildir."* Yani buradaki "BAŞARISIZ", **zincirin çalışmadığı**
> anlamına gelmez; aday politikanın kabul eşiklerini **henüz** karşılamadığı
> anlamına gelir. Bu beklenen ve bilgilendirici bir ara sonuçtur.

## Bu test neyi doğrular?
Seçilen politika adayının, tam ROS/Gazebo zinciri (UKF + guidance + kontrolcüler)
üzerinde, **farklı akıntı senaryolarında** hedefe ilerleyip ilerleyemediğini
ölçer. Test bir **episode matrisi** olarak koşulur: her episode için Gazebo
sıfırdan başlatılır, rosbag alınır, analiz yapılır, Gazebo kapatılır.

## Episode matrisi (6 senaryo)
| # | Episode | Akıntı (x, y, z) m/s | Süre |
|---|---|---|---:|
| ep01 | `no_current` | (0.00, 0.00, 0.00) | 75 s |
| ep02 | `following_current` | (0.25, 0.00, 0.00) | 75 s |
| ep03 | `cross_current` | (0.00, 0.25, 0.00) | 75 s |
| ep04 | `diagonal_current` | (0.25, 0.20, 0.00) | 85 s |
| ep05 | `reverse_current` | (−0.20, 0.00, 0.00) | 90 s |
| ep06 | `hard_cross_current` | (0.00, 0.40, 0.00) | 95 s |

Bu pakette **ep03 runner logu `[COMPLETE]`** ile temiz tamamlanmış; ep04/ep05/ep06
simülasyon logları temiz kapanmış (kapanıştaki `exit −2` yalnızca SIGINT'tir).
Yani senaryolar fiilen koşturulmuş.

## README'ye yansıyan (tek) sonuç
README index'i RL için tek bir satır tutuyor ve **son episode'un** metriklerini
gösteriyor (büyük olasılıkla en zoru olan `hard_cross_current`):

| Ölçüt | Değer |
|---|---:|
| Doğrulama kararı | **BAŞARISIZ** |
| Test süresi | 90.1 s |
| Hedef mesafe | 49.52 m |
| İlerleme | 58.66 m |
| Son yana sapma | −8.30 m |
| Yana sapma RMSE | 3.14 m |
| Derinlik RMSE | 1.62 m |
| **UKF konum RMSE** | **35.09 m** |
| Maks. hız | 0.95 m/s |
| DVL hız sınırı ihlali | 0 |
| Navigasyon geçerli oranı | 1.0 |

## ⚠️ İki önemli boşluk (Melike'nin notlarıyla birebir)

**1) Episode'lar ayrı ayrı saklanmıyor.**
Koşucu (`run_final_validation.py`), RL matrisinin **her episode'unu aynı
`rl_policy` satırına yazıp üzerine bindiriyor** (`existing_results` en son
`rl_policy_*` klasörünü seçiyor). Sonuç: README yalnızca **son episode'u**
gösteriyor; "no_current başardı mı, cross_current'ta ne oldu?" ayrımı kayboluyor.
Melike'nin "farklı senaryolarda test yaptım ama emin olamadım" demesinin
teknik karşılığı bu: matris koşuyor ama **per-episode karşılaştırma raporu yok**.
→ *Düzeltme önerisi aşağıda, `rl_tools/README_RL.md` içinde.*

**2) Birkaç RL grafiği eksik / üretilmemiş.**
README'nin RL için beklediği grafikler `rl_policy_tracking.png` ve
`rl_policy_trajectory.png`. CSV'ler kayıtlı olduğu için bunlar CSV'den
üretilebilir — ancak **bu zip içinde RL çıktı klasörleri (CSV'ler) yok, yalnızca
loglar var.** Bu yüzden grafikleri burada doğrudan üretemedim.
→ Bunun yerine, RL kayıt CSV'leriyle **doğrudan çalışan** bir grafik üreticisi
hazırladım: `rl_tools/plot_validation_figures.py`. Aynı şema üzerinde çalışan
ortam-doğrulama kaydıyla **test edilip kanıtlandı** (örnek figürler `figures/`
altında). RL CSV'lerini bu araca verdiğinde tracking/trajectory/UKF-hata/akıntı
figürlerini tek komutla üretir.

## Jüriye not (yorum)
- Zincirin tamamı (Gazebo→sensör→UKF→guidance→kontrolcü) **6 akıntı senaryosunda
  da koştu**; navigasyon geçerli oranı 1.0 ve DVL hız ihlali 0.
- "BAŞARISIZ", aday politikanın özellikle **güçlü yanal akıntı** altında hedefe
  oturamadığını gösteriyor (UKF konum RMSE 35 m, son yana sapma −8.3 m).
- Bir sonraki adım: aday politikayı **eğitilmiş SAC ajanı** ile değiştirmek ve
  matrisi per-episode raporlayacak şekilde koşmak.

## Bu pakette ne var / ne eksik
- ✅ ep03 runner + ep04/05/06 simülasyon logları (senaryoların koştuğunun kanıtı)
- ✅ Çalışan RL grafik üreticisi (`rl_tools/plot_validation_figures.py`)
- ❌ RL çıktı klasörleri / CSV'leri (bu zip'te yok → grafik için gerekli)
