> [← Stage 2 BT](../stage2_bt/README.md) - [Ana Dogrulama Sayfasi](../README.md) - [Navigation Straight →](../navigation_straight/README.md)

# RL Politika Dogrulama Sonuclari

## Amac

Bu calisma, ZEMHERI kontrol mimarisinde kullanilan RL tabanli rota takip politika adayinin farkli akinti kosullari altindaki performansini degerlendirmek amaciyla gerceklestirilmistir. Testler Gazebo Harmonic simülasyon ortaminda ROS 2 tabanli guidance, navigation ve control katmanlari kullanilarak kosulmustur.

## Test Senaryolari

| Senaryo | Akinti X (m/s) | Akinti Y (m/s) | Sonuc |
|---|---:|---:|---|
| [Akintisiz](episode_detaylari/01_akintisiz/README.md) | 0.00 | 0.00 | KABUL |
| [Takip Eden Akinti](episode_detaylari/02_takip_eden_akinti/README.md) | 0.25 | 0.00 | KABUL |
| [Capraz Akinti](episode_detaylari/03_capraz_akinti/README.md) | 0.00 | 0.25 | KABUL |
| [Diyagonal Akinti](episode_detaylari/04_diyagonal_akinti/README.md) | 0.25 | 0.20 | KABUL |
| [Ters Akinti](episode_detaylari/05_ters_akinti/README.md) | -0.20 | 0.00 | KABUL |
| [Guclu Capraz Akinti Stres](episode_detaylari/06_guclu_capraz_akinti_stres/README.md) | 0.00 | 0.40 | KABUL EDILMEDI |

## Genel Sonuc

| Olcut | Deger |
|---|---:|
| Episode sayisi | 6 |
| Kabul edilen episode | 5 |
| Hedefe ulasan episode | 5 |
| DVL hiz siniri ihlali | 0 |
| Ortalama cross-track RMSE | 1.66389 m |
| Ortalama derinlik RMSE | 0.27356 m |
| Ortalama UKF konum RMSE | 0.10695 m |

## Performans Karsilastirmasi

<img src="gorseller/rl_episode_performance_bars.png" width="1000">

## Rota Karsilastirmasi

<img src="gorseller/rl_episode_trajectories.png" width="1000">

## Yorum

Politika adayi, akintisiz, takip eden, ters ve orta seviyeli capraz/diyagonal akinti kosullarinda hedefe ulasmis ve kabul kosullarini saglamistir. Tum senaryolarda DVL hiz siniri asilmamistir.

Guclu capraz akinti stres senaryosu kabul edilmemistir. Bu sonuc, politikanin saklanmasi gereken operasyonel sinirini gosterir. Navigasyon verisi tamamen kopmamis, fakat yanal suruklenme ve derinlik hatasi artmistir. Bu durum, guclu yanal akinti kosullarinda adaptive LOS, crab-angle telafisi veya kontrol otoritesi iyilestirmesi gerektigini gosterir.

## Dosya Indeksi

| Klasor | Icerik |
|---|---|
| `ozet/` | Birlesik episode karsilastirma CSV/Markdown dosyalari. |
| `gorseller/` | Tum episode'lari birlikte gosteren ana grafikler. |
| `episode_detaylari/` | Her akinti kosulu icin metrik, gorsel, log ve manifest dosyalari. |
| `episode_detaylari/*/ham_veriler/` | Her episode icin guncel `final_validation/results` kosumundan alinmis CSV/JSON/Markdown kayıt dışa aktarımları. |

> [← Stage 2 BT](../stage2_bt/README.md) - [Ana Dogrulama Sayfasi](../README.md) - [Navigation Straight →](../navigation_straight/README.md)
