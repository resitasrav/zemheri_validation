> [Ana Dogrulama Sayfasi](../README.md) 
# Kodlar

Bu klasor, dogrulama sonuclarinin uretilmesinde kullanilan ana analiz ve test kosum betiklerinin inceleme kopyalarini icerir. Bu dosyalar burada **bagimsiz calistirilabilir bir yazilim paketi** olarak tutulmaz. Aktif gelistirme ve gercek test kosumu ana ZEMHERI ROS 2 workspace'i icindeki paketlerle yapilir.

## Neden Bu Kodlar Burada Var?

Bu klasordeki Python dosyalarinin amaci, juri/takim incelemesinde testlerin arka plan mantigini gorunur kılmaktir:

- Hangi test senaryolarinin hangi sirayla kosuldugunu gostermek.
- Hangi ROS 2 topic'lerinin kayda alindigini gostermek.
- Ground truth, UKF, guidance, sensor ve gorev durumlarinin nasil hizalandigini belgelemek.
- CSV/Markdown metriklerinin ve PNG grafiklerinin hangi analiz akisiyle uretildigini gostermek.
- Ana paketlerdeki veri mimarisinin validasyon tarafindan nasil okundugunu seffaf hale getirmek.

Bu nedenle kodlar, sonuclari destekleyen teknik kanit niteligindedir. Tek basina bu depoda calistirildiginda ayni ciktilari uretmesi beklenmez.

## Calisma Bagimliliklari

Betikler asagidaki ana sistem bilesenlerine baglidir:

| Bagimlilik | Neden gerekli? |
|---|---|
| Ana ZEMHERI ROS 2 workspace'i | `zemheri_simulation`, `zemheri_navigation`, `zemheri_guidance`, `zemheri_controller`, `zemheri_mission` paketleri gerekir. |
| ROS 2 mesaj arayuzleri | AUV state, navigation status, fire permission, DVL ve odometri mesajlari ana workspace uzerinden gelir. |
| Gazebo simülasyon ortami | Ground truth, sensor topic'leri, akinti ve arac modeli simülasyondan uretilir. |
| Launch ve parametre dosyalari | Test senaryolari ilgili launch/config dosyalariyla baslatilir. |
| Daha once uretilmis test kayitlari | Analiz betikleri rosbag/CSV/telemetry ciktilarini okuyarak metrik uretir. |

## Dosya Aciklamalari

| Dosya | Gorev | Not |
|---|---|---|
| `report_test_runner.py` | Tekil test senaryosunu baslatir, gerekli topic'leri kaydeder ve analiz cikti klasoru uretir. | Ana ROS 2 workspace, launch dosyalari ve simülasyon olmadan anlamli cikti uretmez. |
| `run_algorithm_validation_suite.py` | Birden fazla dogrulama senaryosunu sirali kosmak icin kullanilir. | Test otomasyonu icin ana giris noktalarindan biridir. |
| `rl_policy_validation.py` | RL episode ciktilarini ground truth, UKF ve hedef rota metrikleriyle analiz eder. | RL test klasorlerindeki metrik ve grafiklerin uretim mantigini gosterir. |
| `aggregate_rl_episodes.py` | Farkli RL akinti senaryolarini tek tablo ve ortak grafiklerde birlestirir. | `rl_politika/ozet` ve `rl_politika/gorseller` ciktilarinin olusumunu temsil eder. |

## Kullanım Notu

Bu dosyalar dogrudan calistirilacaksa once ana ZEMHERI workspace'i kaynaklanmali, ilgili paketler derlenmeli, simülasyon ve gerekli ROS 2 servis/topic altyapisi hazir olmalidir. Aksi durumda import hatasi, eksik mesaj arayuzu, bulunamayan launch/config dosyasi veya bos analiz ciktisi alinmasi normaldir.

Bu validasyon deposundaki asil incelenmesi gereken ciktilar, test klasorlerindeki `README.md`, `metrikler/` ve `gorseller/` dosyalaridir. Kodlar ise bu ciktilarin nasil uretildigini gosteren destekleyici seffaflik katmanidir.
