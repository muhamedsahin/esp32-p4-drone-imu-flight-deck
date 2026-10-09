# Drone IMU Flight Deck

ESP32-P4-NANO üzerinde MPU6050/MPU6500 verilerini okuyup kalibre eden, gyro ve ivmeölçeri birleştirerek **yönelim** hesaplayan ve sonucu yerel ağdaki 3D web panelinde gösteren ESP-IDF projesi.

**Geliştiren: Muhammed Fatih Şahin**

> Bu proje roll, pitch ve başlangıç yönüne göre yaw üretir. GPS konumu, dünya üzerinde X/Y/Z yer değiştirmesi, mutlak kuzey yönü veya motor kontrolü üretmez. Web paneli bir gözlem aracıdır; uçuş kontrol sistemi olarak doğrulanmamıştır.

## Kısa bakış

- MPU6050 (`WHO_AM_I=0x68`) ve MPU6500 (`WHO_AM_I=0x70`) kimliklerini ayırt eder. Sınıf adı `MPU6050`, mevcut API ile uyum için korunmuştur.
- I²C üzerinden yaklaşık **100 Hz** ölçüm alır; ham değerleri kalibre edilmiş **g** ve **rad/s** birimlerine dönüştürür.
- Altı yüzlü ivmeölçer ve sabit durumda gyro kalibrasyonu yapar. Tamamlanan ölçümleri NVS'de saklar; `calibration.txt` içindeki `null` alanlardan devam edebilir.
- Başlangıç roll/pitch değerlerini ivmeölçerden alır. Sonrasında yönelimi quaternion ile gyrodan ilerletir, güvenilir ivme ölçümleriyle eğimi düzeltir.
- ESP32-P4-NANO üzerindeki ESP32-C6 üzerinden Wi-Fi kullanır. `wifi.txt` ayarına göre modeme bağlanır veya kendi erişim noktasını açar.
- Yerel web panelinde 3D model, açılar, ivme/gyro ölçümleri, grafik ve CSV dışa aktarma bulunur. Arayüz dosyaları firmware'e gömülüdür; CDN veya internet gerekmez.

## Donanım ve bağlantı

Hedef kart **Waveshare ESP32-P4-NANO**'dur. ESP32-P4'ün kendi Wi-Fi radyosu yoktur; kart üzerindeki ESP32-C6, P4'e SDIO üzerinden bağlı Wi-Fi yardımcı işlemcisidir. [Kart bilgisi](https://docs.waveshare.com/ESP32-P4-NANO) ve [şeması](https://files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf).

| IMU pini | ESP32-P4-NANO bağlantısı |
| --- | --- |
| VCC | Modülünün desteklediği besleme gerilimi; modül üzerindeki regülatörü kontrol et |
| GND | GND |
| SDA | GPIO7 |
| SCL | GPIO8 |

Sürücü I²C adresini `0x68` (AD0 düşük) ve hattı 100 kHz olarak ayarlar. AD0 yüksek bağlanırsa adres `0x69` olur; mevcut sürücü bu adres için ayrıca değiştirilmelidir. IMU eksenleri ile drone gövde eksenlerinin aynı yönde olduğu varsayılır; farklı montajda eksen dönüşümü eklenmelidir. Bağlantı ve beslemeyi kendi modülünün özelliklerine göre doğrula.

## Başlatma ve web paneli

ESP-IDF **6.1** ortamını açıp proje kökünde çalıştır:

```powershell
. C:\esp\v6.1\esp-idf\export.ps1
idf.py -B build-idf build
idf.py -B build-idf -p COM17 flash monitor
```

`COM17` yerine kartının portunu yaz. `build-idf`, bu projeye ayrılmış derleme klasörüdür; farklı bir CMake üreticisinin kullandığı `build` klasörüyle karışmaz. İlk derleme ESP-Hosted ve Wi-Fi Remote bağımlılıklarını indirebilir.

Varsayılan [wifi.txt](wifi.txt) ayarında `ssid` ve `password` alanları `null` olduğu için kart **Drone-IMU** ağını açar. Varsayılan parola `drone6050`; bu ağa bağlandıktan sonra **http://192.168.4.1** adresini aç. Modem kullanmak istersen aynı dosyaya çift tırnak içinde 2.4 GHz ağ adını ve parolasını yaz, ardından yeniden derleyip P4'e yükle. STA modundaki panel adresi seri konsolda `DRONE_WIFI: Panel adresi` satırında görünür. Bağlanamazsa yapılandırılan süre sonunda AP moduna geçer.

`wifi.txt` ve `calibration.txt` çalışma anında kartın bilgisayardaki dosyaları okuduğu dosyalar değildir: **derleme sırasında firmware'e gömülürler**. Düzenlemelerin karta geçmesi için yeniden build/flash gerekir. Kart, kalibrasyon ilerlemesinin daha yeni hâlini ayrıca NVS'de korur.

Gerçek kart olmadan yalnız arayüzü görmek için `python tools/preview_drone_web.py` çalıştırıp `http://127.0.0.1:8765` adresini açabilirsin. Bu önizleme gerçek sensör verisi sunmaz; arayüzdeki demo açıkça ayrı işaretlenir.

**Wi-Fi parolası:** Gerçek modem şifresini `wifi.txt` içine koyarsan dosya ve derlenmiş firmware bu bilgiyi içerir. Herkese açık GitHub deposuna gerçek şifreyi veya gerçek şifre içeren firmware'i yükleme; yayımlamadan önce yerel dosyayı örnek/null değerlere döndür. Varsayılan AP parolasını da kullanımdan önce değiştir. Panel yalnız yerel ağ için salt okunur HTTP servisidir.

## Proje mimarisi

```text
main/main.cpp
  ├─ IMU görevi (~100 Hz)
  │    └─ OrientationSystem
  │         ├─ ImuApplication → MPU6050 → I²C, kalibrasyon, NVS
  │         └─ AttitudeEstimator → QuaternionUtils
  └─ DroneDashboard
       ├─ WifiManager → ESP32-C6 üzerinden Wi-Fi
       └─ HTTP /api/telemetry → tarayıcı → WebGL 3D model
```

`main.cpp` iki işi başlatır: IMU ölçüm görevi ve ağ/panel görevi. IMU görevi `OrientationSystem::init()` çağırır, ardından yaklaşık her 10 ms'de `update()` ile yeni ölçüm üretir ve `DroneDashboard::publish()` ile son sonucu paylaşır. Web sunucusu aynı sonucun **kopyasını** okur; HTTP isteği sensörden yeni ölçüm başlatmaz. IMU görevinin önceliği ağ görevinden yüksektir. Mevcut `main.cpp` içinde periyodik `orientation.print()` çağrısı yorum satırındadır; kalibrasyon ve hata mesajları yine konsola yazılır.

| Yol | Görevi |
| --- | --- |
| `main/` | Uygulama girişi, FreeRTOS görevleri ve bileşenlerin bağlanması. |
| `components/imu/` | MPU register/I²C sürücüsü, fiziksel birime dönüşüm, kalibrasyon matematiği, metin/NVS kaydı. |
| `components/attitude/` | İlk açı tahmini, zaman yönetimi, quaternion tabanlı gyro entegrasyonu ve ivme düzeltmesi. |
| `components/orientation/` | IMU ile attitude katmanlarını tek `OrientationSystem` API'sinde birleştirir; kalibrasyon akışını yönetir. |
| `components/drone_web/` | Wi-Fi ayarı, AP/STA yönetimi, HTTP JSON API ve gömülü HTML/CSS/JS/WebGL paneli. |
| `tests/` | Bilgisayarda çalışan kalibrasyon, sürücü, yönelim, ağ yapılandırması ve web matematiği testleri. |
| `tools/` | Seri konsoldan kalibrasyon kaydı alma ve paneli bilgisayarda önizleme araçları. |
| `calibration.txt` | Okunabilir kalibrasyon kaynağı; `null` eksik ölçümü ifade eder. |
| `wifi.txt` | Derlemeye gömülen Wi-Fi/erişim noktası ayarları. |
| `sdkconfig.defaults` | ESP32-P4-NANO'nun P4↔C6 SDIO bağlantısı için başlangıç ayarları. |
| `CALIBRATION.md` | Seçici kalibrasyon, kayıt ve kurtarma akışının ayrıntıları. |

Her bileşenin kendi `CMakeLists.txt` dosyası bağımlılıklarını tanımlar. `components/*/include/` dışarıya açılan başlıkları, `components/drone_web/private/` yalnız web bileşeninin kullandığı başlıkları tutar. Daha ayrıntılı bileşen belgeleri: [attitude](components/attitude/README.md), [orientation](components/orientation/README.md), [web paneli](components/drone_web/README.md).

## Temel sınıflar ve fonksiyonlar

### `MPU6050` — sensör ve ham ölçüm

| Fonksiyon | Görevi |
| --- | --- |
| `init()` | I²C hattını/sensörü başlatır, kimliği denetler ve örnekleme/filtre/ölçüm aralıklarını ayarlar. |
| `sensorIdentity()`, `sensorName()` | Okunan çip kimliğini ve model adını verir. |
| `readRaw(...)` | Sensör register'larından zaman damgalı ham ivme, gyro ve sıcaklık sayımlarını okur. |
| `read(...)` | Ham veriye kayıtlı kalibrasyonu uygular; ivmeyi g, gyro hızını rad/s olarak `IMUData` içinde döndürür. |
| `calibrateGyroscope(...)` | Sabit sensörde gyro sıfır kaymasını ve ölçüm kalitesini hesaplar. |
| `calibrateAccelerometer(...)` | Altı yüz ölçümüyle ivmeölçer modelini kurar ve doğrular. |
| `calibrateFromText(...)` | `calibration.txt`/NVS durumuna göre yalnız eksik adımları çalıştırır; her kabulden sonra checkpoint üretir. |
| `loadAccelerometerCalibration()`, `saveAccelerometerCalibration()` | Eski ikili ivme kalibrasyon kaydıyla uyumluluk için okuma/yazma API'si. |
| `recoverAccelerometerFromLog(...)` | Gerçek eski log ortalamalarından kurtarma için özel API; normal ana akış bunu çağırmaz. |
| `accelerometerCalibration()`, `gyroscopeCalibration()`, `calibrationReport()` | Son kalibrasyon modellerini ve durum raporunu verir. |

### Kalibrasyon yardımcıları

`imu_calibration::RunningStats::add()` örnekleri sabit bellekle toplar; `mean()`, `stddev()` ve `range()` kararlılık denetimi için ortalama, standart sapma ve aralığı verir. `stationary()` hareketli veya kararsız ölçüm penceresini reddeder. `fitAccelerometer()` altı yüzden ivme bias/ölçek modelini hesaplar; `correctAcceleration()` modeli yeni ölçüme uygular. `parseCalibrationText()` ve `calibrationText()` okunabilir kayıt ile model arasında dönüşüm yapar; `completedAccelerometer()`/`completedGyroscope()` kaydın gerçekten kullanılabilir olup olmadığını kontrol eder. `TextStore::load()` kaynak TXT ile NVS'deki daha yeni durumu uzlaştırır; `save()` kaydı saklar. CRC ve sensör profili kontrolleri bozuk ya da başka sensöre ait verinin kullanılmasını önler.

`ImuApplication::init()` sensörü ve metin kaydını hazırlar; `calibrate()` eksikleri tamamlar; `read()` geçerli ölçüm alır. `printCalibration()` son kayıt durumunu, `printMeasurement()` son fiziksel ölçümü konsola yazar.

### `AttitudeEstimator` ve `QuaternionUtils` — yönelim

| Fonksiyon | Görevi |
| --- | --- |
| `calculateAccelAngles(data)` | İvme vektöründen başlangıç roll/pitch açısını hesaplar. |
| `gyroinitialize(data)` | İlk güvenilir ölçümden roll/pitch, sıfır başlangıç yaw ve ilk zaman referansını kurar. |
| `update(data)` | Normal yol: gyro ile quaternion'u ilerletir, uygun ivme verisiyle eğimi düzeltir. |
| `updateGyro(data)` | Karşılaştırma için yalnız gyro entegrasyonu yapar; aynı örnekte `update()` ile birlikte çağrılmaz. |
| `isInitialized()`, `getAngles()`, `getQuaternion()` | Başlangıç durumunu ve son Euler/quaternion sonucunu verir. |
| `printGyroAngles()` | Hesaplanan Euler açılarını konsola yazdırır. |
| `QuaternionUtils::fromEuler(...)` | İlk roll/pitch/yaw değerlerinden birim quaternion oluşturur. |
| `updateFromGyroscope(...)` | Açısal hızı quaternion türeviyle entegre eder. |
| `updateFromImu(...)` | İvme güvenine göre gyro hızını düzeltip entegre eder. |
| `getEulerAngles()`, `getQuaternion()` | İçte tutulan yönelimi çıktı biçiminde sunar. |

`OrientationSystem::init()` sensörü ve kalibrasyonu hazırlar. `update()` sensörden kendisi okur; `update(const IMUData&)` dışarıdan verilen kalibre edilmiş ölçümü işler. `getOrientation()` son `OrientationData` kopyasını döndürür: açılar, quaternion, o sonucu üreten ölçüm ve `valid` bayrağı. Başarısız okumada önceki sayılar korunur ama `valid=false` olur; eski veri yeniymiş gibi kullanılmamalıdır. `print()` yalnız geçerli sonucu konsola basar.

### `DroneDashboard` — ağ ve görselleştirme

`start()` Wi-Fi/HTTP görevini bir kez başlatır. `publish(orientation)` son sonucu kısa bir kritik bölümde kopyalar ve sıra numarasını artırır. İç ağ görevi `wifi.txt` içeriğini `parseNetworkConfig()` ile okur; `WifiManager::begin()` AP veya STA bağlantısını açar, `maintain()` kopma/zaman aşımı durumunu yönetir. HTTP sunucusu gömülü arayüz dosyalarını ve `GET /api/telemetry` yanıtını sunar. Tarayıcıdaki `app.js` canlı geçerlilik, gösterge/grafik/CSV ve isteğe bağlı demo akışını; `math.js` koordinat/quaternion işlemlerini; `renderer.js` WebGL sahnesini yönetir.

## Kullanılan matematik

### 1. Kalibrasyon ve birimler

Gyro için her eksende sabit durum bias'ı çıkarılır: `ω_dps = (raw_gyro - bias_count) / 131`; sonuç `π/180` ile **rad/s**'ye çevrilir. İvmeölçer için her eksende `a_g = (raw_acc - bias_count) / counts_per_g` kullanılır. Altı yaklaşık ±X/±Y/±Z yönünden üç bias ve üç hassasiyet bulunur; model şu 1g norm koşuluna uydurulur:

```text
((raw_x - b_x)/s_x)^2 + ((raw_y - b_y)/s_y)^2 + ((raw_z - b_z)/s_z)^2 ≈ 1
```

İlk tahmin karşılıklı yüzlerden çıkarılır, ardından sınırlı Newton/backtracking iyileştirmesi yapılır. Fit için kullanılmayan eğik duruşlar ayrıca doğrulanır. Bu **köşegen** bir bias/ölçek modelidir; tam 3×3 eksenler arası dönüşüm veya sıcaklık telafisi değildir.

### 2. Başlangıç açıları

İlk güvenilir ve yaklaşık 1g ölçümünde:

```text
roll  = atan2(ay, az)
pitch = atan2(-ax, sqrt(ay² + az²))
yaw   = 0   (başlangıç referansı)
```

Bu açılar ZYX sırasıyla `[w, x, y, z]` Hamilton quaternion'una çevrilir. Quaternion **gövde → dünya** yönelimini tutar. Dünya +Z, düz duran sensörün gördüğü +1g destek ivmesi yönü olarak kabul edilir.

### 3. Gyro + ivme birleştirme

Her yeni ölçümde iki zaman damgasının farkı saniyeye çevrilir. Gyro, saf açısal hız quaternion'u `Ω = [0, ωx, ωy, ωz]` olarak kullanılır:

```text
q̇ = ½ (q ⊗ Ω)
q_yeni = normalize(q + q̇ · Δt)
```

İvmenin büyüklüğü yaklaşık 1g ise ölçülen birim ivme yönü ile quaternion'dan beklenen yön aynı **gövde eksenlerinde** karşılaştırılır. `e = a_ölçülen × a_beklenen` hata vektörü, `ω_düzeltilmiş = ω_gyro + Kp · güven · e` şeklinde geri beslenir (`Kp = 2/s`). 1g'den sapma 0,05g'ye kadar tam güven, 0,20g'de sıfır güvendir. İvme güvenilmezse gyro takibi sürer. Her adımda quaternion normalize edilir; Euler roll/pitch/yaw yalnız gösterim için bundan çıkarılır.

Bu oransal **complementary** filtredir; çevrimiçi integral gyro bias öğrenmez. Manyetometre olmadığı için yaw mutlak kuzey değildir ve zamanla kayabilir. Pitch ±90° yakınında Euler gösterimi sıçrayabilir; iç quaternion temsili bu tekilliği yaşamaz. `|a|≈1g`, sensörün kesinlikle hareketsiz olduğunu kanıtlamaz.

## Kalibrasyon ve kayıt akışı

Başlangıçta dosya/sensör kimliği kontrol edilir, gömülü `calibration.txt` ile NVS'nin kayıt durumu değerlendirilir. Geçerli tam kayıt varsa kalibrasyon atlanır. `null` eksik alanı ifade eder; **sayısal sıfır geçerli değerdir**. Bir ivme ekseni eksikse ilgili yüzler yeniden ölçülür ve bütün model yeniden fit/doğrulama görür. Gyro ekseni eksikse gyro bias'ları sabit bir pencerede birlikte ölçülür. Her kabul edilen aşama NVS'ye checkpoint olarak yazılır; yarım işlem sonraki açılışta devam edebilir.

Kartın son kaydını PC'deki metin dosyasına almak için seri portu kullanan araç:

```powershell
python tools/calibration_monitor.py --port COM17
```

Araç güncel kaydı `calibration.txt` dosyasına atomik yazar, öncekini `calibration.previous.txt` olarak tutar. `idf.py monitor` ile aynı COM portunu eşzamanlı açma. Dosya biçimi, seçici `null` düzenlemeleri ve bütün kurtarma ayrıntıları [CALIBRATION.md](CALIBRATION.md) içinde.

## Telemetri sözleşmesi ve sınırlar

`GET /api/telemetry` JSON'u `valid`, `sequence`, `timestamp_us`, `age_ms`, `quaternion`, `angles_rad`, `accel_g`, `gyro_rad_s`, `temperature_c` ve kalibrasyon bayraklarını içerir. Sonuç geçersiz, sonlu olmayan veya **500 ms**'den eskiyse `valid=false` döner. Tarayıcı yaklaşık **20 Hz** istek yapar; 3D sahne kendi çizim döngüsünde ara kareleri quaternion interpolasyonuyla üretir. Demo gerçek sensör verisi yerine kendiliğinden geçmez.

Mevcut sensör profili **±2g, ±250°/s**, yaklaşık 100 Hz ve düşük geçiren filtre ayarlarıdır. [Okuma kodu](components/imu/mpu6050.cpp) herhangi bir ham eksen sınıra yaklaşınca (`|count| ≥ 32700`) tüm örneği geçersiz sayar. Çok hızlı döndürme veya sert sarsma bu yüzden panelde kısa veri kesintisi gibi görünebilir. I²C okuma hataları ise ayrıca konsola yazılır. 100 ms'den uzun ölçüm boşluğunda filtre eksik hareketi tahmin etmez; zaman referansını yenileyip sonraki ölçümden devam eder. Ölçüm aralığı değiştirilecekse dönüşüm katsayıları ve kayıtlı kalibrasyon profili beraber ele alınmalıdır.

## Testler ve durum

Bilgisayardaki C++ testleri gerçek kalibrasyon, sürücü ve API kodunu simüle I²C/NVS/FreeRTOS katmanlarıyla çalıştırır:

```powershell
cmake -S tests -B build/host-tests
cmake --build build/host-tests --config Debug
ctest --test-dir build/host-tests -C Debug --output-on-failure
node tests/web_math_tests.mjs
```

Web matematiği testleri quaternion dönüşümleri, eksen eşlemesi ve interpolasyonu denetler. Proje ESP-IDF 6.1 ile derlenmiştir; **gerçek P4↔C6 Wi-Fi bağlantısı ve uçuş davranışı donanımda ayrıca doğrulanmalıdır**. ESP-Hosted bağımlılığının derlenmesi, karttaki C6 firmware sürümünün uyumlu olduğunu tek başına göstermez. Donanım/ağ ayrıntıları için [web bileşeni belgesine](components/drone_web/README.md) bak.

---

Geliştiren: **Muhammed Fatih Şahin**.
