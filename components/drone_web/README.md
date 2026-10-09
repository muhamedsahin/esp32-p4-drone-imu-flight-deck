# Flight Deck — yerel 3D yönelim paneli

`DroneDashboard` yalnız mevcut `OrientationData` sonucunu yayımlar. IMU, attitude,
orientation ve kalibrasyon algoritmaları değişmez. Bu bir görselleştirme panelidir;
uçuş kontrolü, motor komutu veya dünya üzerindeki XYZ konumu üretmez.

## Kullanım

Proje kökündeki `wifi.txt` dosyasını düzenle. Varsayılan:

```text
ssid = null
password = null
ap_ssid = "Drone-IMU"
ap_password = "drone6050"
connect_timeout_s = 30
```

1. ESP-IDF terminalinde `idf.py -B build-idf build` çalıştır.
2. `idf.py -B build-idf -p COM17 flash monitor` ile **P4** firmware'ini yükle.
   COM portu farklıysa değiştir; monitör açıksa önce Ctrl+] ile kapat.
3. Telefon/bilgisayardan `Drone-IMU` ağına `drone6050` şifresiyle bağlan.
4. Tarayıcıdan **http://192.168.4.1** aç. Ağın interneti olmaması normaldir;
   telefonda bu ağda kalmayı seç. Bu bir captive portal değildir, adresi açmalısın.

Kendi modemine bağlanması için yalnız `ssid` ve `password` alanlarını çift tırnaklı
gerçek değerlerle değiştir. C6 için **2.4 GHz**, WPA2 veya WPA2/WPA3 karma ağ kullan.
Her iki alan doluysa STA denenir; herhangi biri `null`/boş ise doğrudan AP açılır.
Bağlantı kurulamaz veya DHCP dahil bağlantı belirtilen süre boyunca kaybolursa
AP'ye geçilir. AP'ye geçiş bu açılış için kalıcıdır; modemi yeniden denemek için
kartı yeniden başlat. STA modunda adresi `DRONE_WIFI: Panel adresi` logundan oku.

TXT, dosya sistemi değil **derleme sırasında gömülen bir kaynak**: bilgisayarda
değiştirince build + flash gerekir. Kart diski/SD kart gerekmez. Hatalı dosya
varsayılan güvenli AP'ye düşer ve sır içermeyen hata yazar. SSID en çok 32 bayt,
şifreler 8–63 ASCII karakter; zaman aşımı 5–120 saniye. `#` tırnak içinde şifrenin
parçasıdır. Tırnak içindeki ters bölü normal karakterdir; kaçış dizisi desteklenmez.

Şifre konsola veya HTTP API'ye yazılmaz. TXT ve firmware şifreyi açık biçimde
içerir: bunları herkese açık depoya gönderme. Varsayılan AP şifresini değiştir.
Panel salt okunur HTTP'dir, yalnız güvenilir yerel ağ içindir; internete port açma.

## ESP32-P4-NANO donanım yolu

Waveshare NANO'daki ESP32-C6-MINI-1, P4'e SDIO ile bağlıdır. P4 tek başına radyo
içermez. CLK=18, CMD=19, D0=14, D1=15, D2=16, D3=17, C6 EN/reset=54;
SDMMC slot 1, 4 bit. Harici MPU'nun GPIO7/8 hattıyla çakışmaz.
`sdkconfig.defaults` bu ayarları belgeler; mevcut `sdkconfig` önce gelir.

ESP-IDF 6.1 için `esp_hosted` **2.12.7**, `esp_wifi_remote` **1.6.5** bağımlılıkları
çözümlendi; `dependencies.lock` sürümleri sabitler. İlk derleme sürücüleri indirir.
IDF 6.1 Wi-Fi Remote uygulamasını zaten içerir; registry remote bileşeni bunu
algılar ve ikinci bir sürücü derlemez. ESP-Hosted güçlü RPC uygulamasını bağlar.

**P4 derlemesinin başarılı olması C6'nın fabrikadaki firmware uyumunu kanıtlamaz.**
C6 yazılımını bu görev değiştirmez. Açılışta SDIO/C6 tanıma logu ve API başarıları
görülmeli. C6 reset/timeout döngüsü varsa P4 açılış logunu incele; rastgele C6
firmware'i veya flash-erase uygulama. Fabrika sürümü bilinmeden uyumluluk sözü
verilemez. Ana uygulama Wi-Fi API hatalarında IMU'yu durdurmaz; ESP-Hosted'in
alt seviye bellek/transport fatal hataları yine kartı yeniden başlatabilir.

Kaynaklar: [NANO kart bilgisi](https://docs.waveshare.com/ESP32-P4-NANO),
[kart şeması](https://files.waveshare.com/wiki/ESP32-P4-NANO/ESP32-P4-NANO-schematic.pdf),
[Waveshare P4/C6 sürüm rehberi](https://github.com/waveshareteam/ESP32-P4-Platform/blob/main/docs/P4_C6_HOSTED_WIFI.md),
[ESP-Hosted 2.12.7](https://components.espressif.com/components/espressif/esp_hosted/versions/2.12.7/readme).

## Dosyalar ve veri akışı

```text
IMU + attitude -> OrientationSystem -> dashboard.publish(kopya)
                                         |
                           kısa, kilitli son-örnek deposu
                                         |
                           HTTP /api/telemetry -> WebGL
```

- `include/drone_dashboard.hpp`: dışarıya açık `start()` / `publish()` API'si.
- `drone_dashboard.cpp`: ağ görevi, HTTP sunucu, statik dosyalar, JSON.
- `wifi_manager.cpp`: STA/AP seçimi, bağlantı denemesi ve AP yedeği.
- `network_config.cpp`: ESP'den bağımsız, test edilen TXT okuyucu.
- `web/index.html`, `styles.css`: Türkçe, responsive arayüz.
- `web/app.js`: canlı veri, geçerlilik, grafik, CSV ve **açıkça işaretli demo**.
- `web/math.js`: quaternion ve sahne dönüşüm matematiği.
- `web/renderer.js`: bağımsız WebGL model/ışık/kamera; harici CDN yok.

`main.cpp` paneli başlatır ve her `orientation.update()` sonrasında sonucu
`publish()` eder (başarısız sonucu da). publish ağ/HTTP beklemez; HTTP yalnız
son sonucu okur. IMU önceliği 5, web/ağ görevleri 3; filtre (~100 Hz) değişmez.
Tarayıcı en çok ~20 Hz tek sıralı HTTP isteği yapar; 3D görüntü bağımsız çizilir.
Tek nesne uygulama boyunca yaşar; `start()` ana görevden bir kez çağrılır.

JSON: `valid`, `sequence`, `timestamp_us`, `age_ms`, `quaternion:[w,x,y,z]`,
`angles_rad:[roll,pitch,yaw]`, `accel_g`, `gyro_rad_s`, sıcaklık ve kalibrasyon
bayrakları. 500 ms'den eski/geçersiz/sonlu olmayan sonuç canlı sayılmaz. Ağ
sessizleşince tarayıcı da ölçümü kapatır; son 3D duruş üzerinde bekleme bildirir.
Eski sayısal değerler yeni ölçüm gibi gösterilmez. Demo otomatik devreye girmez.

## Eksenler ve yorumlama

Quaternion gövdeden dünyaya dönüşümdür. Model +X burnu, +Y sol yanı, +Z üstü
temsil eder; sensörü buna göre monte et. WebGL için `[X,Y,Z] -> [X,Z,-Y]` sağ
elli görsel dönüşüm uygulanır. Sensör farklı monte edilirse önce sabit montaj
dönüşümü tanımlamak gerekir; panel otomatik montaj kalibrasyonu yapmaz.

Kamera düğmeleri yalnız bakış açısını değiştirir. Model quaternion ile döner;
Euler değerleri yalnız gösterim içindir. ±90° pitch civarında Euler sıçraması
gimbal-lock etkisidir. Yaw, açılıştaki referansa göre; manyetometre olmadığı
için gerçek kuzey değildir ve zamanla kayabilir.

CSV bu tarayıcı oturumundaki son 3.600 geçerli örneği tutar; demo/canlı geçişi
ve kart yeniden başlaması kayıtları temizler. Dondur yalnız görüntü/grafik/CSV
örnek toplamayı durdurur, sensörü durdurmaz. Pervaneler gerçek motor verisi
olmadığından statiktir.

## Donanım olmadan önizleme ve test

`python tools/preview_drone_web.py` → `http://127.0.0.1:8765`.
Önizleme gerçek veri uydurmaz; istersen **Demoyu dene** düğmesini kullan.

Host CMake testlerine Wi-Fi parser sınır/hata/null testleri eklendi.
`node tests/web_math_tests.mjs` body/world/view dönüşümü, 90° dönüşler,
normalizasyon ve quaternion'un kısa yol interpolasyonunu doğrular.
Donanım kabulü: soğuk açılış, AP'ye bağlanma, STA/DHCP, yanlış parola yedeği,
modem kopması, yeniden açılış ve sensörü tek tek +X/+Y/+Z döndürme.
Wi-Fi ile uçuş güvenliği/realtime garantisi sağlanmaz; pervaneleri sökerek dene.
