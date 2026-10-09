# MPU6050 / MPU6500: okunabilir kalibrasyon ve eksiklerden devam

## Güncel kullanım

Ana kayıt dosyası proje kökündeki `calibration.txt` dosyasıdır. Her satır
`anahtar = değer` biçimindedir; yorumlar `#` ile başlar. `null` eksik ölçümü
belirtir. Sayısal sıfır geçerli bir değerdir ve eksik sayılmaz.

```text
x = 546.472412109375, 16409.43359375
y = 113.77349090576172, 16437.134765625
z = 1128.4708251953125, 16747.64453125
gyro_x = null
```

X/Y/Z satırlarında ilk sayı bias [ham count], ikinci sayı hassasiyet [count/g]
değeridir. Pozitif/negatif yüz satırları ham X,Y,Z vektör ortalamalarıdır.
Sıcaklıklar °C, accel_quality RMS/maksimum hata [g], gyro_noise ve gyro_error
[dps] birimindedir. sensor=112 MPU6500, sensor=104 MPU6050 kimliğini belirtir.

Bu projedeki ilk ACC katsayıları karttan okunan, CRC'si doğrulanmış tamamlanmış
`accel_v1` kaydından aktarılmıştır. Eski logdaki yüz ortalamaları da dosyadadır.
Eski kayıtta tek tek eğik pencere hataları saklanmadığından validation_* alanları
başlangıçta null olabilir. Tamamlanmış modelin accel_quality kaydı mevcut olduğu
için bu alanlar geçmiş ölçüm uydurulmadan boş bırakılır; ACC tekrar ölçülmez.
Yeni kalibrasyonlarda eğik pencere hatalarının tamamı yazılır.

## Dosyayı değiştirme ve karta aktarma

ESP-IDF terminalinde proje klasöründen:

```powershell
idf.py -p COM17 build flash
python tools/calibration_monitor.py --port COM17
```

PC dosyası derleme sırasında firmware'e gömülür. ESP32 bilgisayarın diskindeki
dosyayı doğrudan açamaz; dosya değişikliklerinin karta geçmesi için yeniden
build/flash gerekir. Derleme dosyanın güncel içeriğini otomatik kullanır.

Kartta metnin son hali, kaynak dosya kimliği ve CRC32 `mpu6050_cal:text_v1`
NVS kaydına yazılır ve geri okunarak doğrulanır. Kaynak dosya değişmediyse
NVS'deki daha yeni ölçümler kullanılır. Sayısal kaynak içerik değişmişse yeni
dosya bir kez içeri alınır. Yalnız yorum/boşluk değiştirmek ilerlemeyi sıfırlamaz.
NVS açma, CRC veya parse hatasında kayıt otomatik silinmez.

`calibration_monitor.py` konsolu gösterir ve her kabul edilen ölçümün tam
raporunu PC'deki `calibration.txt` dosyasına atomik olarak yazar. Önceki sürüm
`calibration.previous.txt` olarak korunur. Yarım seri rapor dosyaya yazılmaz.
Ctrl+C monitörü kapatır. Tek rapor alıp çıkmak için:

```powershell
python tools/calibration_monitor.py --port COM17 --once
```

Aynı COM portunu aynı anda idf.py monitor ve bu araç açamaz. Standart
`idf.py -p COM17 monitor` değerleri gösterir ama PC dosyasını güncellemez.
Kart, monitör bağlı olmasa da her kabul edilen adımı NVS'ye kaydeder.
Sonradan monitör açıldığında en yeni kart kaydı dosyaya aktarılır.

## null ile seçici kalibrasyon

- `x = null`: eski modelin yalnız +X ve −X yüz ortalamaları sıfırlanır.
  Y/Z ölçümleri korunur. Üç katsayı birlikte yeniden fit edilir ve üç eğik
  kontrol yeniden yapılır. Yalnız X katsayısını diğerlerinden bağımsız
  değiştirmek norm modelinin matematiğine uygun değildir.
- `positive_x = null`: yalnız +X yüzü yeniden ölçülür; mevcut tamamlanmış
  modelin katsayıları ve kalite kontrolleri yeniden hesaplanır.
- `accel_quality = null`: model eksik kabul edilir; mevcut yüzler korunur.
  Modelin katsayıları veya yüzleri değiştirildiyse ilgili validation_* alanlarını
  da null yap. Aksi halde mevcut eğik kontrol sonuçları yeniden kullanılabilir.
- `gyro_x = null` (veya diğer gyro ekseni): üç gyro bias'ı aynı sabitlik
  penceresinden birlikte ölçülür; ACC yüzleri yeniden istenmez.
- Tüm katsayılar ve kalite alanları geçerliyse kalibrasyon atlanır.
  Tam sayı/nümerik görünmek yetmez: fiziksel sınırlar ve kalite denetlenir.

Her kabul edilen tek yüz ve tek eğik kontrol sonrasında kayıt yapılır.
Yarım oturumda tekrar başlatma tamamlanan yüzleri ve kontrolleri atlar.
Bozuk dosya, tekrar eden/bilinmeyen anahtar, yanlış vektör uzunluğu, NaN/Inf,
yanlış sensör profili veya bozuk NVS kaydı sessizce kullanılmaz.

Kalibrasyon değiştirildiğinde yeni aday bütün eğik kontroller bitene kadar
aktif olmaz. Kaydedilmiş tamamlanmış ACC için yüz ölçümü gerekmez.
Gyro da artık dosyadan yeniden kullanılabilir. Açılıştaki 20 s ısınma beklemesi
kalibrasyon değildir. Kayıtlı gyro referansından sıcaklık 5 °C'den fazla
uzaktaysa bias otomatik yenilenir; kayıtlı bias sıcaklık telafisi modeli değildir.
Dosya üzerinden başlatılmış yarım ACC oturumunda referans sıcaklığı çevresindeki
3 °C sınırı korunur. Motorlar kapalıyken sabit destek üzerinde ölçüm yap.

## Konsol ve mimari

Açılışta SON KALIBRASYON altında CALIBRATION_BEGIN/END arasında en son kayıt,
tüm katsayılar, sıcaklıklar, kalite ve null alanlar yazılır. Yeni ölçümler
CALIBRATION_UPDATE_BEGIN/END arasında da yayınlanır. Kalibrasyon durursa bile
o ana kadar saklanan değerler raporlanır.

- `main/main.cpp`: görev oluştur, init, kalibre et/yükle, raporla, oku/yazdır.
- `main/imu_application.*`: uygulama politikası, kaynak dosya, konsol sunumu.
- `components/imu/imu_calibration_text.cpp`: ESP-IDF bağımsız dosya parser'ı/modeli.
- `components/imu/imu_text_store.cpp`: metin NVS kaydı, CRC, kaynak sürümü, readback.
- `components/imu/mpu6050_text_calibration.cpp`: seçici ve kaldığı yerden devam eden ölçüm.
- `mpu6050.cpp`: register/I2C/sensör kimliği ve fiziksel birim dönüşümü.
- `imu_calibration.cpp`: istatistik, kararlılık ve norm fit matematiği.

Eski binary `accel_v1` ve `pending_v1` kayıtları geçişte silinmez.
Eski API'ler sürücü uyumluluğu için korunur. Güncel main akışı eski
MPU6050_RECOVER_LAST_LOG/FORCE seçeneklerini kullanmaz; null düzenlemeleri
`calibration.txt` üzerinden yapılır. Yeni fiziksel modül takıldığında eski
değerleri kullanma; sensör kimliğini doğru seçip tüm katsayı/yüz/kalite
alanlarını null yaparak yeni oturum başlat.

## Ölçüm modelinin kapsamı

±2 g, ±250 dps, 100 Hz ve DLPF=3 profili kullanılır.
MPU6500 için ayrıca ACCEL_CONFIG2=3 ayarlanır. Her register geri okunur.
WHO_AM_I üç kez reset öncesi ve üç kez reset sonrası kararlı okunmalıdır.
PCB'de MPU6050 yazması gerçek çip kimliğini belirlemez: bu kartta 0x70 MPU6500
okunmuştur. I2C adresi AD0 düşükken 0x68, SDA=GPIO7 ve SCL=GPIO8'dir.

Norm fit aşağıdaki denklemi altı yaklaşık yön ortalamasında çözer:

```text
((raw_x-bias_x)/scale_x)^2 + ((raw_y-bias_y)/scale_y)^2 +
((raw_z-bias_z)/scale_z)^2 = 1
```

Tam 90° hizalama gerekmez. Normalize Newton/backtracking çözümü pivot ve
fiziksel sınır kontrollerini korur. Fit'te kullanılmayan XY/XZ/YZ eğik
duruşlarının her birinde iki bağımsız sabit pencere ölçülür. İki ilgili ham
eksenin mutlak büyüklüğü 0.30 g'den büyük, üçüncüsü 0.30 g'den küçük olmalıdır.
Düzeltilmiş normun 1 g'den uzaklığı en fazla 0.035 g olabilir.
Yüz ortalamaları aynı fit verisi olduğundan bağımsız doğrulama diye sayılmaz;
güncel dosya akışında RMS altı eğik pencereden hesaplanır.

Bu üç bias/üç hassasiyetli diagonal modeldir. Tam 3×3 çapraz eksen matrisi,
sensör-kart hizası ve sıcaklık telafisi öğrenilmez. |ACC|≈1 g tek başına
mutlak yön doğruluğu veya uçuşa hazır kontrol sistemi kanıtı değildir.

## Bilgisayar testleri

```powershell
cmake -S tests -B build/host-tests
cmake --build build/host-tests --config Debug
ctest --test-dir build/host-tests -C Debug --output-on-failure
```

Testler gerçek matematik ve sürücü akışını simüle I2C/FreeRTOS/NVS ile çalıştırır.
Metin roundtrip, null/sıfır ayrımı, tam dosyada atlama, X'e özel yüz yenileme,
tek tek checkpoint, reset sonrası devam, CRC/yazma hatası ve gyro sıcaklık
yenilemesi ayrıca denetlenir. Donanımda dosyaya aktarma ve ikinci açılışta
kalibrasyonun atlanması kontrol edilmelidir.
