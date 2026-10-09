# Orientation API

Bu katman IMU okumasını, mevcut kalibrasyon akışını ve attitude filtresini
birlikte yönetir. `imu` ve `attitude` bileşenlerinin koduna müdahale etmez.
`imu_application.cpp`, eskiden `main` altında bulunan kalibrasyon ve rapor
yöneticisidir; aynı dosya/NVS işleyişi burada kullanılır.

## Sensörden otomatik okuma

Çağıran bileşenin CMake bağımlılıklarına `REQUIRES orientation` eklenir.
Kullanıcı kodunda yalnız `orientation_system.hpp` eklemek yeterlidir:

```cpp
#include "orientation_system.hpp"

OrientationSystem orientation; // Döngü dışında: durum örnekler arasında korunur.

// Bir kez çağır. init() kayıtları yükler, yalnız eksikleri kalibre eder.
if (!orientation.init())
    return;

// Her yeni ölçüm zamanı geldiğinde çağır (yaklaşık 10ms aralıklarla):
if (orientation.update())
{
    const OrientationData result = orientation.getOrientation();
    const float roll = result.angles.roll_rad;
    const float pitch = result.angles.pitch_rad;
    const float yaw = result.angles.yaw_rad;
    // result.quaternion: gövde -> dünya yönelimi.
    // result.measurement: aynı güncellemeye ait kalibre edilmiş IMU ölçümü.
    // Sonucu kullan veya orientation.print() ile konsola yazdır.
}
```

`init()` sensörü okumaya hazırlar. İlk geçerli ve başlangıca uygun ölçümde
quaternion ve gyro referansı kendiliğinden kurulur; ayrıca gyro/quaternion
başlatma çağrısı yapılmaz. Kart başlangıçta sabit tutulmalıdır.

## Dışarıdan ölçüm verme

Sensörü başka bir kod okuyorsa aynı API veriyi de kabul eder:

```cpp
OrientationSystem orientation;
IMUData data = /* başka okuyucudan kalibre edilmiş ölçüm */;
if (orientation.update(data))
{
    const auto result = orientation.getOrientation();
    // result.angles.roll_rad, pitch_rad, yaw_rad
}
```

Bu kullanımda `init()` gerekmez; API sensöre erişmez. `IMUData.valid`, ilgili
kalibrasyon bayrakları ve artan `timestamp_us` değerleri doğru olmalıdır.
Tek nesneye farklı saat veya gövde eksenleri kullanan kaynaklar karıştırılmaz.

## Sonuç sözleşmesi

- `getOrientation()` hesaplama/okuma yapmaz; sonucun bir kopyasını döndürür.
- İlk başarılı güncelleme öncesinde `valid=false` olur.
- Başarısız okumada/güncellemede `valid=false` olur. Önceki açı, quaternion ve
  ölçüm korunur; `measurement.timestamp_us` bu eski başarılı örneğin zamanıdır.
- `print()` yalnız son güncelleme geçerliyse ölçümü ve açıları yazdırır.
- Uzun veri boşluğu, dt kontrolleri ve ivme güveni mevcut attitude katmanında
  yönetilir. Bu API filtreyi veya kalibrasyon matematiğini kopyalamaz.
- Nesne tek görevden kullanılır; birden fazla nesne aynı I2C bus'a sahip olmamalıdır.

Çıktı **yönelimdir**; XYZ konumu değildir. Roll/pitch yerçekimi referansına göre,
yaw başlangıç yönüne göre verilir. Manyetometre olmadan mutlak dünya heading'i
bilinmez ve yaw kayabilir. Açılar radyan, ivme g, gyro rad/s cinsindedir.
