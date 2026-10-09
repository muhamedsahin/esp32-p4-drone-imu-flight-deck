# Attitude katmanı

`AttitudeEstimator` zaman damgasını ve başlangıcı yönetir. `QuaternionUtils`
yönelim matematiğini yapar. `main.cpp` yalnız ölçümü alır, `update(data)` çağırır
ve sonucu yazdırır. IMU kalibrasyonu bu katmandan bağımsızdır.

## Kullanım

Estimator ölçüm döngüsünün dışında oluşturulur; her örnekte aynı nesne kullanılır:

```cpp
AttitudeEstimator attitude;
// Her yeni IMU ölçümünde:
if (attitude.update(data))
    attitude.printGyroAngles();
```

İlk geçerli örnek ivmeölçerden roll/pitch başlatır, yaw=0 kabul eder ve zaman
referansını kaydeder. Başlangıçta kart sabit tutulmalıdır. Sonraki örneklerde:

1. Zaman farkı saniyeye çevrilir.
2. İvme vektörü normalize edilir ve 1g'ye yakınlığına göre güven ağırlığı bulunur.
3. Quaternion'dan beklenen +1g yönü gövde ekseninde hesaplanır.
4. Ölçülen yön × beklenen yön ile hata vektörü oluşturulur.
5. `gyro_corrected = gyro + Kp * trust * error` hesaplanır.
6. Quaternion türevi entegre edilir ve sonuç normalize edilir.

`updateGyro(data)` karşılaştırma için yalnız gyro kullanır. İki güncelleme
fonksiyonu aynı örnekte birlikte çağrılmaz. `getAngles()` Euler çıktısını,
`getQuaternion()` gövde -> dünya quaternion'unu verir. Başlangıç öncesinde
çıktı kullanılacaksa `isInitialized()` kontrol edilir.

## Birimler ve sınırlar

- Quaternion düzeni Hamilton `[w, x, y, z]`, Euler dönüşüm sırası ZYX'tir.
- Sensör ve gövde eksenleri aynı kabul edilir. Farklı montaj için önce IMU
  eksenleri gövde eksenlerine dönüştürülmelidir.
- Dünya +Z, düz sensörün ölçtüğü +1g / destek ivmesi yönüdür. İvmeölçer
  yerçekimi kuvvetini doğrudan değil, özgül kuvveti ölçer.
- İvme **g**, gyro **rad/s**, açılar **rad**, zaman damgası **mikrosaniye**.
- `Kp=2 /s`; 1g'den sapma 0.05g'ye kadar tam güven, 0.20g'de sıfır güven.
  Bu başlangıç ayarları `quaternion.cpp` başında açıklanmıştır.
- İvme kalibresiz, sonlu olmayan, sıfır veya güven aralığı dışındaysa yalnız
  gyro ile devam edilir. Gyro veya dt geçersizse güncelleme reddedilir.
- 100ms'den uzun ölçüm boşluğunda yönelim korunur ve zaman referansı yenilenir;
  kayıp aralıktaki dönüş geri kazanılamaz. Normal ölçüm hedefi yaklaşık 100 Hz'dir.
- `|A| ≈ 1g` hareketsizliği kanıtlamaz. Sürekli doğrusal ivme ve titreşim
  filtreyi yanıltabilir; bu sürüm uçuşta doğrulanmış bir kontrol sistemi değildir.
- Bu sürüm oransal complementary filtredir; integral bias öğrenmesi içermez.
  İvmeölçer mutlak yaw sağlayamaz. Yaw referansı ve yaw gyro biası kayabilir.
- Quaternion tekil değildir; Euler çıktısı ±90° pitch civarında tekildir.
  Tam ters yöndeki iki vektörün cross product'ı sıfırdır; başlangıç yöneliminin
  doğru kurulması bu yüzden önemlidir.

Oransal hata geri beslemesinin matematik referansı:
[Mahony filtresi — AHRS dokümantasyonu](https://ahrs.readthedocs.io/en/latest/filters/mahony.html).
