#pragma once

#include "attitude_estimator.hpp"
#include "detail/imu_application.hpp"

/**
 * Bir başarılı güncellemenin çıktısı.
 * Açılar radyandır; quaternion gövdeden dünya referansına dönüşümü temsil eder.
 * Yaw başlangıçta sıfırdır ve manyetometre olmadığı için zamanla kayabilir.
 */
struct OrientationData
{
    EulerAngles angles{};
    Quaternion quaternion{};
    IMUData measurement{}; // Bu yönelimi üreten kalibre edilmiş IMU ölçümü.
    bool valid = false;    // Son update() başarılı mı? İlk örnek öncesinde false.
};

/**
 * IMU ve attitude katmanlarını tek API arkasında birleştirir.
 * Tek görevden kullanılır. Nesne, ölçüm döngüsünün dışında oluşturulmalıdır.
 * Sahip olduğu sensörün I2C kaynakları nesne yok olduğunda serbest bırakılır.
 */
class OrientationSystem
{
public:
    /**
     * Sensörü başlatır; mevcut kalibrasyonu yükler, yalnız eksikleri tamamlar.
     * Kalibrasyon kullanıcıdan sensörü çevirmesini isteyebilir ve zaman alabilir.
     * Başarı, sensörün okumaya hazır olduğunu belirtir. İlk quaternion, ilk
     * uygun update() ölçümünde otomatik kurulur; başlangıçta sensörü sabit tut.
     * Tekrar çağrıldığında başarılı hazırlığı veya yönelimi sıfırlamaz.
     */
    bool init();

    /** init() sonrasında sensörden bir ölçüm okur ve yönelimi günceller. */
    bool update();

    /**
     * Dışarıdan verilmiş, kalibre edilmiş IMU ölçümünü işler.
     * Bu yol I2C okuması yapmaz ve init() gerektirmez; ölçümün kalibrasyon ve
     * zaman kontrollerini attitude katmanı yapar. Tek nesnede bir ölçüm kaynağı
     * kullan: farklı saatlerle gelen sensör verilerini karıştırma.
     */
    bool update(const IMUData& data);

    /**
     * Son sonucu kopya olarak verir; çağıran API'nin iç durumunu değiştiremez.
     * Başarısız update() sonrasında önceki değerler korunur ama valid=false olur.
     * Böylece eski açı, yeni ve geçerli bir ölçüm sanılmaz.
     */
    OrientationData getOrientation() const;

    /** Son geçerli ölçümü ve açıları konsola yazdırır; hesaplama yapmaz. */
    void print();

private:
    ImuApplication imu_;
    AttitudeEstimator attitude_;
    OrientationData result_{};
    bool imu_initialized_ = false;
    bool sensor_ready_ = false;
};
