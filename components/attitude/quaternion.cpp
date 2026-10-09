#include "quaternion.hpp"
#include <algorithm>
#include <cmath>

namespace
{
// Başlangıç ayarları: gerçek sensörün titreşimi ve örnekleme hızıyla ayarlanır.
// Hata boyutsuzdur. Kp [1/s], gyroya eklenecek düzeltmeyi rad/s yapar.
constexpr float ACCEL_CORRECTION_KP = 2.0f;
constexpr float FULL_TRUST_DEVIATION_G = 0.05f;
constexpr float ZERO_TRUST_DEVIATION_G = 0.20f;

bool usableGyroscope(const IMUData& data, float dt)
{
    return data.valid && data.gyro_calibrated &&
           std::isfinite(data.gx) && std::isfinite(data.gy) && std::isfinite(data.gz) &&
           std::isfinite(dt) && dt > 0.0f && dt <= QuaternionUtils::MAX_DT_SECONDS;
}
} // namespace

// İlk yönelim: ivmeölçerden alınan roll/pitch ve başlangıç yaw değeri.
bool QuaternionUtils::fromEuler(const EulerAngles &data)
{
    if (!std::isfinite(data.roll_rad) || !std::isfinite(data.pitch_rad) ||
        !std::isfinite(data.yaw_rad))
        return false;

    const float cr = std::cos(data.roll_rad * 0.5f);
    const float sr = std::sin(data.roll_rad * 0.5f);
    const float cp = std::cos(data.pitch_rad * 0.5f);
    const float sp = std::sin(data.pitch_rad * 0.5f);
    const float cy = std::cos(data.yaw_rad * 0.5f);
    const float sy = std::sin(data.yaw_rad * 0.5f);

    // Başarısız işlem eski yönelimi bozmamalı; önce geçici sonucu hesapla.
    Quaternion initial{};
    initial.w = cr * cp * cy + sr * sp * sy;
    initial.x = sr * cp * cy - cr * sp * sy;
    initial.y = cr * sp * cy + sr * cp * sy;
    initial.z = cr * cp * sy - sr * sp * cy;
    if (!normalize(initial))
        return false;

    quaternion_ = initial;
    return true;
}

bool QuaternionUtils::updateFromGyroscope(
    const IMUData& data,
    const float dt)
{
    if (!usableGyroscope(data, dt))
        return false;

    return integrateAngularVelocity(data.gx, data.gy, data.gz, dt);
}

bool QuaternionUtils::updateFromImu(const IMUData& data, float dt)
{
    if (!usableGyroscope(data, dt))
        return false;

    // Sensör verisini değiştirmeyiz; yalnız entegrasyonda kullanılan hız düzeltilir.
    float gx = data.gx;
    float gy = data.gy;
    float gz = data.gz;
    applyAccelerometerCorrection(data, gx, gy, gz);

    // İvme güvenilir değilse hızlar ham gyro olarak kalır; takip devam eder.
    return integrateAngularVelocity(gx, gy, gz, dt);
}

void QuaternionUtils::applyAccelerometerCorrection(
    const IMUData& data, float& gx, float& gy, float& gz) const
{
    if (!data.accel_calibrated || !std::isfinite(data.ax) ||
        !std::isfinite(data.ay) || !std::isfinite(data.az))
        return;

    // 1. Ölçülen ivmenin büyüklüğü: IMUData ivmeyi g biriminde verir.
    const double norm_squared = static_cast<double>(data.ax) * data.ax +
                                static_cast<double>(data.ay) * data.ay +
                                static_cast<double>(data.az) * data.az;
    if (!std::isfinite(norm_squared) || norm_squared < 1e-12)
        return; // Serbest düşüşte veya sıfır veride yön bulunamaz.

    const double magnitude = std::sqrt(norm_squared);
    const double deviation = std::abs(magnitude - 1.0);
    if (deviation >= ZERO_TRUST_DEVIATION_G)
        return;

    // 2. |A|, 1g'den uzaklaştıkça düzeltmeyi yumuşakça azalt.
    // 0.05g sapmaya kadar ağırlık 1; 0.20g sapmada ağırlık 0 olur.
    // |A| ~= 1g, doğrusal ivme YOK demek değildir. Sürekli yatay ivme
    // ve titreşim için bu basit güven kontrolü tek başına yeterli değildir.
    const float trust = static_cast<float>(std::clamp(
        (ZERO_TRUST_DEVIATION_G - deviation) /
            (ZERO_TRUST_DEVIATION_G - FULL_TRUST_DEVIATION_G),
        0.0, 1.0));

    // 3. Büyüklüğü ayır, yalnız ölçülen birim ivme yönünü kullan.
    const float accel_x = static_cast<float>(data.ax / magnitude);
    const float accel_y = static_cast<float>(data.ay / magnitude);
    const float accel_z = static_cast<float>(data.az / magnitude);

    // 4. Dünya +Z birim vektörünü quaternion ile gövdeye geri döndür.
    // Sabit sensörün görmesini beklediğimiz +1g / destek ivmesi yönüdür.
    // İki vektör de GÖVDE eksenindedir; cross product aynı uzayda alınır.
    const Quaternion& q = quaternion_;
    const float gravity_x = 2.0f * (q.x * q.z - q.w * q.y);
    const float gravity_y = 2.0f * (q.y * q.z + q.w * q.x);
    const float gravity_z = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    // 5. Hata = ölçülen yön x tahmini yön (cross product).
    // Sıra, body->world quaternion ve +gyro işaretiyle birlikte önemlidir.
    const float error_x = accel_y * gravity_z - accel_z * gravity_y;
    const float error_y = accel_z * gravity_x - accel_x * gravity_z;
    const float error_z = accel_x * gravity_y - accel_y * gravity_x;

    // 6. Hatayı gyroya oransal geri besleme olarak ekle.
    // error_z gövde Z düzeltmesidir; mutlak dünya yaw bilgisi sağlamaz.
    // Şimdilik integral / online gyro bias öğrenmesi kullanılmıyor.
    const float gain = ACCEL_CORRECTION_KP * trust;
    gx += gain * error_x;
    gy += gain * error_y;
    gz += gain * error_z;
}

bool QuaternionUtils::integrateAngularVelocity(float gx, float gy, float gz, float dt)
{

    // Hesaplamanın tamamında aynı eski quaternion kullanılmalı.
    const Quaternion current = quaternion_;

    // Açısal hız quaternion'u:
    // omega = [0, gx, gy, gz] saf quaternion'dur; normalize edilmez.
    //
    // Quaternion türevi:
    // q_dot = 0.5 * current ⊗ omega
    Quaternion q_dot{};

    q_dot.w = -0.5f * (
        current.x * gx +
        current.y * gy +
        current.z * gz);

    q_dot.x = 0.5f * (
        current.w * gx +
        current.y * gz -
        current.z * gy);

    q_dot.y = 0.5f * (
        current.w * gy -
        current.x * gz +
        current.z * gx);

    q_dot.z = 0.5f * (
        current.w * gz +
        current.x * gy -
        current.y * gx);

    // Birinci derece entegrasyon; küçük dt için uygundur.
    // Büyük dönüş adımları sayısal hata oluşturabilir.
    Quaternion updated{};

    updated.w = current.w + q_dot.w * dt;
    updated.x = current.x + q_dot.x * dt;
    updated.y = current.y + q_dot.y * dt;
    updated.z = current.z + q_dot.z * dt;

    if (!normalize(updated))
        return false;

    // Bütün kontroller başarılı olduktan sonra gerçek durumu değiştir.
    quaternion_ = updated;

    return true;
}

bool QuaternionUtils::normalize(Quaternion& value)
{
    // Normalizasyon, kayan nokta hatalarının quaternion normunu bozmasını önler.
    const double norm_squared = static_cast<double>(value.w) * value.w +
                                static_cast<double>(value.x) * value.x +
                                static_cast<double>(value.y) * value.y +
                                static_cast<double>(value.z) * value.z;
    if (!std::isfinite(norm_squared) || norm_squared < 1e-12)
        return false;

    const double magnitude = std::sqrt(norm_squared);
    value.w = static_cast<float>(value.w / magnitude);
    value.x = static_cast<float>(value.x / magnitude);
    value.y = static_cast<float>(value.y / magnitude);
    value.z = static_cast<float>(value.z / magnitude);
    return true;
}

const Quaternion& QuaternionUtils::getQuaternion() const
{
    return quaternion_;
}

EulerAngles QuaternionUtils::getEulerAngles() const
{
    // Euler açıları yalnız çıktı içindir; ana yönelim durumu quaternion'dur.
    const Quaternion& q = quaternion_;
    EulerAngles result{};
    result.roll_rad = std::atan2(2.0f * (q.w * q.x + q.y * q.z),
                                1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    const float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
    result.pitch_rad = std::asin(std::clamp(sin_pitch, -1.0f, 1.0f));
    result.yaw_rad = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                               1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    return result;
}
