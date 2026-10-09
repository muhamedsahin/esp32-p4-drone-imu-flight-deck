#include "attitude_estimator.hpp"
#include <stdio.h>
#include <cmath>

AccelAngles AttitudeEstimator::calculateAccelAngles(const IMUData &data) const
{
    AccelAngles result{};
    if (!data.valid || !data.accel_calibrated || !std::isfinite(data.ax) || !std::isfinite(data.ay) ||
        !std::isfinite(data.az))
        return result;

    const double horizontal_squared =
        static_cast<double>(data.ay) * data.ay + static_cast<double>(data.az) * data.az;
    const double norm_squared = horizontal_squared + static_cast<double>(data.ax) * data.ax;
    if (norm_squared < 1e-12)
        return result; // atan2(0, 0) fiziksel bir yön değildir.

    result.roll_rad = static_cast<float>(std::atan2(data.ay, data.az));
    result.pitch_rad = static_cast<float>(std::atan2(-data.ax, std::sqrt(horizontal_squared)));
    result.valid = true;

    return result;
}

bool AttitudeEstimator::gyroinitialize(const IMUData &data)
{
    if (initialized_)
        return true;

    if (!data.gyro_calibrated || data.timestamp_us <= 0 ||
        !std::isfinite(data.gx) || !std::isfinite(data.gy) || !std::isfinite(data.gz))
        return false;

    const AccelAngles accel_angles = calculateAccelAngles(data);
    if (!accel_angles.valid)
        return false;

    // Başlangıçta da belirgin doğrusal ivme / serbest düşüş verisini reddet.
    // Kartı sabit tutmak gerekir; yalnız 1g kontrolü hareketi kesin ayıramaz.
    const double magnitude = std::sqrt(static_cast<double>(data.ax) * data.ax +
                                       static_cast<double>(data.ay) * data.ay +
                                       static_cast<double>(data.az) * data.az);
    if (std::abs(magnitude - 1.0) >= 0.20)
        return false;

    const EulerAngles initial{accel_angles.roll_rad, accel_angles.pitch_rad, 0.0f};
    if (!orientation_.fromEuler(initial))
        return false;

    // İlk örnekte dt hesaplanmaz; sonraki ölçüme zaman referansı bırakılır.
    last_timestamp_us_ = data.timestamp_us;
    initialized_ = true;
    return true;
}

bool AttitudeEstimator::update(const IMUData& data)
{
    return updateOrientation(data, true);
}

bool AttitudeEstimator::updateGyro(const IMUData& data)
{
    return updateOrientation(data, false);
}

bool AttitudeEstimator::updateOrientation(const IMUData& data, bool use_accelerometer)
{
    // update() ilk geçerli örnekte başlangıcı da yapar; aynı örneği entegre etmez.
    if (!initialized_)
        return gyroinitialize(data);

    if (!data.valid || !data.gyro_calibrated || !std::isfinite(data.gx) ||
        !std::isfinite(data.gy) || !std::isfinite(data.gz) ||
        data.timestamp_us <= last_timestamp_us_)
        return false;

    const float dt = static_cast<float>(data.timestamp_us - last_timestamp_us_) * 1e-6f;
    if (dt > QuaternionUtils::MAX_DT_SECONDS)
    {
        // Veri boşluğu: açıya büyük bir sıçrama eklemeden zaman referansını yenile.
        // Boşluk boyunca gerçek hareket bilinmez; sonraki örnekten takibe devam edilir.
        last_timestamp_us_ = data.timestamp_us;
        return false;
    }

    const bool updated = use_accelerometer
        ? orientation_.updateFromImu(data, dt)
        : orientation_.updateFromGyroscope(data, dt);
    if (updated)
        last_timestamp_us_ = data.timestamp_us;
    return updated;
}

bool AttitudeEstimator::isInitialized() const
{
    return initialized_;
}

EulerAngles AttitudeEstimator::getAngles() const
{
    return orientation_.getEulerAngles();
}

const Quaternion& AttitudeEstimator::getQuaternion() const
{
    return orientation_.getQuaternion();
}

void AttitudeEstimator::printGyroAngles() const
{
    if (!initialized_)
        return;

    const EulerAngles angles = getAngles();
    printf("Attitude: Roll: %.4f rad, Pitch: %.4f rad, Yaw: %.4f rad\n",
           angles.roll_rad, angles.pitch_rad, angles.yaw_rad);
}
