#pragma once

#include "quaternion_type.hpp"
#include "imu_types.hpp"

/**
 * Gövde -> dünya yönelimini tutar (Hamilton quaternion, ZYX Euler sırası).
 * Dünya +Z yönü, düz duran sensörün ölçtüğü +1g yönüyle aynı kabul edilir.
 * Açılar radyan, gyro rad/s, ivme g ve dt saniye cinsindedir.
 */
class QuaternionUtils
{
public:
    // Uzun veri boşluklarında eski gyro hızını tüm boşluğa yaymayız.
    static constexpr float MAX_DT_SECONDS = 0.1f;

    bool fromEuler(const EulerAngles& data);
    /** Yalnız gyro entegrasyonu; ivmeölçer düzeltmesi uygulanmaz. */
    bool updateFromGyroscope(const IMUData& data, float dt);
    /** Gyro + güven ağırlıklı ivmeölçer düzeltmesi (oransal complementary filtre). */
    bool updateFromImu(const IMUData& data, float dt);
    const Quaternion& getQuaternion() const;
    EulerAngles getEulerAngles() const;

private:
    void applyAccelerometerCorrection(const IMUData& data,
                                      float& gx, float& gy, float& gz) const;
    bool integrateAngularVelocity(float gx, float gy, float gz, float dt);
    static bool normalize(Quaternion& value);

    Quaternion quaternion_;
};
