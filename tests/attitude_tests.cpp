#include "attitude_estimator.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

namespace
{
constexpr double PI = 3.14159265358979323846;
int checks = 0;
void check(bool condition, const char *message)
{
    ++checks;
    if (!condition)
    {
        std::printf("FAIL: %s\n", message);
        std::exit(1);
    }
}
bool close(double value, double expected, double tolerance = 1e-6)
{
    return std::abs(value - expected) <= tolerance;
}
IMUData sample(float ax, float ay, float az)
{
    IMUData data{};
    data.ax = ax;
    data.ay = ay;
    data.az = az;
    data.valid = true;
    data.accel_calibrated = true;
    return data;
}

IMUData readySample(float ax = 0, float ay = 0, float az = 1)
{
    IMUData data = sample(ax, ay, az);
    data.gyro_calibrated = true;
    data.gyro_temperature_valid = true;
    data.timestamp_us = 1'000'000;
    return data;
}

bool sameQuaternion(const Quaternion& a, const Quaternion& b, double tolerance = 1e-6)
{
    return close(a.w, b.w, tolerance) && close(a.x, b.x, tolerance) &&
           close(a.y, b.y, tolerance) && close(a.z, b.z, tolerance);
}

bool unitQuaternion(const Quaternion& q)
{
    return close(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z, 1, 2e-6);
}

void testQuaternionFusion()
{
    // Euler -> quaternion -> Euler dönüşümünde eksen / işaret düzeni korunmalı.
    QuaternionUtils orientation;
    check(orientation.fromEuler({0.4f, -0.3f, 0.8f}), "initialize mixed orientation");
    EulerAngles angles = orientation.getEulerAngles();
    check(close(angles.roll_rad, 0.4) && close(angles.pitch_rad, -0.3) &&
          close(angles.yaw_rad, 0.8), "Euler conversion uses consistent ZYX convention");
    const Quaternion before = orientation.getQuaternion();
    check(!orientation.fromEuler({std::numeric_limits<float>::quiet_NaN(), 0, 0}) &&
          sameQuaternion(before, orientation.getQuaternion()),
          "invalid initialization preserves last orientation");

    // 90 derece roll'de body-Z dönüşü, dünya pitch açısını değiştirir.
    // Euler açılarını ayrı ayrı toplayan eski yöntem bu testi geçemez.
    check(orientation.fromEuler({static_cast<float>(PI / 2), 0, 0}),
          "initialize rolled body for coupled gyro rotation");
    IMUData data = readySample();
    data.gz = 0.5f;
    for (int i = 0; i < 100; ++i)
        check(orientation.updateFromGyroscope(data, 0.01f), "integrate body-Z rotation");
    angles = orientation.getEulerAngles();
    check(close(angles.roll_rad, PI / 2, 1e-5) && close(angles.pitch_rad, -0.5, 1e-5) &&
          close(angles.yaw_rad, 0, 1e-5), "body rotation is correctly coupled in world frame");
    check(unitQuaternion(orientation.getQuaternion()), "gyro integration remains normalized");

    // Sabit, eğik sensörde düzeltme her iki işarette de doğru yöne çekmeli.
    // İvme yönü bağımsız Euler geometrisinden üretilir (yaw'dan bağımsızdır).
    for (const EulerAngles target : {EulerAngles{0.5f, 0.3f, 0},
                                    EulerAngles{-0.5f, -0.3f, 0}})
    {
        QuaternionUtils fused;
        data = readySample(-std::sin(target.pitch_rad),
                           std::sin(target.roll_rad) * std::cos(target.pitch_rad),
                           std::cos(target.roll_rad) * std::cos(target.pitch_rad));
        const IMUData original = data;
        for (int i = 0; i < 1000; ++i)
            check(fused.updateFromImu(data, 0.01f), "stationary tilt correction succeeds");
        angles = fused.getEulerAngles();
        check(close(angles.roll_rad, target.roll_rad, 1e-4) &&
              close(angles.pitch_rad, target.pitch_rad, 1e-4),
              "cross-product correction converges to measured tilt");
        check(data.gx == original.gx && data.gy == original.gy && data.gz == original.gz,
              "feedback never mutates caller IMU data");
        check(unitQuaternion(fused.getQuaternion()), "fusion remains normalized");
    }

    // Aynı başlangıç yönelimi ve aynı ölçülen yön: sahte düzeltme olmamalı.
    check(orientation.fromEuler({0.4f, -0.3f, 1.2f}), "initialize arbitrary heading");
    data = readySample(std::sin(0.3f), std::sin(0.4f) * std::cos(0.3f),
                       std::cos(0.4f) * std::cos(0.3f));
    const Quaternion aligned = orientation.getQuaternion();
    check(orientation.updateFromImu(data, 0.01f) &&
          sameQuaternion(aligned, orientation.getQuaternion()),
          "matched gravity generates no correction even with nonzero yaw");

    // İvme geçersiz olduğunda yönelim gyro ile ilerlemeye devam etmeli.
    for (int mode = 0; mode < 6; ++mode)
    {
        QuaternionUtils pure_gyro;
        QuaternionUtils fused;
        pure_gyro.fromEuler({0.4f, -0.2f, 0.3f});
        fused.fromEuler({0.4f, -0.2f, 0.3f});
        data = readySample();
        data.gx = 0.1f;
        data.gy = -0.2f;
        data.gz = 0.3f;
        if (mode == 0) data.accel_calibrated = false;
        if (mode == 1) data.az = 0; // Serbest düşüş.
        if (mode == 2) data.az = 2; // Belirgin doğrusal ivme.
        if (mode == 3) data.ax = std::numeric_limits<float>::quiet_NaN();
        if (mode == 4) data.ax = std::numeric_limits<float>::infinity();
        if (mode == 5) data.ax = std::numeric_limits<float>::max();
        check(pure_gyro.updateFromGyroscope(data, 0.01f) && fused.updateFromImu(data, 0.01f) &&
              sameQuaternion(pure_gyro.getQuaternion(), fused.getQuaternion()),
              "untrusted acceleration falls back to unchanged gyro integration");
    }

    // 1.1g ölçümde düzeltme uygulanır, fakat 1g'deki kadar güçlü değildir.
    QuaternionUtils full_trust;
    QuaternionUtils partial_trust;
    full_trust.fromEuler({0.4f, 0, 0});
    partial_trust.fromEuler({0.4f, 0, 0});
    data = readySample();
    check(full_trust.updateFromImu(data, 0.01f), "full-trust correction succeeds");
    data.az = 1.1f;
    check(partial_trust.updateFromImu(data, 0.01f), "partial-trust correction succeeds");
    const float full_roll = full_trust.getEulerAngles().roll_rad;
    const float partial_roll = partial_trust.getEulerAngles().roll_rad;
    check(full_roll < partial_roll && partial_roll < 0.4f,
          "acceleration trust reduces feedback gradually");

    // İvmeölçer mutlak yaw referansı sağlayamaz.
    orientation.fromEuler({0, 0, 1.0f});
    data = readySample();
    for (int i = 0; i < 100; ++i)
        check(orientation.updateFromImu(data, 0.01f), "flat yaw hold succeeds");
    check(close(orientation.getEulerAngles().yaw_rad, 1.0),
          "accelerometer does not reset an arbitrary yaw reference");
    data.gz = 0.05f;
    for (int i = 0; i < 100; ++i)
        check(orientation.updateFromImu(data, 0.01f), "yaw-rate update succeeds");
    check(close(orientation.getEulerAngles().yaw_rad, 1.05, 1e-5),
          "yaw bias remains unobservable without heading reference");

    const Quaternion saved = orientation.getQuaternion();
    check(!orientation.updateFromImu(data, 0) && !orientation.updateFromImu(data, -0.01f) &&
          !orientation.updateFromImu(data, 0.2f) &&
          !orientation.updateFromImu(data, std::numeric_limits<float>::quiet_NaN()),
          "invalid time steps rejected");
    data.gx = std::numeric_limits<float>::quiet_NaN();
    check(!orientation.updateFromImu(data, 0.01f), "nonfinite gyro rejected");
    data = readySample();
    data.valid = false;
    check(!orientation.updateFromImu(data, 0.01f), "invalid IMU rejected by fusion");
    data.valid = true;
    data.gyro_calibrated = false;
    check(!orientation.updateFromImu(data, 0.01f), "uncalibrated gyro rejected by fusion");
    check(sameQuaternion(saved, orientation.getQuaternion()),
          "rejected samples preserve orientation");
}

void testEstimatorWorkflow()
{
    AttitudeEstimator estimator;
    IMUData data = readySample();
    data.valid = false;
    check(!estimator.update(data) && !estimator.isInitialized(),
          "invalid first sample cannot initialize estimator");
    data.valid = true;
    data.az = 2;
    check(!estimator.update(data), "initialization rejects non-gravity magnitude");
    data.az = 1;
    check(estimator.update(data) && estimator.isInitialized(),
          "first valid sample initializes once");
    data.gy = 1;
    data.timestamp_us += 10'000;
    check(estimator.updateGyro(data) && close(estimator.getAngles().pitch_rad, 0.01, 1e-5),
          "timestamp interval integrates gyro in seconds");
    const Quaternion saved = estimator.getQuaternion();
    check(estimator.gyroinitialize(data) && sameQuaternion(saved, estimator.getQuaternion()),
          "repeated initialization never resets orientation");
    check(!estimator.update(data), "duplicate timestamp rejected");
    data.timestamp_us -= 1;
    check(!estimator.update(data), "backward timestamp rejected");
    data.timestamp_us += 1'000'001;
    check(!estimator.update(data) && sameQuaternion(saved, estimator.getQuaternion()),
          "large data gap resynchronizes time without angle jump");
    data.timestamp_us += 10'000;
    data.gy = 0;
    check(estimator.update(data) && estimator.getAngles().pitch_rad < 0.01f,
          "fusion resumes after gap and corrects tilt");

    // 90 derece pitch civarında Euler çıktı sonlu kalmalı; quaternion tekil değildir.
    QuaternionUtils vertical;
    check(vertical.fromEuler({0, static_cast<float>(PI / 2), 0}),
          "vertical orientation initializes");
    const EulerAngles angles = vertical.getEulerAngles();
    check(std::isfinite(angles.roll_rad) && std::isfinite(angles.pitch_rad) &&
          std::isfinite(angles.yaw_rad), "Euler output clamps asin input near singularity");
}
} // namespace

int main()
{
    AttitudeEstimator estimator;
    auto angles = estimator.calculateAccelAngles(sample(0, 0, 1));
    check(angles.valid && close(angles.roll_rad, 0) && close(angles.pitch_rad, 0),
          "flat sensor is zero roll and pitch");

    angles = estimator.calculateAccelAngles(sample(0, 0.70710678f, 0.70710678f));
    check(angles.valid && close(angles.roll_rad, PI / 4), "positive 45 degree roll");

    angles = estimator.calculateAccelAngles(sample(-0.5f, 0, 0.8660254f));
    check(angles.valid && close(angles.pitch_rad, PI / 6), "positive 30 degree pitch");

    IMUData invalid{};
    check(!estimator.calculateAccelAngles(invalid).valid, "invalid IMU sample rejected");
    invalid.valid = invalid.accel_calibrated = true;
    check(!estimator.calculateAccelAngles(invalid).valid, "zero acceleration rejected");
    invalid.ax = std::numeric_limits<float>::quiet_NaN();
    check(!estimator.calculateAccelAngles(invalid).valid, "non-finite acceleration rejected");

    testQuaternionFusion();
    testEstimatorWorkflow();
    std::printf("%d attitude and quaternion fusion checks passed\n", checks);
}
