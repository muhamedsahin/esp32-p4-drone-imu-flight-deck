#include "orientation_system.hpp"

bool OrientationSystem::init()
{
    if (sensor_ready_)
        return true; // Yinelenen init, kalibrasyonu veya quaternion'u sıfırlamaz.

    result_.valid = false;

    if (!imu_initialized_)
    {
        if (!imu_.init())
            return false;
        imu_initialized_ = true;
    }

    // Mevcut calibration.txt / NVS akışını aynen kullanırız.
    // Yarım kalan kalibrasyonda da son kaydedilmiş değerler görünür kalır.
    sensor_ready_ = imu_.calibrate();
    imu_.printCalibration();
    return sensor_ready_;
}

bool OrientationSystem::update()
{
    result_.valid = false;
    if (!sensor_ready_)
        return false;

    IMUData measurement{};
    if (!imu_.read(measurement))
        return false;

    return update(measurement);
}

bool OrientationSystem::update(const IMUData& data)
{
    result_.valid = false;

    // İlk uygun ölçümde gyro referansı ve quaternion başlangıcı otomatik yapılır.
    // Sonrasında dt, gyro entegrasyonu ve ivme düzeltmesini attitude yönetir.
    if (!attitude_.update(data))
        return false;

    // Yalnız başarılı işlemin ölçümü ve yönelimi birlikte yayınlanır.
    result_.measurement = data;
    result_.angles = attitude_.getAngles();
    result_.quaternion = attitude_.getQuaternion();
    result_.valid = true;
    return true;
}

OrientationData OrientationSystem::getOrientation() const
{
    return result_;
}

void OrientationSystem::print()
{
    if (!result_.valid)
        return;

    imu_.printMeasurement(result_.measurement);
    attitude_.printGyroAngles();
}
