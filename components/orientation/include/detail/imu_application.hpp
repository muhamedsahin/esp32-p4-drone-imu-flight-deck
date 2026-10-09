#pragma once

#include "imu_text_store.hpp"
#include "mpu6050.hpp"

/** API'nin iç yardımcısı: mevcut dosya politikası ve konsol raporunu yönetir. */
class ImuApplication
{
  public:
    bool init();
    bool calibrate();
    void printCalibration() const;
    bool read(IMUData &data);
    void printMeasurement(const IMUData &data);

  private:
    static bool checkpoint(const imu_calibration::TextCalibration& state, void* context);
    MPU6050 imu_;
    imu_calibration::TextStore store_;
    imu_calibration::TextCalibration state_{};
    uint32_t failed_reads_ = 0;
    bool temperature_warning_ = false;
};
