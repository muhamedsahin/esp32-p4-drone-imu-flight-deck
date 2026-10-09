#pragma once

#include "imu_calibration.hpp"
#include <string>

namespace imu_calibration
{
/** null, sıfırdan farklıdır: present=false yalnız eksik ölçümü belirtir. */
template <size_t N> struct TextValue
{
    std::array<double, N> value{};
    bool present = false;
};

/** Okunabilir dosya modeli. Sürücü, NVS veya uygulama ayrıntısı içermez. */
struct TextCalibration
{
    uint8_t identity = 0;
    std::array<TextValue<2>, 3> axes{};  // Her eksen: bias[count], hassasiyet[count/g].
    std::array<TextValue<3>, 6> faces{}; // +X,-X,+Y,-Y,+Z,-Z ham vektör ortalamaları.
    TextValue<1> accel_temperature{};
    TextValue<2> accel_quality{};        // RMS[g], maksimum hata[g]; null = henüz doğrulanmadı.
    std::array<TextValue<2>, 3> tilts{}; // XY,XZ,YZ: iki bağımsız pencerenin hataları.
    std::array<TextValue<1>, 3> gyro_axes{};
    TextValue<1> gyro_temperature{};
    TextValue<3> gyro_noise{};
    TextValue<1> gyro_error{};
};

bool parseCalibrationText(const std::string &text, TextCalibration &output, std::string &error);
std::string calibrationText(const TextCalibration &state);
bool completedAccelerometer(const TextCalibration &state, AccelCalibration &model);
bool completedGyroscope(const TextCalibration &state, GyroCalibration &model);
// Bir eksen null yapılınca yalnız o eksenin iki yüzünü ve modelin kalite kanıtını sıfırla.
void applyCalibrationEdits(TextCalibration &state);
uint32_t textChecksum(const std::string &text);
} // namespace imu_calibration
