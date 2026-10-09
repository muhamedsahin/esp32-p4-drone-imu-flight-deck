#pragma once

#include "imu_calibration.hpp"

namespace imu_calibration
{
/** Kalıcı kayıt: format/sensör profili + katsayılar + CRC32. ESP-IDF bağımsızdır. */
struct AccelRecord
{
    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t profile = 0;
    std::array<float, 9> values{};
    uint32_t crc32 = 0;
};
static_assert(sizeof(float) == 4 && sizeof(AccelRecord) == 52, "Kayit formati degisti");
// Varsayılan MPU6050 profili eski kayıtlarla uyumludur. Sürücü gerçek kimliği iletir.
AccelRecord makeRecord(const AccelCalibration& calibration, uint8_t sensor_identity = 0x68);
bool decodeRecord(const AccelRecord& record, AccelCalibration& output, uint8_t sensor_identity = 0x68);
} // namespace imu_calibration
