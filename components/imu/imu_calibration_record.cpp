#include "imu_calibration_record.hpp"

#include <cstddef>

namespace imu_calibration
{
namespace
{
constexpr uint32_t MAGIC = 0x494D5543; // "IMUC"
constexpr uint32_t VERSION = 2; // v2: norm fit + yeni eğik konumlarda doğrulama.
// Bu profil değişirse eski count/g katsayıları otomatik kullanılamaz.
constexpr uint32_t profileFor(uint8_t identity)
{
    // WHO, gyro DLPF, ayrı accel DLPF (6050'da yok), örnekleme hızı.
    // ±2g/±250dps sabittir. MPU6050 profilini eski kayıtlar için koru.
    return identity == 0x68 ? 0x68030064U : identity == 0x70 ? 0x70030364U : 0U;
}
uint32_t checksum(const AccelRecord& record)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(&record);
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < offsetof(AccelRecord, crc32); ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}
}

AccelRecord makeRecord(const AccelCalibration& calibration, uint8_t sensor_identity)
{
    AccelRecord record{};
    record.magic = MAGIC;
    record.version = VERSION;
    record.profile = profileFor(sensor_identity);
    for (size_t axis = 0; axis < 3; ++axis)
    {
        record.values[axis] = static_cast<float>(calibration.bias_counts[axis]);
        record.values[axis + 3] = static_cast<float>(calibration.counts_per_g[axis]);
    }
    record.values[6] = static_cast<float>(calibration.reference_temperature_c);
    record.values[7] = static_cast<float>(calibration.validation_rms_g);
    record.values[8] = static_cast<float>(calibration.validation_max_g);
    record.crc32 = checksum(record);
    return record;
}

bool decodeRecord(const AccelRecord& record, AccelCalibration& output, uint8_t sensor_identity)
{
    const uint32_t expected_profile = profileFor(sensor_identity);
    if (expected_profile == 0 || record.magic != MAGIC || record.version != VERSION ||
        record.profile != expected_profile || record.crc32 != checksum(record)) return false;
    AccelCalibration candidate{};
    for (size_t axis = 0; axis < 3; ++axis)
    {
        candidate.bias_counts[axis] = record.values[axis];
        candidate.counts_per_g[axis] = record.values[axis + 3];
    }
    candidate.reference_temperature_c = record.values[6];
    candidate.validation_rms_g = record.values[7];
    candidate.validation_max_g = record.values[8];
    // Varsayılan kalite politikasını geçmeyen eski kayıt da kabul edilmez.
    if (!validModel(candidate) || candidate.validation_max_g > Config{}.max_accel_validation_g)
        return false;
    candidate.calibrated = true;
    output = candidate;
    return true;
}
} // namespace imu_calibration
