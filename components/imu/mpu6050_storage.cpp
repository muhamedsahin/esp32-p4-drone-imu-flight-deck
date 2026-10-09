#include "mpu6050.hpp"
#include "imu_calibration_record.hpp"

#include <cstring>
#include <cmath>
#include <cstddef>
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace
{
constexpr const char* TAG = "IMU_NVS";
constexpr const char* NAMESPACE = "mpu6050_cal";
constexpr const char* KEY = "accel_v1";
constexpr const char* PENDING_KEY = "pending_v1";
uint32_t progressChecksum(const void* object, size_t length)
{
    const auto* bytes = static_cast<const uint8_t*>(object);
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}

bool initializeStorage()
{
    const esp_err_t err = nvs_flash_init();
    if (err != ESP_OK)
    {
        // NVS başka bileşenlerin verilerini de içerebilir; hatada komple silinmez.
        ESP_LOGE(TAG, "NVS acilamadi: %s. Otomatik silme yapilmadi.", esp_err_to_name(err));
        return false;
    }
    return true;
}

} // namespace

bool MPU6050::loadPendingCalibration(PendingCalibration& pending)
{
    using namespace imu_calibration;
    if (!initializeStorage()) { report_.status = Status::StorageError; return false; }
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        report_.status = err == ESP_ERR_NVS_NOT_FOUND ? Status::NoStoredCalibration : Status::StorageError;
        return false;
    }
    PendingCalibration record{};
    size_t length = sizeof(record);
    err = nvs_get_blob(handle, PENDING_KEY, &record, &length);
    nvs_close(handle);
    if (err != ESP_OK)
    {
        report_.status = err == ESP_ERR_NVS_NOT_FOUND ? Status::NoStoredCalibration : Status::StorageError;
        return false;
    }
    if (length != sizeof(record) || record.version != 1 || record.identity != sensor_identity_ ||
        record.next_pose > 3 || record.imported_from_log > 1 ||
        record.validation_groups != (record.imported_from_log ? 0U : 6U) + 2 * record.next_pose ||
        !std::isfinite(record.squared_error) || record.squared_error < 0 ||
        !std::isfinite(record.temperature_min) || !std::isfinite(record.temperature_max) ||
        record.temperature_min < -40 || record.temperature_max > 85 || record.temperature_min > record.temperature_max ||
        !validModel(record.model) || record.model.validation_max_g > Config{}.max_accel_validation_g ||
        record.crc32 != progressChecksum(&record, offsetof(PendingCalibration, crc32)))
    {
        report_.status = Status::InvalidModel;
        ESP_LOGE(TAG, "Yarim kalibrasyon kaydi bozuk veya sensor modeli farkli; kullanilmadi.");
        return false;
    }
    std::memcpy(&pending, &record, sizeof(pending)); // CRC kapsamındaki padding byte'larını da koru.
    report_.status = Status::Ok;
    return true;
}

bool MPU6050::savePendingCalibration(PendingCalibration& pending)
{
    using namespace imu_calibration;
    if (!initialized_ || !initializeStorage()) { report_.status = Status::StorageError; return false; }
    pending.identity = sensor_identity_;
    pending.crc32 = progressChecksum(&pending, offsetof(PendingCalibration, crc32));
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_set_blob(handle, PENDING_KEY, &pending, sizeof(pending));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
    }
    if (err != ESP_OK) { report_.status = Status::StorageError; return false; }
    PendingCalibration check{};
    if (!loadPendingCalibration(check) || std::memcmp(&check, &pending, sizeof(check)) != 0)
    {
        report_.status = Status::StorageError;
        return false;
    }
    ESP_LOGI(TAG, "Ilerleme kaydedildi: 6 yon tamam, egik kontrol %lu/3.",
             static_cast<unsigned long>(pending.next_pose));
    return true;
}

bool MPU6050::recoverAccelerometerFromLog(const std::array<imu_calibration::Vector3, 6>& means,
                                        const imu_calibration::Config& config)
{
    using namespace imu_calibration;
    if (!initialized_ || sensor_identity_ != 0x70 || !gyro_.calibrated)
    { report_.status = Status::InvalidModel; return false; }
    PendingCalibration pending{};
    if (loadPendingCalibration(pending)) return true;
    if (report_.status != Status::NoStoredCalibration) return false;
    // Zaten tamamlanmış kalibrasyon varsa log importu tekrar etkinleşmez.
    if (loadAccelerometerCalibration()) return true;
    if (report_.status != Status::NoStoredCalibration && report_.status != Status::InvalidModel) return false;
    // Log yalnız eğitim ortalamalarını içerir. Aynı veriyi bağımsız doğrulama
    // diye sayma: yalnız model fit edilir, kalite üç YENİ eğik pozdan alınır.
    if (fitAccelerometer(means, means, config, pending.model) != Status::Ok)
    { report_.status = Status::InvalidModel; return false; }
    pending.imported_from_log = 1;
    pending.validation_groups = 0;
    pending.model.validation_rms_g = 0;
    pending.model.validation_max_g = 0;
    pending.model.reference_temperature_c = gyro_.reference_temperature_c;
    pending.temperature_min = pending.temperature_max = gyro_.reference_temperature_c;
    ESP_LOGW(TAG, "Logdan 6 yon kurtarildi. Model henuz aktif DEGIL; 3 yeni egik kontrol zorunlu.");
    return savePendingCalibration(pending);
}

bool MPU6050::loadAccelerometerCalibration()
{
    using namespace imu_calibration;
    report_ = {};
    if (!initialized_) { report_.status = Status::NotInitialized; return false; }
    if (!initializeStorage()) { report_.status = Status::StorageError; return false; }
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        report_.status = err == ESP_ERR_NVS_NOT_FOUND ? Status::NoStoredCalibration : Status::StorageError;
        return false;
    }
    AccelRecord record{};
    size_t length = sizeof(record);
    err = nvs_get_blob(handle, KEY, &record, &length);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) { report_.status = Status::NoStoredCalibration; return false; }
    if (err != ESP_OK) { report_.status = Status::StorageError; return false; }
    AccelCalibration candidate{};
    if (length != sizeof(record) || !decodeRecord(record, candidate, sensor_identity_))
    {
        report_.status = Status::InvalidModel;
        ESP_LOGW(TAG, "Kalibrasyon kaydi gecersiz veya %s profiliyle uyumsuz; kullanilmadi.", sensorName());
        return false;
    }
    accel_ = candidate;
    report_.status = Status::Ok;
    report_.worst_accel_error_g = candidate.validation_max_g;
    ESP_LOGI(TAG, "ACC kalibrasyonu yuklendi. RMS=%.6f g, max=%.6f g, referans=%.2f C",
             accel_.validation_rms_g, accel_.validation_max_g, accel_.reference_temperature_c);
    return true;
}

bool MPU6050::saveAccelerometerCalibration()
{
    using namespace imu_calibration;
    if (!accel_.calibrated || !validModel(accel_))
    {
        report_.status = Status::InvalidModel;
        return false;
    }
    if (!initializeStorage()) { report_.status = Status::StorageError; return false; }
    const AccelRecord record = makeRecord(accel_, sensor_identity_);
    AccelCalibration check{};
    if (!decodeRecord(record, check, sensor_identity_)) { report_.status = Status::InvalidModel; return false; }
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) { report_.status = Status::StorageError; return false; }
    err = nvs_set_blob(handle, KEY, &record, sizeof(record));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK)
    {
        report_.status = Status::StorageError;
        ESP_LOGE(TAG, "ACC kaydedilemedi: %s", esp_err_to_name(err));
        return false;
    }

    // Yeni handle ile geri oku: yalnızca yazma çağrısının başarı koduna güvenme.
    AccelRecord readback{};
    size_t length = sizeof(readback);
    err = nvs_open(NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK)
    {
        err = nvs_get_blob(handle, KEY, &readback, &length);
        nvs_close(handle);
    }
    if (err != ESP_OK || length != sizeof(readback) ||
        std::memcmp(&record, &readback, sizeof(record)) != 0 ||
        !decodeRecord(readback, check, sensor_identity_))
    {
        report_.status = Status::StorageError;
        return false;
    }
    report_.status = Status::Ok;
    ESP_LOGI(TAG, "ACC NVS'ye kaydedildi ve geri okunarak dogrulandi.");
    return true;
}
