#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "imu_calibration.hpp"
#include "imu_types.hpp"
#include "imu_calibration_text.hpp"

/**
 * Tek görevden kullanılır. Kalibrasyon sırasında başka görev read() çağırmamalı.
 * Katmanlar: ham register okuması → doğrulanmış kalibrasyon → g / rad/s.
 * Bus bu nesneye aittir. Başka cihazlarla paylaşılacaksa bus sahipliği ayrıca tasarlanmalıdır.
 * MPU6050/MPU6500 kimliğe göre seçilir. Sınıf adı eski çağrılarla uyum için korunur.
 */
class MPU6050
{
public:
    MPU6050() = default;
    ~MPU6050();
    MPU6050(const MPU6050&) = delete;
    MPU6050& operator=(const MPU6050&) = delete;

    bool init();// Aygıtın bus protokolünü ayarlar
    uint8_t sensorIdentity() const { return sensor_identity_; }// sensör kimliği Örnek MPU 6050
    const char* sensorName() const;// Aygıtın isimi ney?
    // bool sürümlerinde hata gerçek sıfır ölçümünden ayrılır.
    bool readRaw(RawIMUData& output);
    bool read(IMUData& output);
    RawIMUData readRaw();   // Eski çağrılar için; sonuç.valid kontrol edilmeli.
    IMUData read();

    bool calibrateGyroscope(const imu_calibration::Config& config = {});
    bool calibrateAccelerometer(const imu_calibration::Config& config = {}, bool resume = true);
    // Yalnız logdaki gerçek altı ortalamayı kurtarır; eğik doğrulamayı atlamaz.
    bool recoverAccelerometerFromLog(const std::array<imu_calibration::Vector3, 6>& means,
                                    const imu_calibration::Config& config = {});
    bool loadAccelerometerCalibration();
    bool saveAccelerometerCalibration();

    // Dosya akışı: saklama politikasını çağıran katman belirler; her kabulde checkpoint.
    using TextCheckpoint = bool (*)(const imu_calibration::TextCalibration&, void*);
    bool calibrateFromText(imu_calibration::TextCalibration& state,
                           TextCheckpoint checkpoint, void* context,
                           const imu_calibration::Config& config = {});

    const imu_calibration::AccelCalibration& accelerometerCalibration() const { return accel_; }
    const imu_calibration::GyroCalibration& gyroscopeCalibration() const { return gyro_; }
    const imu_calibration::Report& calibrationReport() const { return report_; }

private:
    struct PendingCalibration
    {
        uint32_t version = 1;
        uint32_t identity = 0;
        uint32_t next_pose = 0;
        uint32_t imported_from_log = 0;
        imu_calibration::AccelCalibration model{};
        double squared_error = 0;
        double temperature_min = 0;
        double temperature_max = 0;
        uint32_t validation_groups = 6;
        uint32_t crc32 = 0;
    };
    bool loadPendingCalibration(PendingCalibration& pending);
    bool savePendingCalibration(PendingCalibration& pending);
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    i2c_master_dev_handle_t device_handle_ = nullptr;
    bool initialized_ = false;
    uint8_t sensor_identity_ = 0; // WHO_AM_I: 0x68=MPU6050, 0x70=MPU6500.
    int64_t startup_us_ = 0;
    int64_t last_sample_read_us_ = 0;
    imu_calibration::AccelCalibration accel_{};
    imu_calibration::GyroCalibration gyro_{};
    imu_calibration::Report report_{};
    imu_calibration::Status read_status_ = imu_calibration::Status::Ok;

    static constexpr uint8_t ADDRESS = 0x68;  // AD0=GND; AD0=VCC ise 0x69 gerekir.
    static constexpr uint8_t REG_SAMPLE_RATE = 0x19;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_CONFIG_2 = 0x1D; // Yalnız MPU6500.
    static constexpr uint8_t REG_INT_PIN_CONFIG = 0x37;
    static constexpr uint8_t REG_INT_ENABLE = 0x38;
    static constexpr uint8_t REG_INT_STATUS = 0x3A;
    static constexpr uint8_t REG_START_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
    static constexpr uint8_t REG_WHO_AM_I = 0x75;
    static constexpr uint32_t SCL_SPEED_HZ = 100000; // Jumper kablolarıyla daha rahat sinyal marjı.
    static constexpr int I2C_TIMEOUT_MS = 50;

    void releaseBus();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    bool configureRegister(uint8_t reg, uint8_t value);
    bool readStableIdentity(uint8_t& identity);
    double temperatureCelsius(int16_t raw) const;
    // Deadline mutlak esp_timer zamanı: sonsuza kadar beklemek yok.
    bool readFresh(RawIMUData& output, int64_t deadline_us);
    bool prepareCalibration(const imu_calibration::Config& config);
    imu_calibration::Status collectWindow(uint32_t samples, int64_t deadline_us,
                                          const imu_calibration::Config& config,
                                          imu_calibration::Window& output);
    bool captureStationary(int face_index, uint32_t samples,
                           const imu_calibration::Config& config,
                           imu_calibration::Window& training,
                           imu_calibration::Window& validation);
};
