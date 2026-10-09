#include "mpu6050.hpp"

#include <cmath>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
    constexpr const char *TAG = "MPU_IMU";
    int16_t signedWord(uint8_t high, uint8_t low)
    {
        // Big-endian two's complement. 16 bit = 2 byte; işareti açıkça çözüyoruz.
        const uint16_t bits = (static_cast<uint16_t>(high) << 8) | low;
        const int32_t value = bits >= 0x8000 ? static_cast<int32_t>(bits) - 65536 : bits;
        return static_cast<int16_t>(value);
    }
    void delayMilliseconds(uint32_t ms)
    {
        // 100 Hz FreeRTOS'ta 2 ms → 0 tick olabilir. Daima en az bir tick bekle.
        const TickType_t ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }
}

MPU6050::~MPU6050() { releaseBus(); }

void MPU6050::releaseBus()
{
    initialized_ = false;
    sensor_identity_ = 0;
    if (device_handle_)
    {
        const esp_err_t err = i2c_master_bus_rm_device(device_handle_);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Cihaz serbest birakilamadi: %s", esp_err_to_name(err));
            return;
        }
        device_handle_ = nullptr;
    }
    if (bus_handle_)
    {
        const esp_err_t err = i2c_del_master_bus(bus_handle_);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Bus serbest birakilamadi: %s", esp_err_to_name(err));
            return;
        }
        bus_handle_ = nullptr;
    }
}

bool MPU6050::init()
{
    if (initialized_)
        return true;
    releaseBus();
    if (bus_handle_ || device_handle_)
        return false;

    i2c_master_bus_config_t bus{};
    bus.i2c_port = I2C_NUM_0;
    bus.sda_io_num = GPIO_NUM_7;// bunları nesne oluşturulurken al burada olmasın !!!
    bus.scl_io_num = GPIO_NUM_8;
    bus.clk_source = I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt = 7;
    bus.flags.enable_internal_pullup = true;
    // Dahili pull-up yardımcıdır; hattın harici pull-up'larının yerine geçmez.
    esp_err_t err = i2c_new_master_bus(&bus, &bus_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus olusturulamadi: %s", esp_err_to_name(err));
        return false;
    }
    i2c_device_config_t device{};
    device.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device.device_address = ADDRESS;
    device.scl_speed_hz = SCL_SPEED_HZ;
    err = i2c_master_bus_add_device(bus_handle_, &device, &device_handle_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Cihaz eklenemedi: %s", esp_err_to_name(err));
        releaseBus();
        return false;
    }
    // Sensör ve MCU aynı anda beslenirse ilk register erişiminden önce bekle.
    delayMilliseconds(100);
    uint8_t identity = 0;
    if (!readStableIdentity(identity))
    {
        releaseBus();
        return false;
    }
    if (identity != 0x68 && identity != 0x70)
    {
        ESP_LOGE(TAG, "Desteklenmeyen WHO_AM_I=0x%02X. MPU6050=0x68, MPU6500=0x70.", identity);
        releaseBus();
        return false;
    }
    if (!writeRegister(REG_PWR_MGMT_1, 0x80))
    {
        releaseBus();
        return false;
    }
    delayMilliseconds(100);
    uint8_t after_reset = 0;
    if (!readStableIdentity(after_reset))
    {
        releaseBus();
        return false;
    }
    if (after_reset != identity)
    {
        ESP_LOGE(TAG, "Reset sonrasi kimlik degisti: 0x%02X -> 0x%02X. I2C hattini kontrol edin.",
                 identity, after_reset);
        releaseBus();
        return false;
    }
    sensor_identity_ = identity;
    ESP_LOGI(TAG, "Algilandi: %s, WHO_AM_I=0x%02X, I2C adres=0x%02X",
             sensorName(), sensor_identity_, ADDRESS);

    // Önceki ayarlara güvenme. PLL clock, bütün eksenler açık,
    // gyro DLPF=3 (~42/41 Hz), 1 kHz / (1+9) = 100 Hz, ±2g ve ±250°/s.
    // MPU6500 GYRO_CONFIG=0: FCHOICE_B=00, yani DLPF devrede.
    // Bunlar kalibrasyon/test ayarlarıdır; agresif uçuş için ayrıca tasarım gerekir.
    if (!configureRegister(REG_PWR_MGMT_1, 0x01) ||
        !configureRegister(REG_PWR_MGMT_2, 0x00) ||
        !configureRegister(REG_CONFIG, 0x03) ||
        !configureRegister(REG_SAMPLE_RATE, 0x09) ||
        !configureRegister(REG_GYRO_CONFIG, 0x00) ||
        !configureRegister(REG_ACCEL_CONFIG, 0x00) ||
        !configureRegister(REG_INT_PIN_CONFIG, 0x00) ||
        !configureRegister(REG_INT_ENABLE, 0x01))
    {
        releaseBus();
        return false;
    }
    // MPU6500 ivme filtresi CONFIG'ten bağımsızdır: ACCEL_FCHOICE_B=0,
    // A_DLPF_CFG=3 (~41 Hz). MPU6050'da 0x1D'ye kesinlikle yazma.
    if (sensor_identity_ == 0x70 && !configureRegister(REG_ACCEL_CONFIG_2, 0x03))
    {
        releaseBus();
        return false;
    }
    // Yeniden başlatılmış bir cihaz önceki RAM kalibrasyonunu devralmamalı.
    accel_ = {};
    gyro_ = {};
    startup_us_ = esp_timer_get_time();
    last_sample_read_us_ = 0;
    delayMilliseconds(100);
    initialized_ = true;
    ESP_LOGI(TAG, "%s hazir: SDA=7 SCL=8, I2C=100 kHz, 100 Hz, ACC +/-2g, GYRO +/-250 dps",
             sensorName());
    return true;
}

const char *MPU6050::sensorName() const
{
    switch (sensor_identity_)
    {
    case 0x68:
        return "MPU6050";
    case 0x70:
        return "MPU6500";
    default:
        return "Bilinmeyen IMU";
    }
}

bool MPU6050::readStableIdentity(uint8_t &identity)
{
    if (!readRegisters(REG_WHO_AM_I, &identity, 1))
    {
        ESP_LOGE(TAG, "Kimlik okunamadi: SDA=7 SCL=8, VCC/GND ve AD0=LOW (adres 0x68) kontrol edin.");
        return false;
    }
    // Bir bozuk byte'ı başka sensör modeli sanma. Reset öncesi ve sonrası
    // üçer okuma aynı olmalı; bilinmeyen kimlikler ayrıca reddedilir.
    for (unsigned i = 0; i < 2; ++i)
    {
        delayMilliseconds(2);
        uint8_t next = 0;
        if (!readRegisters(REG_WHO_AM_I, &next, 1))
            return false;
        if (next != identity)
        {
            ESP_LOGE(TAG, "Kimlik kararsiz: 0x%02X / 0x%02X. Kablo ve pull-up'lari kontrol edin.",
                     identity, next);
            return false;
        }
    }
    return true;
}

double MPU6050::temperatureCelsius(int16_t raw) const
{
    // İki modelin ACC/GYRO ölçekleri aynı, sıcaklık ölçekleri farklıdır.
    // Tek dönüşüm hem read() hem kalibrasyon pencereleri için kullanılır.
    return sensor_identity_ == 0x70 ? raw / 333.87 + 21.0 : raw / 340.0 + 36.53;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value)
{
    if (!device_handle_)
        return false;
    const uint8_t bytes[] = {reg, value};
    const esp_err_t err = i2c_master_transmit(device_handle_, bytes, sizeof(bytes), I2C_TIMEOUT_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "0x%02X yazma: %s", reg, esp_err_to_name(err));
    return err == ESP_OK;
}

bool MPU6050::readRegisters(uint8_t reg, uint8_t *buffer, size_t length)
{
    if (!device_handle_ || !buffer || length == 0)
        return false;
    const esp_err_t err = i2c_master_transmit_receive(
        device_handle_, &reg, 1, buffer, length, I2C_TIMEOUT_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "0x%02X okuma: %s", reg, esp_err_to_name(err));
    return err == ESP_OK;
}

bool MPU6050::configureRegister(uint8_t reg, uint8_t value)
{
    uint8_t actual = 0;
    if (!writeRegister(reg, value) || !readRegisters(reg, &actual, 1))
        return false;
    if (actual != value)
    {
        ESP_LOGE(TAG, "0x%02X ayari dogrulanamadi: yazilan 0x%02X, okunan 0x%02X", reg, value, actual);
        return false;
    }
    return true;
}

bool MPU6050::readFresh(RawIMUData &output, int64_t deadline_us)
{
    output = {};
    read_status_ = imu_calibration::Status::NotInitialized;
    if (!initialized_)
        return false;
    // INT_RD_CLEAR=0: sadece INT_STATUS okuması hazır bayrağını temizler.
    while (esp_timer_get_time() < deadline_us)
    {
        // Status/burst arasına yeni örnek denk gelse bile sonraki okuma en az
        // bir sensör periyodu sonra yapılır; aynı örnek iki kez sayılamaz.
        if (last_sample_read_us_ && esp_timer_get_time() - last_sample_read_us_ < 10000)
        {
            vTaskDelay(1);
            continue;
        }
        uint8_t status = 0;
        if (!readRegisters(REG_INT_STATUS, &status, 1))
        {
            read_status_ = imu_calibration::Status::IoError;
            return false;
        }
        if (status & 0x01)
        {
            // Tek burst: ACC[0..5], sıcaklık[6..7], GYRO[8..13].
            uint8_t bytes[14]{};
            if (!readRegisters(REG_START_ACCEL_XOUT_H, bytes, sizeof(bytes)))
            {
                read_status_ = imu_calibration::Status::IoError;
                return false;
            }
            if (esp_timer_get_time() >= deadline_us)
            {
                read_status_ = imu_calibration::Status::Timeout;
                return false;
            }
            output.ax = signedWord(bytes[0], bytes[1]);
            output.ay = signedWord(bytes[2], bytes[3]);
            output.az = signedWord(bytes[4], bytes[5]);
            output.temperature = signedWord(bytes[6], bytes[7]);
            output.gx = signedWord(bytes[8], bytes[9]);
            output.gy = signedWord(bytes[10], bytes[11]);
            output.gz = signedWord(bytes[12], bytes[13]);
            output.timestamp_us = esp_timer_get_time();
            last_sample_read_us_ = output.timestamp_us;
            output.valid = true;
            read_status_ = imu_calibration::Status::Ok;
            return true;
        }
        vTaskDelay(1);
    }
    read_status_ = imu_calibration::Status::Timeout;
    return false;
}

bool MPU6050::readRaw(RawIMUData &output)
{
    return readFresh(output, esp_timer_get_time() + 100000);
}
RawIMUData MPU6050::readRaw()
{
    RawIMUData result{};
    readRaw(result);
    return result;
}

bool MPU6050::read(IMUData &output)
{
    output = {};
    RawIMUData raw{};
    if (!readRaw(raw))
        return false;
    const auto accel = imu_calibration::correctAcceleration(
        {double(raw.ax), double(raw.ay), double(raw.az)}, accel_);
    output.ax = static_cast<float>(accel[0]);
    output.ay = static_cast<float>(accel[1]);
    output.az = static_cast<float>(accel[2]);
    constexpr double gyro_to_radians = imu_calibration::DEGREES_TO_RADIANS /
                                       imu_calibration::GYRO_COUNTS_PER_DPS;
    output.gx = static_cast<float>((raw.gx - gyro_.bias_counts[0]) * gyro_to_radians);
    output.gy = static_cast<float>((raw.gy - gyro_.bias_counts[1]) * gyro_to_radians);
    output.gz = static_cast<float>((raw.gz - gyro_.bias_counts[2]) * gyro_to_radians);
    output.temperature_c = static_cast<float>(temperatureCelsius(raw.temperature));
    output.timestamp_us = raw.timestamp_us;
    output.accel_calibrated = accel_.calibrated;
    output.gyro_calibrated = gyro_.calibrated;
    output.gyro_temperature_valid = gyro_.calibrated &&
                                    std::abs(output.temperature_c - gyro_.reference_temperature_c) <= 5.0;
    output.valid = std::abs(int(raw.ax)) < 32700 && std::abs(int(raw.ay)) < 32700 &&
                   std::abs(int(raw.az)) < 32700 && std::abs(int(raw.gx)) < 32700 &&
                   std::abs(int(raw.gy)) < 32700 && std::abs(int(raw.gz)) < 32700;
    return output.valid;
}
IMUData MPU6050::read()
{
    IMUData result{};
    read(result);
    return result;
}
