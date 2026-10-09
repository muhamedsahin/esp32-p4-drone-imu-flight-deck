#include "detail/imu_application.hpp"

#include "esp_log.h"
#include "sdkconfig.h"
#include <cmath>
#include <cstdio>

#if defined(ESP_PLATFORM)
// ESP-IDF, calibration.txt dosyasını bu linker sembolüyle firmware'e gömer.
extern const char calibration_source[] asm("_binary_calibration_txt_start");
#else
// Bilgisayardaki API testleri aynı akışa kendi test dosyası içeriğini verir.
extern const char calibration_source[];
#endif
namespace
{
constexpr const char *TAG = "IMU_APP";
}

bool ImuApplication::init()
{
    if (!imu_.init())
    {
        ESP_LOGE(TAG, "Sensor baslatilamadi.");
        return false;
    }
    std::string error;
    imu_calibration::TextCalibration seed{};
    if (!imu_calibration::parseCalibrationText(calibration_source, seed, error))
    {
        ESP_LOGE(TAG, "Kalibrasyon dosyasi: %s", error.c_str());
        return false;
    }
    if (seed.identity != imu_.sensorIdentity())
    {
        ESP_LOGE(TAG, "Kaynak dosya baska sensor profiline ait; kayit degistirilmedi.");
        return false;
    }
    if (!store_.load(calibration_source, state_, error))
    {
        ESP_LOGE(TAG, "Kalibrasyon dosyasi: %s", error.c_str());
        return false;
    }
    if (state_.identity != imu_.sensorIdentity())
    {
        ESP_LOGE(TAG, "Dosya baska sensor profiline ait; kullanilmadi.");
        return false;
    }
    ESP_LOGI(TAG, "%s: calibration.txt / NVS text_v1 hazir.", imu_.sensorName());
    return true;
}

bool ImuApplication::calibrate()
{
    imu_calibration::Config config{};
    config.warmup_ms = CONFIG_MPU6050_CALIBRATION_WARMUP_MS;
    if (!imu_.calibrateFromText(state_, checkpoint, this, config))
    {
        ESP_LOGE(TAG, "Kalibrasyon durdu: %s. Kaydedilen ilerleme korunuyor.",
                 imu_calibration::statusName(imu_.calibrationReport().status));
        return false;
    }
    ESP_LOGI(TAG, "Olcum hazir. ACC=g, GYRO=rad/s.");
    return true;
}

bool ImuApplication::checkpoint(const imu_calibration::TextCalibration& state, void* context)
{
    auto& app = *static_cast<ImuApplication*>(context);
    if (!app.store_.save(state))
        return false;
    // PC monitörü bu çerçeveyi her kabul edilen yüz/kontrol sonrasında dosyaya yazar.
    const auto text = imu_calibration::calibrationText(state);
    std::printf("\nCALIBRATION_UPDATE_BEGIN\n%sCALIBRATION_UPDATE_END\n", text.c_str());
    std::fflush(stdout);
    return true;
}

void ImuApplication::printCalibration() const
{
    ESP_LOGI(TAG, "SON KALIBRASYON: x/y/z = bias[count], count/g; null = eksik.");
    const auto text = imu_calibration::calibrationText(state_);
    std::printf("\nCALIBRATION_BEGIN\n%sCALIBRATION_END\n", text.c_str());
    std::fflush(stdout);
}

bool ImuApplication::read(IMUData &data)
{
    if (imu_.read(data))
        return true;
    ++failed_reads_;
    if (failed_reads_ == 1 || failed_reads_ % 20 == 0)
        ESP_LOGW(TAG, "Gecersiz olcum atlandi (toplam %lu).", static_cast<unsigned long>(failed_reads_));
    return false;
}

void ImuApplication::printMeasurement(const IMUData &data)
{
    if (!data.gyro_temperature_valid && !temperature_warning_)
        ESP_LOGW(TAG, "Gyro referansindan >5 C uzakta. Sabitken bias'i yenileyin.");
    temperature_warning_ = !data.gyro_temperature_valid;
    const double norm = std::sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    std::printf("ACC[g] X=%+.5f Y=%+.5f Z=%+.5f |A|=%.5f | "
                "GYRO[rad/s] X=%+.6f Y=%+.6f Z=%+.6f | T=%.2f C\n",
                data.ax, data.ay, data.az, norm, data.gx, data.gy, data.gz, data.temperature_c);
}
