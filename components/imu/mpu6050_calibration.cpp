#include "mpu6050.hpp"

#include <algorithm>
#include <cmath>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
using namespace imu_calibration;
constexpr const char* TAG = "IMU_CAL";

bool samePosition(const Window& a, const Window& b, const Config& c)
{
    for (size_t axis = 0; axis < 3; ++axis)
    {
        if (std::abs(a.accel[axis].mean() - b.accel[axis].mean()) /
                ACCEL_COUNTS_PER_G > c.max_accel_drift_g ||
            std::abs(a.gyro[axis].mean() - b.gyro[axis].mean()) /
                GYRO_COUNTS_PER_DPS > c.max_gyro_validation_dps) return false;
    }
    return std::abs(a.temperature.mean() - b.temperature.mean()) <= c.max_temperature_span_c;
}
}

bool MPU6050::prepareCalibration(const imu_calibration::Config& config)
{
    using namespace imu_calibration;
    report_ = {};
    if (!initialized_) { report_.status = Status::NotInitialized; return false; }
    if (!validConfig(config)) { report_.status = Status::InvalidConfiguration; return false; }

    // Açılış sıcaklık geçişini azalt. Bu bekleme bir sıcaklık modeli değildir;
    // kalan kararsızlık sonraki pencerelerde sıcaklık/drift kontrolüyle reddedilir.
    const int64_t ready_at = startup_us_ + static_cast<int64_t>(config.warmup_ms) * 1000;
    if (esp_timer_get_time() < ready_at)
        ESP_LOGI(TAG, "Sensor isinmasi bekleniyor (acilistan itibaren %lu ms).",
                 static_cast<unsigned long>(config.warmup_ms));
    while (esp_timer_get_time() < ready_at) vTaskDelay(1);
    report_.status = Status::Ok;
    return true;
}

imu_calibration::Status MPU6050::collectWindow(
    uint32_t samples, int64_t deadline_us, const imu_calibration::Config& config,
    imu_calibration::Window& output)
{
    using namespace imu_calibration;
    output = {};
    uint8_t discarded_status = 0;
    // Önceki fazdan kalan DATA_RDY'yi at: pencere yeni ölçümle başlasın.
    if (!readRegisters(REG_INT_STATUS, &discarded_status, 1)) return Status::IoError;
    int64_t previous_sample_us = 0;
    for (uint32_t i = 0; i < samples; ++i)
    {
        RawIMUData sample{};
        const int64_t sample_deadline = std::min(deadline_us, esp_timer_get_time() + 100000);
        if (!readFresh(sample, sample_deadline)) return read_status_;
        // Uzun bir görev kesintisini tek bir kesintisiz pencere sayma.
        if (previous_sample_us && sample.timestamp_us - previous_sample_us > 100000)
            return Status::Motion;
        previous_sample_us = sample.timestamp_us;
        output.add({double(sample.ax), double(sample.ay), double(sample.az)},
                   {double(sample.gx), double(sample.gy), double(sample.gz)},
                   temperatureCelsius(sample.temperature), i < samples / 2);
    }
    return stationary(output, config);
}

bool MPU6050::captureStationary(
    int face_index, uint32_t samples, const imu_calibration::Config& config,
    imu_calibration::Window& training, imu_calibration::Window& validation)
{
    using namespace imu_calibration;
    const int64_t deadline = esp_timer_get_time() + static_cast<int64_t>(config.pose_timeout_ms) * 1000;
    int64_t next_feedback_us = 0;
    Status last_rejection = Status::Motion;
    const auto orientationOk = [face_index](const Window& w) {
        if (face_index >= 6) return matchesTiltedPose(w.accelMean(), static_cast<size_t>(face_index - 6));
        return face_index < 0 || matchesFace(w.accelMean(), static_cast<Face>(face_index));
    };

    while (esp_timer_get_time() < deadline)
    {
        Window settle{};
        Status status = collectWindow(config.settle_samples, deadline, config, settle);
        if (status == Status::Ok && !orientationOk(settle)) status = Status::WrongPose;
        if (status == Status::Ok)
        {
            ESP_LOGI(TAG, "Sabit durus bulundu. Olcum ve dogrulama bitene kadar dokunmayin.");
            status = collectWindow(samples, deadline, config, training);
        }
        if (status == Status::Ok && !orientationOk(training)) status = Status::WrongPose;
        if (status == Status::Ok && !samePosition(settle, training, config)) status = Status::Motion;
        if (status == Status::Ok)
            status = collectWindow(config.validation_samples, deadline, config, validation);
        if (status == Status::Ok && !orientationOk(validation)) status = Status::WrongPose;
        if (status == Status::Ok && !samePosition(training, validation, config)) status = Status::Motion;
        if (status == Status::Ok)
        {
            report_.accepted_samples += samples + config.validation_samples;
            return true;
        }

        // Bozuk I2C'den sıfır örnek üretme; iletişim ve data-ready hatası oturumu bitirir.
        if (status == Status::IoError || status == Status::Timeout)
        {
            report_.status = status;
            return false;
        }
        ++report_.rejected_windows;
        last_rejection = status;
        if (esp_timer_get_time() >= next_feedback_us)
        {
            ESP_LOGW(TAG, "Pencere reddedildi: %s. Pozisyonu duzeltin, sabit tutun.", statusName(status));
            if (status == Status::WrongPose)
            {
                const auto a = settle.accelMean();
                ESP_LOGI(TAG, "Yon yardimi ham[g]: X=%+.3f Y=%+.3f Z=%+.3f",
                         a[0] / ACCEL_COUNTS_PER_G, a[1] / ACCEL_COUNTS_PER_G, a[2] / ACCEL_COUNTS_PER_G);
            }
            next_feedback_us = esp_timer_get_time() + 2000000;
        }
        vTaskDelay(1);
    }
    ESP_LOGE(TAG, "Pozisyon icin sure doldu. Son ret nedeni: %s", statusName(last_rejection));
    report_.status = Status::Timeout;
    return false;
}

bool MPU6050::calibrateGyroscope(const imu_calibration::Config& config)
{
    using namespace imu_calibration;
    if (!prepareCalibration(config)) return false;
    ESP_LOGI(TAG, "GYRO: sensoru sabit bir destek uzerine koyun. Motorlar kapali olsun.");
    Window training{}, validation{};
    if (!captureStationary(-1, config.gyro_samples, config, training, validation)) return false;

    GyroCalibration candidate{};
    candidate.bias_counts = training.gyroMean();
    candidate.reference_temperature_c = training.temperature.mean();
    for (size_t axis = 0; axis < 3; ++axis)
    {
        candidate.noise_dps[axis] = training.gyro[axis].stddev() / GYRO_COUNTS_PER_DPS;
        const double residual = std::abs(validation.gyro[axis].mean() - candidate.bias_counts[axis]) /
                                GYRO_COUNTS_PER_DPS;
        candidate.validation_max_dps = std::max(candidate.validation_max_dps, residual);
    }
    if (candidate.validation_max_dps > config.max_gyro_validation_dps)
    {
        report_.status = Status::ValidationFailed;
        return false;
    }
    candidate.calibrated = true;
    // Bütün kontroller bitene kadar önceki iyi bias korunur.
    gyro_ = candidate;
    report_.worst_gyro_error_dps = candidate.validation_max_dps;
    report_.status = Status::Ok;
    ESP_LOGI(TAG, "GYRO bias [count]: %.4f %.4f %.4f; T=%.2f C",
             gyro_.bias_counts[0], gyro_.bias_counts[1], gyro_.bias_counts[2],
             gyro_.reference_temperature_c);
    ESP_LOGI(TAG, "GYRO gurultu [dps std]: %.4f %.4f %.4f; dogrulama max=%.5f dps",
             gyro_.noise_dps[0], gyro_.noise_dps[1], gyro_.noise_dps[2],
             gyro_.validation_max_dps);
    return true;
}

bool MPU6050::calibrateAccelerometer(const imu_calibration::Config& config, bool resume)
{
    using namespace imu_calibration;
    if (!prepareCalibration(config)) return false;
    PendingCalibration pending{};
    bool restored = resume && loadPendingCalibration(pending);
    if (resume && !restored && report_.status != Status::NoStoredCalibration) return false;
    if (restored)
    {
        report_.completed_faces = 6;
        report_.verified_tilted_poses = static_cast<uint8_t>(pending.next_pose);
        ESP_LOGI(TAG, "KALDIGI YERDEN DEVAM: 6 yon kayitli; tamamlanan egik kontrol %lu/3.",
                 static_cast<unsigned long>(pending.next_pose));
        ESP_LOGI(TAG, "Alti yonu TEKRAR YAPMAYIN. Yalniz kalan egik kontroller yapilacak.");
    }
    else
    {
        ESP_LOGI(TAG, "ACC: +X -X +Y -Y +Z -Z. Her yonu yukari cevirip sabit destek kullanin.");
        ESP_LOGI(TAG, "Yeni yontem: toplam ivme 1g fit. Tam 90 derece gerekmez; sonra 3 egik kontrol var.");
    }
    ESP_LOGI(TAG, "Elinizde tutmayin. Yanlis yon/hareket kabul edilmez; her yon icin %lu s var.",
             static_cast<unsigned long>(config.pose_timeout_ms / 1000));

    std::array<Vector3, FACE_COUNT> training_means{}, validation_means{};
    RunningStats session_temperature{};
    if (restored)
    {
        session_temperature.add(pending.temperature_min);
        session_temperature.add(pending.temperature_max);
    }
    for (size_t face = restored ? FACE_COUNT : 0; face < FACE_COUNT; ++face)
    {
        ESP_LOGI(TAG, "[%u/6] %s yonunu YUKARI cevirin. OLCUM ALINDI mesajini bekleyin.",
                 static_cast<unsigned>(face + 1), faceName(static_cast<Face>(face)));
        Window training{}, validation{};
        if (!captureStationary(static_cast<int>(face), config.face_samples, config, training, validation))
            return false;
        training_means[face] = training.accelMean();
        validation_means[face] = validation.accelMean();
        // Yalnızca yüz ortalamalarını değil, bütün oturumun sıcaklık uçlarını denetle.
        session_temperature.add(training.temperature.minimum());
        session_temperature.add(training.temperature.maximum());
        session_temperature.add(validation.temperature.minimum());
        session_temperature.add(validation.temperature.maximum());
        ++report_.completed_faces;
        ESP_LOGI(TAG, "%s OLCUM ALINDI (model henuz dogrulanmadi). Ortalama [count]: %.3f %.3f %.3f",
                 faceName(static_cast<Face>(face)), training_means[face][0],
                 training_means[face][1], training_means[face][2]);
    }
    if (session_temperature.range() > config.max_session_temperature_span_c)
    {
        report_.status = Status::TemperatureUnstable;
        ESP_LOGE(TAG, "Alti yon boyunca sicaklik %.2f C degisti; tekrar deneyin.",
                 session_temperature.range());
        return false;
    }
    AccelCalibration candidate{};
    report_.status = restored ? Status::Ok : fitAccelerometer(training_means, validation_means, config, candidate);
    if (report_.status != Status::Ok)
    {
        ESP_LOGE(TAG, "ACC norm modeli reddedildi: %s. Yon kapsami/olcum kararliligini kontrol edin.",
                 statusName(report_.status));
        return false;
    }
    if (restored) candidate = pending.model;
    else
    {
        candidate.reference_temperature_c = session_temperature.mean();
        pending.model = candidate;
        pending.squared_error = candidate.validation_rms_g * candidate.validation_rms_g * FACE_COUNT;
        pending.temperature_min = session_temperature.minimum();
        pending.temperature_max = session_temperature.maximum();
        if (!savePendingCalibration(pending)) return false;
    }
    if (std::abs(candidate.reference_temperature_c - gyro_.reference_temperature_c) > 3.0)
    {
        report_.status = Status::TemperatureUnstable;
        ESP_LOGW(TAG, "Kayitli olcum sicakligindan >3 C uzaktasiniz; eski ilerleme korundu.");
        return false;
    }
    // Altı parametre altı noktayı tam açıklayabilir. Başarı ilanından önce fit'te
    // kullanılmayan XY/XZ/YZ eğik konumlarını ölç; aşırı uyumu böyle yakala.
    constexpr const char* planes[] = {"XY", "XZ", "YZ"};
    double squared_error = pending.squared_error;
    for (size_t pose = pending.next_pose; pose < 3; ++pose)
    {
        ESP_LOGI(TAG, "[D%u/3] %s EGIK KONTROL: bu iki ekseni yaklasik 45 derece egip sabit destekleyin.",
                 static_cast<unsigned>(pose + 1), planes[pose]);
        ESP_LOGI(TAG, "Kesin aci gerekmez. Iki hedef eksen de 0.30g'den buyuk, digeri 0.30g'den kucuk olsun.");
        Window first{}, second{};
        if (!captureStationary(static_cast<int>(6 + pose), config.validation_samples, config, first, second))
            return false;
        const double first_error = std::abs(accelerationNorm(first.accelMean(), candidate) - 1.0);
        const double second_error = std::abs(accelerationNorm(second.accelMean(), candidate) - 1.0);
        const double error = std::max(first_error, second_error);
        report_.worst_accel_error_g = std::max(candidate.validation_max_g, error);
        ESP_LOGI(TAG, "%s norm kontrolu: hata=%.6f g, sinir=%.6f g", planes[pose], error,
                 config.max_accel_validation_g);
        if (!std::isfinite(error) || error > config.max_accel_validation_g)
        {
            report_.status = Status::ValidationFailed;
            ESP_LOGE(TAG, "%s egik kontrol gecemedi. Yeni model kullanilmadi; onceki kalibrasyon korundu.", planes[pose]);
            return false;
        }
        candidate.validation_max_g = std::max(candidate.validation_max_g, error);
        squared_error += first_error * first_error + second_error * second_error;
        session_temperature.add(first.temperature.minimum());
        session_temperature.add(first.temperature.maximum());
        session_temperature.add(second.temperature.minimum());
        session_temperature.add(second.temperature.maximum());
        if (session_temperature.range() > config.max_session_temperature_span_c ||
            std::abs(first.temperature.mean() - candidate.reference_temperature_c) > 3.0 ||
            std::abs(second.temperature.mean() - candidate.reference_temperature_c) > 3.0)
        {
            report_.status = Status::TemperatureUnstable;
            return false; // Önceki iyi kontrol korunur; bu faz kaydedilmez.
        }
        pending.next_pose = static_cast<uint32_t>(pose + 1);
        pending.validation_groups += 2;
        pending.model = candidate;
        pending.squared_error = squared_error;
        pending.temperature_min = session_temperature.minimum();
        pending.temperature_max = session_temperature.maximum();
        pending.model.validation_rms_g = std::sqrt(squared_error / pending.validation_groups);
        if (!savePendingCalibration(pending)) return false;
        ++report_.verified_tilted_poses;
        ESP_LOGI(TAG, "%s EGIK KONTROL KABUL.", planes[pose]);
    }
    if (session_temperature.range() > config.max_session_temperature_span_c)
    {
        report_.status = Status::TemperatureUnstable;
        ESP_LOGE(TAG, "Oturum sicakligi %.2f C degisti; yeni model kullanilmadi.", session_temperature.range());
        return false;
    }
    candidate.validation_rms_g = std::sqrt(squared_error / pending.validation_groups);
    accel_ = candidate; // Altı yön + üç yeni eğik konum geçmeden aktif model değişmez.
    report_.worst_accel_error_g = candidate.validation_max_g;
    ESP_LOGI(TAG, "ACC bias [count]: %.4f %.4f %.4f",
             accel_.bias_counts[0], accel_.bias_counts[1], accel_.bias_counts[2]);
    ESP_LOGI(TAG, "ACC hassasiyet [count/g]: %.4f %.4f %.4f",
             accel_.counts_per_g[0], accel_.counts_per_g[1], accel_.counts_per_g[2]);
    ESP_LOGI(TAG, "ACC dogrulama RMS=%.6f g, max=%.6f g; kabul=%lu, ret=%lu",
             accel_.validation_rms_g, accel_.validation_max_g,
             static_cast<unsigned long>(report_.accepted_samples),
             static_cast<unsigned long>(report_.rejected_windows));
    return true;
}
