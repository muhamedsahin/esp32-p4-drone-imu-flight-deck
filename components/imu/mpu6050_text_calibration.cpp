#include "mpu6050.hpp"

#include "esp_log.h"
#include <algorithm>
#include <cmath>

bool MPU6050::calibrateFromText(imu_calibration::TextCalibration &state, TextCheckpoint checkpoint,
                                void *context, const imu_calibration::Config &config)
{
    using namespace imu_calibration;
    constexpr const char *TAG = "IMU_FILE_CAL";
    if (!prepareCalibration(config))
        return false;
    if (state.identity != sensor_identity_ || !checkpoint)
    {
        report_.status = Status::InvalidConfiguration;
        return false;
    }
    const auto save = [&]() {
        if (checkpoint(state, context))
            return true;
        report_.status = Status::StorageError;
        return false;
    };

    GyroCalibration saved_gyro{};
    RawIMUData sample{};
    bool reused_gyro = false;
    const bool complete_gyro = completedGyroscope(state, saved_gyro);
    if (complete_gyro)
    {
        if (!readRaw(sample))
        {
            report_.status = read_status_;
            return false;
        }
        if (std::abs(temperatureCelsius(sample.temperature) - saved_gyro.reference_temperature_c) <= 5.0)
        {
            gyro_ = saved_gyro;
            reused_gyro = true;
            ESP_LOGI(TAG, "GYRO dosyadan yuklendi; olcum atlandi.");
        }
        else
            ESP_LOGW(TAG, "GYRO referans sicakligindan >5 C uzakta; bias tazelenecek.");
    }
    if (!reused_gyro)
    {
        // Üç gyro ekseni aynı sabitlik penceresinden gelir. null varsa birlikte ölçülür.
        if (!calibrateGyroscope(config))
            return false;
        for (size_t i = 0; i < 3; ++i)
            state.gyro_axes[i] = {{gyro_.bias_counts[i]}, true};
        state.gyro_temperature = {{gyro_.reference_temperature_c}, true};
        state.gyro_noise = {gyro_.noise_dps, true};
        state.gyro_error = {{gyro_.validation_max_dps}, true};
        if (!save())
            return false;
    }

    AccelCalibration candidate{};
    if (completedAccelerometer(state, candidate))
    {
        accel_ = candidate;
        report_.status = Status::Ok;
        ESP_LOGI(TAG, "ACC dosyadan yuklendi; kalibrasyonun tamami atlandi.");
        return true;
    }
    // Dolu ama fiziksel olarak geçersiz bir dosyayı eksik gibi kabul etme.
    if (state.accel_quality.present)
    {
        report_.status = Status::InvalidModel;
        return false;
    }

    // Yeni yüz ölçümü eski modelin eğik kontrolleriyle doğrulanamaz.
    for (const auto& face : state.faces)
    {
        if (!face.present)
        {
            for (auto& axis : state.axes) axis.present = false;
            for (auto& tilt : state.tilts) tilt.present = false;
            break;
        }
    }

    if (!state.accel_temperature.present)
        state.accel_temperature = {{gyro_.reference_temperature_c}, true};
    const double reference = state.accel_temperature.value[0];
    if (std::abs(reference - gyro_.reference_temperature_c) > config.max_session_temperature_span_c)
    {
        report_.status = Status::TemperatureUnstable;
        return false;
    }
    const auto temperature_ok = [&](const Window &a, const Window &b) {
        const double low = std::min(a.temperature.minimum(), b.temperature.minimum());
        const double high = std::max(a.temperature.maximum(), b.temperature.maximum());
        return high - low <= config.max_session_temperature_span_c &&
               std::abs(low - reference) <= config.max_session_temperature_span_c &&
               std::abs(high - reference) <= config.max_session_temperature_span_c;
    };
    std::array<Vector3, FACE_COUNT> means{};
    for (size_t face = 0; face < FACE_COUNT; ++face)
    {
        if (!state.faces[face].present)
        {
            ESP_LOGI(TAG, "[%u/6] %s yonunu YUKARI cevirin. OLCUM ALINDI mesajini bekleyin.",
                     unsigned(face + 1), faceName(static_cast<Face>(face)));
            Window training{}, validation{};
            if (!captureStationary(int(face), config.face_samples, config, training, validation))
                return false;
            if (!temperature_ok(training, validation))
            {
                report_.status = Status::TemperatureUnstable;
                return false;
            }
            state.faces[face] = {training.accelMean(), true};
            if (!save())
                return false; // Tek yüz de kalıcıdır; altı yüzün bitmesi beklenmez.
            ESP_LOGI(TAG, "%s OLCUM ALINDI ve dosya kaydi guncellendi.", faceName(static_cast<Face>(face)));
        }
        if (!matchesFace(state.faces[face].value, static_cast<Face>(face)))
        {
            report_.status = Status::WrongPose;
            return false;
        }
        means[face] = state.faces[face].value;
        ++report_.completed_faces;
    }

    bool have_model = true;
    for (const auto &axis : state.axes)
        have_model = have_model && axis.present;
    if (have_model)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            candidate.bias_counts[i] = state.axes[i].value[0];
            candidate.counts_per_g[i] = state.axes[i].value[1];
        }
        candidate.reference_temperature_c = reference;
        if (!validModel(candidate))
        {
            report_.status = Status::InvalidModel;
            return false;
        }
    }
    else
    {
        // Dosyada yüz ortalamaları var; geçmiş bağımsız pencereler yok.
        // Fit verisi kalite kanıtı sayılmaz. Kalite YENİ eğik pencerelerden gelir.
        report_.status = fitAccelerometer(means, means, config, candidate);
        if (report_.status != Status::Ok)
            return false;
        candidate.reference_temperature_c = reference;
        candidate.validation_rms_g = candidate.validation_max_g = 0;
        for (size_t i = 0; i < 3; ++i)
            state.axes[i] = {{candidate.bias_counts[i], candidate.counts_per_g[i]}, true};
        for (auto &tilt : state.tilts)
            tilt.present = false;
        if (!save())
            return false;
    }

    constexpr const char *names[] = {"XY", "XZ", "YZ"};
    double squared = 0, worst = 0;
    for (size_t pose = 0; pose < 3; ++pose)
    {
        if (!state.tilts[pose].present)
        {
            ESP_LOGI(TAG, "[D%u/3] %s EGIK KONTROL: iki hedef ekseni egip sabit destekleyin.",
                     unsigned(pose + 1), names[pose]);
            Window first{}, second{};
            if (!captureStationary(int(6 + pose), config.validation_samples, config, first, second))
                return false;
            if (!temperature_ok(first, second))
            {
                report_.status = Status::TemperatureUnstable;
                return false;
            }
            const double a = std::abs(accelerationNorm(first.accelMean(), candidate) - 1);
            const double b = std::abs(accelerationNorm(second.accelMean(), candidate) - 1);
            if (!std::isfinite(a) || !std::isfinite(b) || std::max(a, b) > config.max_accel_validation_g)
            {
                report_.status = Status::ValidationFailed;
                return false;
            }
            state.tilts[pose] = {{a, b}, true};
            if (!save())
                return false;
            ESP_LOGI(TAG, "%s EGIK KONTROL KABUL ve kaydedildi.", names[pose]);
        }
        for (double error : state.tilts[pose].value)
        {
            if (error < 0 || !std::isfinite(error) || error > config.max_accel_validation_g)
            {
                report_.status = Status::ValidationFailed;
                return false;
            }
            squared += error * error;
            worst = std::max(worst, error);
        }
        ++report_.verified_tilted_poses;
    }
    state.accel_quality = {{std::sqrt(squared / 6), worst}, true};
    if (!completedAccelerometer(state, candidate))
    {
        report_.status = Status::InvalidModel;
        return false;
    }
    if (!save())
        return false;
    accel_ = candidate;
    report_.status = Status::Ok;
    return true;
}
