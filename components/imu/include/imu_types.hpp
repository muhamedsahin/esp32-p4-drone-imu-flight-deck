#pragma once

#include <cstdint>

/** Sensör register'ları: eksenler signed 16-bit ham count değeridir. */
struct RawIMUData
{
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t temperature = 0;
    // Sensördeki örnekleme anı değil, ESP32'nin okuma tamamlanma zamanı.
    int64_t timestamp_us = 0;
    bool valid = false;
};

/** read() çıktısı: ivme g, açısal hız rad/s, sıcaklık °C cinsindedir. */
struct IMUData
{
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float temperature_c = 0;
    int64_t timestamp_us = 0;
    bool valid = false;
    bool accel_calibrated = false;
    bool gyro_calibrated = false;
    // Sıcaklık modeli öğrenilmedi: referanstan uzaklaşınca yeniden kalibrasyon gerekir.
    bool gyro_temperature_valid = false;
};
