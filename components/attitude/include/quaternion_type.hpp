#pragma once

/** Birim quaternion; varsayılan değer sıfır dönüşü temsil eder. */
struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/** ZYX sıralı Euler çıktısı; bütün açılar radyandır. */
struct EulerAngles
{
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
};
