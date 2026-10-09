#pragma once

/** İvmeölçerin yerçekimi yönünden hesaplanan roll/pitch sonucu. */
struct AccelAngles
{
    // Gyro rad/s verdiği için attitude katmanında temel açı birimi radyandır.
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    bool valid = false;
};
