#include "orientation_system.hpp"
#include "fake_esp.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Gerçek adapter + sahte I2C/NVS ile API'nin tüm hazırlık akışı test edilir.
// Karttaki calibration.txt ve kalibrasyon kayıtları bu testten etkilenmez.
extern const char calibration_source[] = "format = 1\nsensor = 104\nx = null\n";

namespace
{
int checks = 0;
void check(bool condition, const char* label)
{
    ++checks;
    if (!condition)
    {
        std::printf("FAIL: %s\n", label);
        std::exit(1);
    }
}

IMUData sample()
{
    IMUData data{};
    data.az = 1;
    data.valid = data.accel_calibrated = data.gyro_calibrated = true;
    data.gyro_temperature_valid = true;
    data.timestamp_us = 1'000'000;
    return data;
}

bool sameQuaternion(const Quaternion& a, const Quaternion& b)
{
    return a.w == b.w && a.x == b.x && a.y == b.y && a.z == b.z;
}

void externalMeasurementApi()
{
    fake::reset();
    OrientationSystem orientation;
    check(!orientation.getOrientation().valid, "no valid output before first sample");
    check(!orientation.update(), "sensor read requires init");

    IMUData data = sample();
    data.valid = false;
    check(!orientation.update(data), "invalid initial external sample rejected");
    data.valid = true;
    check(orientation.update(data), "external sample initializes without opening hardware");
    auto result = orientation.getOrientation();
    check(result.valid && result.angles.roll_rad == 0 && result.angles.pitch_rad == 0 &&
          result.angles.yaw_rad == 0, "first sample produces flat reference");
    check(fake::state.bursts == 0 && fake::state.identity_reads == 0,
          "external update never reads I2C");

    data.timestamp_us += 10'000;
    data.gz = 0.5f;
    check(orientation.update(data), "external gyro update succeeds");
    result = orientation.getOrientation();
    check(result.valid && std::abs(result.angles.yaw_rad - 0.005f) < 1e-6f,
          "API exposes updated quaternion-derived yaw");
    check(result.measurement.timestamp_us == data.timestamp_us &&
          result.measurement.gz == data.gz, "angles and measurement refer to same update");
    const OrientationData saved = result;
    result.angles.yaw_rad = 99;
    result.quaternion.w = 99;
    check(orientation.getOrientation().angles.yaw_rad == saved.angles.yaw_rad &&
          sameQuaternion(orientation.getOrientation().quaternion, saved.quaternion),
          "returned copy cannot alter internal state");

    check(!orientation.update(data), "API rejects repeated timestamp");
    result = orientation.getOrientation();
    check(!result.valid && sameQuaternion(result.quaternion, saved.quaternion) &&
          result.measurement.timestamp_us == saved.measurement.timestamp_us,
          "failed update marks preserved output as stale");

    data.timestamp_us += 1'000'000;
    check(!orientation.update(data) && !orientation.getOrientation().valid,
          "large gap is rejected by underlying estimator");
    data.timestamp_us += 10'000;
    check(orientation.update(data) && orientation.getOrientation().valid,
          "API resumes on next normal sample after gap");
}

void sensorReadingApi()
{
    fake::reset();
    {
        OrientationSystem orientation;
        check(orientation.init(), "API initializes sensor and completes missing calibration");
        check(fake::state.face_requests == 6 && fake::state.tilt_requests == 3,
              "existing six-face and holdout workflow remains active");
        check(!orientation.getOrientation().valid, "init waits for first orientation measurement");
        const int bursts_after_init = fake::state.bursts;
        check(orientation.init() && fake::state.bursts == bursts_after_init,
              "repeated init does not rerun successful calibration");

        fake::state.face = 4; // Kalibrasyon sonrasında sensörü düz +Z konumuna koy.
        fake::state.time_us += 10'000;
        check(orientation.update(), "API reads calibrated sensor and initializes orientation");
        const OrientationData saved = orientation.getOrientation();
        check(saved.valid && std::abs(saved.angles.roll_rad) < 1e-4f &&
              std::abs(saved.angles.pitch_rad) < 1e-4f && saved.measurement.accel_calibrated &&
              saved.measurement.gyro_calibrated, "sensor API returns calibrated flat attitude");

        fake::state.io_failure = true;
        check(!orientation.update(), "I2C read failure propagates to caller");
        const auto failed = orientation.getOrientation();
        check(!failed.valid && sameQuaternion(failed.quaternion, saved.quaternion),
              "read failure cannot publish stale attitude as valid");
        fake::state.io_failure = false;
        fake::state.time_us += 10'000;
        check(orientation.update(), "reading recovers after transient I2C failure");
    }
    check(fake::state.released_buses == 1 && fake::state.released_devices == 1,
          "API destructor releases owned I2C resources");

    // Aynı kaynak dosyasıyla reboot: kaydedilmiş ilerleme tekrar ölçülmemeli.
    fake::state.face_requests = fake::state.tilt_requests = 0;
    {
        OrientationSystem reboot;
        check(reboot.init(), "new API instance loads saved calibration");
        check(fake::state.face_requests == 0 && fake::state.tilt_requests == 0,
              "complete stored calibration is not repeated");
    }

    fake::reset();
    {
        OrientationSystem retry;
        fake::state.add_failure = true;
        check(!retry.init() && !retry.update(), "failed sensor init prevents automatic reads");
        fake::state.add_failure = false;
        check(retry.init(), "init can retry after temporary sensor setup failure");
    }
}
} // namespace

int main()
{
    externalMeasurementApi();
    sensorReadingApi();
    std::printf("%d orientation API checks passed\n", checks);
}
