#include "mpu6050.hpp"
#include "fake_esp.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
int checks = 0;
void check(bool condition, const char* message)
{
    ++checks;
    if (!condition) { std::printf("FAIL: %s\n", message); std::exit(1); }
}
imu_calibration::Config fastConfig()
{
    imu_calibration::Config config{};
    config.warmup_ms = 0;
    config.settle_samples = config.gyro_samples = config.face_samples = config.validation_samples = 20;
    config.pose_timeout_ms = 1500;
    return config;
}
}
int main()
{
    using imu_calibration::Status;
    fake::reset();
    {
        MPU6050 sensor;
        RawIMUData raw{};
        check(!sensor.readRaw(raw) && !raw.valid, "uninitialized read invalid");
        fake::state.add_failure = true;
        check(!sensor.init(), "device registration error cannot report success");
        check(fake::state.released_buses == 1, "failed registration cleans bus");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.identity = 0x69;
        check(!sensor.init(), "wrong identity rejected");
        check(fake::state.released_devices == 1 && fake::state.released_buses == 1, "identity failure cleanup");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.io_failure = true;
        check(!sensor.init(), "identity I2C failure rejected");
        check(fake::state.register_writes[0x6B] == 0, "no reset before successful identity read");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.identity_sequence = {0x68, 0x70};
        check(!sensor.init(), "unstable identity cannot select a model");
        check(fake::state.register_writes[0x6B] == 0, "unstable identity cannot reset an unknown device");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.identity_sequence = {0x68, 0x68, 0x68, 0x70, 0x70, 0x70};
        check(!sensor.init(), "changed identity after reset rejected");
        check(sensor.sensorIdentity() == 0 && fake::state.released_buses == 1,
              "reset identity failure clears model and releases bus");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.identity = 0x70;
        fake::state.mismatch_register = 0x1D;
        check(!sensor.init(), "MPU6500 accel filter readback failure rejected");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.mismatch_register = 0x1C;
        check(!sensor.init(), "wrong scale readback rejected");
    }
    fake::reset();
    {
        MPU6050 sensor;
        check(sensor.init() && sensor.init(), "initialization is idempotent");
        check(sensor.sensorIdentity() == 0x68 && std::strcmp(sensor.sensorName(), "MPU6050") == 0,
              "MPU6050 identified explicitly");
        check(fake::state.identity_reads == 6, "stable identity checked before and after reset");
        check(fake::state.scl_speed_hz == 100000, "jumper wiring uses 100 kHz I2C");
        check(fake::state.register_writes[0x1D] == 0, "MPU6050 reserved register untouched");
        check(fake::state.registers[0x6B] == 1 && fake::state.registers[0x19] == 9 &&
              fake::state.registers[0x1A] == 3, "wake PLL rate and filter configured");
        RawIMUData first{}, second{};
        check(sensor.readRaw(first) && sensor.readRaw(second), "fresh signed raw reads");
        check(first.ay == -80 && first.gy == -30 && first.temperature == -2200, "negative big-endian parsing");
        check(second.timestamp_us - first.timestamp_us >= 10000, "duplicate samples cannot be counted");
        const auto before_timeout = fake::state.time_us;
        fake::state.no_ready = true;
        check(!sensor.readRaw(second) && !second.valid, "data-ready timeout invalid");
        check(fake::state.time_us - before_timeout < 120000, "read timeout bounded");
        fake::state.no_ready = false;
        fake::state.io_failure = true;
        IMUData physical{};
        check(!sensor.read(physical) && !physical.valid, "I2C failure is not a corrected zero measurement");
        fake::state.io_failure = false;
        auto config = fastConfig();
        check(sensor.calibrateGyroscope(config), "gyro workflow succeeds");
        check(sensor.gyroscopeCalibration().bias_counts[0] == 120, "gyro bias from actual acquisition path");
        const auto good_gyro = sensor.gyroscopeCalibration().bias_counts;
        fake::state.io_failure = true;
        check(!sensor.calibrateGyroscope(config) && sensor.calibrationReport().status == Status::IoError,
              "calibration aborts on I2C failure");
        check(sensor.gyroscopeCalibration().bias_counts == good_gyro, "I2C failure preserves gyro bias");
        fake::state.io_failure = false;
        fake::state.motion = true;
        check(!sensor.calibrateGyroscope(config), "motion cannot calibrate gyro");
        check(sensor.gyroscopeCalibration().bias_counts == good_gyro, "failed gyro preserves good bias");
        fake::state.motion = false;
        check(!sensor.loadAccelerometerCalibration() && sensor.calibrationReport().status == Status::NoStoredCalibration,
              "empty NVS distinguishable");
        check(sensor.calibrateAccelerometer(config), "six-face acquisition and fit succeeds");
        check(sensor.calibrationReport().completed_faces == 6, "all six faces required");
        check(sensor.calibrationReport().verified_tilted_poses == 3, "three held-out tilted poses required");
        check(fake::state.minimum_burst_gap_us >= 10000, "all calibration bursts are fresh");
        check(sensor.accelerometerCalibration().bias_counts[0] == 120 &&
              sensor.accelerometerCalibration().counts_per_g[0] == 16400, "acquired coefficients recovered");
        check(sensor.saveAccelerometerCalibration(), "NVS write commit and readback");
        fake::state.face = 5;
        check(sensor.read(physical) && std::abs(physical.az + 1.0f) < 1e-6f &&
              std::abs(physical.gx) < 1e-6f, "physical units and bias applied");
        check(std::abs(physical.temperature_c - (-2200.0 / 340.0 + 36.53)) < 1e-5,
              "MPU6050 temperature conversion preserved");
        fake::state.temperature += 2040; // +6 °C; ölçülmemiş sıcaklık düzeltmesi uygulanmamalı.
        check(sensor.read(physical) && !physical.gyro_temperature_valid,
              "gyro temperature departure explicitly flagged");
        fake::state.temperature -= 2040;
        fake::state.gyro[0] = 32767;
        check(!sensor.read(physical) && !physical.valid, "clipped physical sample invalid");
        fake::state.gyro[0] = 120;
        const auto good_accel = sensor.accelerometerCalibration().bias_counts;
        fake::state.tilted_validation_failure = true;
        check(!sensor.calibrateAccelerometer(config, false) && sensor.calibrationReport().status == Status::ValidationFailed,
              "new tilted pose detects a bad model despite successful six-face fit");
        check(sensor.accelerometerCalibration().bias_counts == good_accel,
              "failed tilted validation preserves good calibration");
        fake::state.tilted_validation_failure = false;
        fake::state.face = 5;
        fake::state.automatic_faces = false;
        check(!sensor.calibrateAccelerometer(config, false) && sensor.calibrationReport().status == Status::Timeout,
              "wrong face times out");
        check(sensor.accelerometerCalibration().bias_counts == good_accel, "failed six-face preserves good model");
        fake::state.commit_failure = true;
        check(!sensor.saveAccelerometerCalibration() && sensor.calibrationReport().status == Status::StorageError,
              "commit failure reported");
        fake::state.commit_failure = false;
    }
    check(fake::state.released_devices == 1 && fake::state.released_buses == 1, "destructor releases bus and device");
    {
        MPU6050 sensor;
        check(sensor.init() && sensor.loadAccelerometerCalibration(), "calibration persists across object lifetime");
        const auto good = sensor.accelerometerCalibration().bias_counts;
        fake::state.stored[16] ^= 1;
        check(!sensor.loadAccelerometerCalibration(), "corrupt NVS data rejected");
        check(sensor.accelerometerCalibration().bias_counts == good, "failed NVS load preserves good model");
        fake::state.nvs_failure = true;
        check(!sensor.loadAccelerometerCalibration() && sensor.calibrationReport().status == Status::StorageError,
              "NVS initialization failure reported without erasing");
    }
    fake::reset();
    {
        MPU6050 sensor;
        fake::state.identity = 0x70;
        fake::state.temperature = 3005; // MPU6500: yaklaşık 30 °C.
        check(sensor.init() && sensor.init(), "MPU6500 initialization and repeat init succeed");
        check(sensor.sensorIdentity() == 0x70 && std::strcmp(sensor.sensorName(), "MPU6500") == 0,
              "0x70 selects MPU6500 model");
        check(fake::state.registers[0x1D] == 3 && fake::state.register_writes[0x1D] == 1,
              "MPU6500 has separate accel DLPF configured");
        check(fake::state.registers[0x1B] == 0 && fake::state.registers[0x19] == 9,
              "MPU6500 gyro DLPF enabled at 100 Hz and 250 dps");
        IMUData physical{};
        const double expected_c = 3005.0 / 333.87 + 21.0;
        check(sensor.read(physical) && std::abs(physical.temperature_c - expected_c) < 1e-5,
              "MPU6500 physical temperature uses its own scale");
        const auto config = fastConfig();
        check(sensor.calibrateGyroscope(config) &&
              std::abs(sensor.gyroscopeCalibration().reference_temperature_c - expected_c) < 1e-9,
              "MPU6500 gyro calibration uses matching temperature scale");
        check(sensor.read(physical) && physical.gyro_temperature_valid && std::abs(physical.gx) < 1e-6,
              "MPU6500 calibrated units and temperature flag valid");
        fake::state.temperature += 2004; // MPU6500 ölçeğinde yaklaşık +6 °C.
        check(sensor.read(physical) && !physical.gyro_temperature_valid,
              "MPU6500 temperature departure detected");
        fake::state.temperature -= 2004;
        check(sensor.calibrateAccelerometer(config) && sensor.calibrationReport().completed_faces == 6,
              "MPU6500 six-face workflow succeeds");
        check(std::abs(sensor.accelerometerCalibration().reference_temperature_c - expected_c) < 1e-9,
              "MPU6500 accel reference temperature correct");
        check(sensor.saveAccelerometerCalibration(), "MPU6500 NVS profile save/readback succeeds");
    }
    {
        MPU6050 sensor;
        check(sensor.init() && sensor.loadAccelerometerCalibration(), "MPU6500 calibration survives reboot");
    }
    // CRC'si doğru olsa bile diğer sensör modelinin kalibrasyonu yüklenmemeli.
    fake::state.identity = 0x68;
    {
        MPU6050 sensor;
        check(sensor.init() && !sensor.loadAccelerometerCalibration() &&
              sensor.calibrationReport().status == Status::InvalidModel,
              "driver rejects saved calibration from another sensor model");
    }
    fake::reset();
    fake::state.identity = 0x70;
    {
        MPU6050 sensor;
        check(sensor.init() && sensor.calibrateGyroscope(fastConfig()), "recovery sensor initialized at current temperature");
        std::array<imu_calibration::Vector3,6> means{};
        for (size_t face = 0; face < 6; ++face)
        {
            for (size_t axis = 0; axis < 3; ++axis) means[face][axis] = fake::state.bias[axis];
            means[face][face/2] += (face%2==0 ? 1 : -1) * fake::state.scale[face/2];
        }
        check(sensor.recoverAccelerometerFromLog(means,fastConfig()), "six logged training means recovered into pending record");
        check(!sensor.accelerometerCalibration().calibrated && fake::state.stored.empty(),
              "log recovery does not create a validated final calibration");
        fake::state.blocked_tilt_pose = 1;
        check(!sensor.calibrateAccelerometer(fastConfig()) && sensor.calibrationReport().status == Status::Timeout,
              "recovered session can pause at second tilted pose");
        check(fake::state.face_requests == 0 && sensor.calibrationReport().verified_tilted_poses == 1,
              "recovered session skips six faces and checkpoints accepted XY");
    }
    {
        MPU6050 sensor;
        fake::state.blocked_tilt_pose = -1;
        fake::state.face_requests = fake::state.tilt_requests = 0;
        check(sensor.init() && sensor.calibrateGyroscope(fastConfig()) && sensor.calibrateAccelerometer(fastConfig()),
              "pending calibration resumes successfully after object reboot");
        check(fake::state.face_requests == 0 && fake::state.tilt_requests == 2,
              "resuming does not repeat six faces or completed XY validation");
        check(sensor.calibrationReport().verified_tilted_poses == 3 && sensor.saveAccelerometerCalibration(),
              "recovered calibration activates and saves only after all three fresh tilted checks");
    }
    {
        MPU6050 sensor;
        fake::state.other_stored["pending_v1"][16] ^= 1;
        check(sensor.init() && !sensor.calibrateAccelerometer(fastConfig()) &&
              sensor.calibrationReport().status == Status::InvalidModel, "corrupted progress rejected without silently restarting six faces");
    }
    std::printf("%d driver/workflow checks passed\n", checks);
}
