#include "imu_calibration.hpp"
#include "imu_calibration_record.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace imu_calibration;
namespace
{
int checks = 0;
void check(bool condition, const char* description)
{
    ++checks;
    if (!condition) { std::cerr << "FAIL: " << description << '\n'; std::exit(1); }
}
bool close(double a, double b, double tolerance = 1e-9) { return std::abs(a - b) <= tolerance; }

std::array<Vector3, FACE_COUNT> faces(const Vector3& bias, const Vector3& scale)
{
    std::array<Vector3, FACE_COUNT> result{};
    for (size_t face = 0; face < FACE_COUNT; ++face)
    {
        result[face] = bias;
        result[face][face / 2] += (face % 2 == 0 ? 1 : -1) * scale[face / 2];
    }
    return result;
}
Window quietWindow()
{
    Window window{};
    for (size_t i = 0; i < 200; ++i)
    {
        const double noise = i % 2 ? 1 : -1;
        window.add({noise, noise, ACCEL_COUNTS_PER_G + noise},
                   {120 + noise, -30 + noise, 10 + noise}, 30.0, i < 100);
    }
    return window;
}
}

int main()
{
    Config config{};
    check(validConfig(config), "default config");
    Config bad = config;
    bad.face_samples = 0;
    check(!validConfig(bad), "zero samples rejected");
    bad = config; bad.max_gyro_std_dps = std::numeric_limits<double>::quiet_NaN();
    check(!validConfig(bad), "NaN threshold rejected");
    bad = config; bad.pose_timeout_ms = 2000;
    check(!validConfig(bad), "insufficient deadline rejected");

    RunningStats stats;
    for (double x : {1.0, 2.0, 3.0, 4.0}) stats.add(x);
    check(close(stats.mean(), 2.5), "Welford mean");
    check(close(stats.stddev(), std::sqrt(5.0 / 3.0)), "unbiased sample stddev");
    check(close(stats.range(), 3), "range");

    check(stationary(quietWindow(), config) == Status::Ok, "quiet biased gyro accepted");
    check(stationary(Window{}, config) == Status::Motion, "empty window rejected");
    Window rotating{}, vibrating{}, drifting{}, hot{}, falling{}, saturated{};
    for (size_t i = 0; i < 200; ++i)
    {
        const double sign = i % 2 ? 1 : -1;
        rotating.add({0, 0, ACCEL_COUNTS_PER_G}, {sign * 2 * GYRO_COUNTS_PER_DPS, 0, 0}, 30, i < 100);
        vibrating.add({sign * 0.1 * ACCEL_COUNTS_PER_G, 0, ACCEL_COUNTS_PER_G}, {0, 0, 0}, 30, i < 100);
        drifting.add({0, 0, ACCEL_COUNTS_PER_G}, {i * 0.002 * GYRO_COUNTS_PER_DPS, 0, 0}, 30, i < 100);
        hot.add({0, 0, ACCEL_COUNTS_PER_G}, {0, 0, 0}, 30 + i * 0.01, i < 100);
        falling.add({0, 0, 0}, {0, 0, 0}, 30, i < 100);
        saturated.add({0, 0, ACCEL_COUNTS_PER_G}, {i == 50 ? 32767.0 : 0, 0, 0}, 30, i < 100);
    }
    check(stationary(rotating, config) == Status::Motion, "rotation rejected");
    check(stationary(vibrating, config) == Status::Motion, "vibration rejected");
    check(stationary(drifting, config) == Status::Motion, "slow gyro drift rejected");
    check(stationary(hot, config) == Status::TemperatureUnstable, "thermal transition rejected");
    check(stationary(falling, config) == Status::Motion, "free fall rejected");
    check(stationary(saturated, config) == Status::Saturation, "one clipped sample rejects window");
    Window invalid{};
    for (size_t i = 0; i < 30; ++i)
        invalid.add({0, 0, std::numeric_limits<double>::quiet_NaN()}, {0, 0, 0}, 30, i < 15);
    check(stationary(invalid, config) == Status::Motion, "nonfinite measurements rejected");

    const Vector3 bias{123.25, -84.5, 205.75};
    const Vector3 scale{16400.5, 16290.25, 16510.75};
    const auto training = faces(bias, scale);
    auto validation = training;
    // Ayrı doğrulama penceresine küçük ölçüm hatası ekle.
    for (auto& mean : validation) for (auto& axis : mean) axis += 2.0;
    AccelCalibration model{};
    check(fitAccelerometer(training, validation, config, model) == Status::Ok, "six-face fit");
    for (size_t axis = 0; axis < 3; ++axis)
    {
        check(close(model.bias_counts[axis], bias[axis]), "fractional bias recovered");
        check(close(model.counts_per_g[axis], scale[axis]), "scale recovered");
    }
    const auto arbitrary = correctAcceleration(
        {bias[0] + 0.6 * scale[0], bias[1] - 0.8 * scale[1], bias[2]}, model);
    check(close(arbitrary[0], 0.6) && close(arbitrary[1], -0.8) && close(arbitrary[2], 0),
          "independent tilted gravity vector corrected");
    check(model.calibrated && model.validation_max_g > 0, "measured quality reported");

    AccelCalibration unchanged = model;
    auto wrong = training;
    wrong[0] = wrong[4];
    check(fitAccelerometer(wrong, validation, config, model) == Status::WrongPose, "duplicate/wrong face");
    check(model.bias_counts == unchanged.bias_counts && model.calibrated, "failed fit preserves active model");
    wrong = validation; wrong[0][0] += 0.08 * ACCEL_COUNTS_PER_G;
    check(fitAccelerometer(training, wrong, config, model) == Status::ValidationFailed,
          "independent validation drift rejected");
    // Yaklaşık konumlar: her örnek gerçekten 1g, ancak diğer eksenler sıfır değil.
    auto tilted = training;
    for (size_t face = 0; face < FACE_COUNT; ++face)
    {
        Vector3 direction{0.055, -0.070, 0.060};
        direction[face / 2] = (face % 2 == 0 ? 1.0 : -1.0);
        const double length = std::sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
        for (size_t axis = 0; axis < 3; ++axis) tilted[face][axis] = bias[axis] + scale[axis] * direction[axis] / length;
    }
    AccelCalibration tilted_model{};
    check(fitAccelerometer(tilted, tilted, config, tilted_model) == Status::Ok,
          "imperfect alignment fits gravity norm without weakening threshold");
    for (size_t axis = 0; axis < 3; ++axis)
        check(close(tilted_model.bias_counts[axis], bias[axis], 1e-5) &&
              close(tilted_model.counts_per_g[axis], scale[axis], 1e-5), "tilted poses recover known bias and scale");
    check(close(accelerationNorm({bias[0]+0.6*scale[0],bias[1]-0.8*scale[1],bias[2]}, tilted_model),1,1e-8),
          "new tilted pose independent of training has unit gravity");
    check(matchesTiltedPose({11585,11585,0},0) && !matchesTiltedPose({0,0,16384},0),
          "held-out XY plane cannot be replaced by axis aligned pose");
    check(!matchesTiltedPose({11585,11585,0},3), "invalid tilted pose index rejected");
    // Son gerçek logdaki altı ortalama. Aynı veriyi validation olarak kullanmak
    // sadece eski retin regresyonudur; fiziksel doğruluk iddiası değildir.
    const std::array<Vector3,6> recorded{{{17026.517,298.017,691.553},
        {-15858.650,228.863,701.800},{637.297,16536.657,650.523},
        {314.840,-16296.500,1945.550},{705.850,298.503,17855.100},
        {1744.217,-238.690,-15572.347}}};
    check(fitAccelerometer(recorded, recorded, config, tilted_model) == Status::Ok,
          "user recorded tilted faces no longer rejected for transverse gravity");
    wrong = training; wrong[0][0] = wrong[1][0];
    check(fitAccelerometer(wrong, validation, config, model) == Status::WrongPose, "zero scale cannot be fitted");
    const auto huge_scale = faces({0, 0, 0}, {1.2 * ACCEL_COUNTS_PER_G, scale[1], scale[2]});
    check(fitAccelerometer(huge_scale, huge_scale, config, model) == Status::InvalidModel,
          "implausible sensitivity rejected");
    wrong = training; wrong[0][0] = std::numeric_limits<double>::infinity();
    check(fitAccelerometer(wrong, validation, config, model) == Status::WrongPose, "infinite face rejected");

    AccelCalibration decoded{};
    model.reference_temperature_c = 32.5;
    auto record = makeRecord(model);
    check(decodeRecord(record, decoded), "record round trip");
    check(close(decoded.bias_counts[0], bias[0], 1e-4) && decoded.calibrated, "record coefficients");
    const auto saved = decoded;
    record.values[0] += 1;
    check(!decodeRecord(record, decoded), "corrupt payload CRC rejected");
    check(decoded.bias_counts == saved.bias_counts, "failed load preserves model");
    record = makeRecord(model); record.version += 1;
    check(!decodeRecord(record, decoded), "different schema rejected");
    record = makeRecord(model); record.profile ^= 1;
    check(!decodeRecord(record, decoded), "different sensor settings rejected");
    const auto record6500 = makeRecord(model, 0x70);
    check(decodeRecord(record6500, decoded, 0x70), "MPU6500 profile round trip");
    check(!decodeRecord(record6500, decoded, 0x68), "MPU6500 record cannot load as MPU6050");
    check(!decodeRecord(makeRecord(model, 0x68), decoded, 0x70), "MPU6050 record cannot load as MPU6500");
    check(!decodeRecord(makeRecord(model, 0x71), decoded, 0x71), "unknown sensor profile rejected");
    auto impossible = model; impossible.counts_per_g[0] = 0;
    check(!decodeRecord(makeRecord(impossible), decoded), "CRC-valid zero sensitivity rejected");
    impossible = model; impossible.bias_counts[0] = std::numeric_limits<double>::quiet_NaN();
    check(!decodeRecord(makeRecord(impossible), decoded), "CRC-valid NaN rejected");
    impossible = model; impossible.validation_max_g = 0.09;
    check(!decodeRecord(makeRecord(impossible), decoded), "old low-quality record rejected");

    std::cout << checks << " calibration checks passed\n";
    return 0;
}
