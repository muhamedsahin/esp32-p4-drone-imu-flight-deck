#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// ESP-IDF'ye bağlı değildir. Matematik bilgisayarda da test edilebilir.
namespace imu_calibration
{
using Vector3 = std::array<double, 3>;
constexpr double ACCEL_COUNTS_PER_G = 16384.0;  // ACCEL_CONFIG: ±2g
constexpr double GYRO_COUNTS_PER_DPS = 131.0;   // GYRO_CONFIG: ±250°/s
constexpr double DEGREES_TO_RADIANS = 0.017453292519943295;

enum class Face : uint8_t { PositiveX, NegativeX, PositiveY, NegativeY, PositiveZ, NegativeZ };
constexpr size_t FACE_COUNT = 6;
const char* faceName(Face face);

enum class Status : uint8_t
{
    Ok, NotInitialized, InvalidConfiguration, IoError, Timeout,
    Motion, WrongPose, Saturation, TemperatureUnstable,
    InvalidModel, ValidationFailed, StorageError, NoStoredCalibration
};
const char* statusName(Status status);

/** Eşikler fiziksel birimdedir. Örnek sayısını artırmak tek başına kaliteyi artırmaz. */
struct Config
{
    uint32_t warmup_ms = 20000;
    uint32_t settle_samples = 100;    // Her pozisyonda önce ~1 s sabit duruş.
    uint32_t gyro_samples = 1000;    // 100 Hz'de en az ~10 s.
    uint32_t face_samples = 300;     // Her yüzde ~3 s eğitim.
    uint32_t validation_samples = 300; // Eğitimden ayrı ~3 s doğrulama.
    uint32_t pose_timeout_ms = 120000;
    double max_accel_std_g = 0.010;
    double max_accel_range_g = 0.080;
    double max_accel_drift_g = 0.015;
    double max_gyro_std_dps = 0.30;
    double max_gyro_range_dps = 2.5;
    double max_gyro_drift_dps = 0.12;
    double max_gyro_bias_dps = 20.0;
    double max_temperature_span_c = 0.8;
    double max_session_temperature_span_c = 3.0;
    double max_gyro_validation_dps = 0.15;
    double max_accel_validation_g = 0.035;
};
bool validConfig(const Config& config);

/** Welford: büyük toplamların farkını almadan kararlı ortalama/varyans. O(1) RAM. */
class RunningStats
{
public:
    void add(double value);
    size_t count() const { return count_; }
    double mean() const { return mean_; }
    double stddev() const;
    double range() const { return maximum_ - minimum_; }
    double minimum() const { return minimum_; }
    double maximum() const { return maximum_; }
private:
    size_t count_ = 0;
    double mean_ = 0, m2_ = 0, minimum_ = 0, maximum_ = 0;
};

/** Kesintisiz pencere; hareketi gizlemek için tek tek uç değerler çıkarılmaz. */
struct Window
{
    std::array<RunningStats, 3> accel{}, gyro{};
    std::array<RunningStats, 3> first_accel{}, second_accel{}, first_gyro{}, second_gyro{};
    RunningStats temperature{};
    bool saturated = false;
    void add(const Vector3& accel_counts, const Vector3& gyro_counts,
             double temperature_c, bool first_half);
    Vector3 accelMean() const;
    Vector3 gyroMean() const;
};
Status stationary(const Window& window, const Config& config);
// İlk yön kontrolü ham değerlerle yapılır; henüz bias/scale bilinmiyor.
bool matchesFace(const Vector3& raw_accel, Face face);
// Ek doğrulama: XY, XZ, YZ düzlemlerinde yaklaşık eğik üç ayrı duruş.
bool matchesTiltedPose(const Vector3& raw_accel, size_t pose);

struct AccelCalibration
{
    Vector3 bias_counts{};
    Vector3 counts_per_g{ACCEL_COUNTS_PER_G, ACCEL_COUNTS_PER_G, ACCEL_COUNTS_PER_G};
    double reference_temperature_c = 0;
    double validation_rms_g = 0;
    double validation_max_g = 0;
    bool calibrated = false;
};
struct GyroCalibration
{
    Vector3 bias_counts{};
    Vector3 noise_dps{};
    double reference_temperature_c = 0;
    double validation_max_dps = 0;
    bool calibrated = false;
};
struct Report
{
    Status status = Status::NotInitialized;
    uint32_t accepted_samples = 0;
    uint32_t rejected_windows = 0;
    uint8_t completed_faces = 0;
    uint8_t verified_tilted_poses = 0;
    double worst_accel_error_g = 0;
    double worst_gyro_error_dps = 0;
};

Vector3 correctAcceleration(const Vector3& raw, const AccelCalibration& calibration);
double accelerationNorm(const Vector3& raw, const AccelCalibration& calibration);
bool validModel(const AccelCalibration& calibration);
// Altı yaklaşık yönle |a|=1g fit edilir; kusursuz 90° hizalama gerekmez.
// Tam 3x3 eksen matrisi öğrenilmez. Oturum ayrıca yeni eğik konumları doğrular.
Status fitAccelerometer(const std::array<Vector3, FACE_COUNT>& training,
                        const std::array<Vector3, FACE_COUNT>& validation,
                        const Config& config, AccelCalibration& output);
} // namespace imu_calibration
