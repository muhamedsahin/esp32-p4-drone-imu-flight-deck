#include "imu_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace imu_calibration
{
const char* faceName(Face face)
{
    constexpr const char* names[] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
    const auto index = static_cast<size_t>(face);
    return index < FACE_COUNT ? names[index] : "?";
}

const char* statusName(Status status)
{
    switch (status)
    {
    case Status::Ok: return "basarili";
    case Status::NotInitialized: return "sensor baslatilmadi";
    case Status::InvalidConfiguration: return "gecersiz kalibrasyon ayari";
    case Status::IoError: return "I2C okuma/yazma hatasi";
    case Status::Timeout: return "sure asimi / yeni veri gelmedi";
    case Status::Motion: return "hareket, titresim veya drift";
    case Status::WrongPose: return "istenen eksen yukari bakmiyor";
    case Status::Saturation: return "sensor olcum araligi asildi";
    case Status::TemperatureUnstable: return "sicaklik kararsiz";
    case Status::InvalidModel: return "bias/scale fiziksel sinirlar disinda";
    case Status::ValidationFailed: return "bagimsiz dogrulama gecemedi";
    case Status::StorageError: return "kalici bellek hatasi";
    case Status::NoStoredCalibration: return "kayitli accelerometer kalibrasyonu yok";
    }
    return "bilinmeyen hata";
}

bool validConfig(const Config& c)
{
    const double thresholds[] = {
        c.max_accel_std_g, c.max_accel_range_g, c.max_accel_drift_g,
        c.max_gyro_std_dps, c.max_gyro_range_dps, c.max_gyro_drift_dps,
        c.max_gyro_bias_dps, c.max_temperature_span_c,
        c.max_session_temperature_span_c, c.max_gyro_validation_dps,
        c.max_accel_validation_g
    };
    for (double t : thresholds)
        if (!std::isfinite(t) || t <= 0) return false;
    if (c.warmup_ms > 300000 || c.pose_timeout_ms < 1000 || c.pose_timeout_ms > 600000)
        return false;
    const uint32_t counts[] = {c.settle_samples, c.gyro_samples, c.face_samples, c.validation_samples};
    for (uint32_t count : counts)
        if (count < 20 || count > 20000) return false;
    const uint32_t longest = c.settle_samples + std::max(c.gyro_samples, c.face_samples)
                             + c.validation_samples;
    return c.pose_timeout_ms > longest * 10U;
}

void RunningStats::add(double value)
{
    if (count_ == 0) minimum_ = maximum_ = value;
    minimum_ = std::min(minimum_, value);
    maximum_ = std::max(maximum_, value);
    ++count_;
    const double delta = value - mean_;
    mean_ += delta / static_cast<double>(count_);
    m2_ += delta * (value - mean_);
}

double RunningStats::stddev() const
{
    return count_ > 1 ? std::sqrt(std::max(0.0, m2_ / static_cast<double>(count_ - 1))) : 0.0;
}

void Window::add(const Vector3& a, const Vector3& g, double t, bool first_half)
{
    temperature.add(t);
    for (size_t axis = 0; axis < 3; ++axis)
    {
        accel[axis].add(a[axis]);
        gyro[axis].add(g[axis]);
        (first_half ? first_accel[axis] : second_accel[axis]).add(a[axis]);
        (first_half ? first_gyro[axis] : second_gyro[axis]).add(g[axis]);
        saturated |= std::abs(a[axis]) >= 32700 || std::abs(g[axis]) >= 32700;
    }
}

Vector3 Window::accelMean() const { return {accel[0].mean(), accel[1].mean(), accel[2].mean()}; }
Vector3 Window::gyroMean() const { return {gyro[0].mean(), gyro[1].mean(), gyro[2].mean()}; }

Status stationary(const Window& w, const Config& c)
{
    if (!validConfig(c)) return Status::InvalidConfiguration;
    if (w.temperature.count() < 20) return Status::Motion;
    if (w.saturated) return Status::Saturation;
    if (!std::isfinite(w.temperature.mean()) || !std::isfinite(w.temperature.range()) ||
        w.temperature.mean() < -40 || w.temperature.mean() > 85 ||
        w.temperature.range() > c.max_temperature_span_c)
        return Status::TemperatureUnstable;
    double gravity_squared = 0;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        const auto& a = w.accel[axis];
        const auto& g = w.gyro[axis];
        if (!std::isfinite(a.mean()) || !std::isfinite(g.mean()) ||
            !std::isfinite(a.stddev()) || !std::isfinite(g.stddev()) ||
            a.count() != w.temperature.count() || g.count() != w.temperature.count() ||
            w.first_accel[axis].count() == 0 || w.second_accel[axis].count() == 0)
            return Status::Motion;
        const double accel_g = a.mean() / ACCEL_COUNTS_PER_G;
        gravity_squared += accel_g * accel_g;
        if (a.stddev() / ACCEL_COUNTS_PER_G > c.max_accel_std_g ||
            a.range() / ACCEL_COUNTS_PER_G > c.max_accel_range_g ||
            g.stddev() / GYRO_COUNTS_PER_DPS > c.max_gyro_std_dps ||
            g.range() / GYRO_COUNTS_PER_DPS > c.max_gyro_range_dps ||
            std::abs(g.mean()) / GYRO_COUNTS_PER_DPS > c.max_gyro_bias_dps ||
            std::abs(w.first_accel[axis].mean() - w.second_accel[axis].mean()) /
                ACCEL_COUNTS_PER_G > c.max_accel_drift_g ||
            std::abs(w.first_gyro[axis].mean() - w.second_gyro[axis].mean()) /
                GYRO_COUNTS_PER_DPS > c.max_gyro_drift_dps)
            return Status::Motion;
    }
    // Serbest düşüşü / büyük doğrusal ivmeyi sabit duruş kabul etme.
    return gravity_squared >= 0.75 * 0.75 && gravity_squared <= 1.25 * 1.25
        ? Status::Ok : Status::Motion;
}

bool matchesFace(const Vector3& a, Face face)
{
    const size_t index = static_cast<size_t>(face);
    if (index >= FACE_COUNT) return false;
    const size_t axis = index / 2;
    const double sign = index % 2 == 0 ? 1.0 : -1.0;
    for (double value : a) if (!std::isfinite(value)) return false;
    const double along = sign * a[axis] / ACCEL_COUNTS_PER_G;
    if (along < 0.75 || along > 1.25) return false;
    for (size_t other = 0; other < 3; ++other)
        if (other != axis && std::abs(a[other]) / ACCEL_COUNTS_PER_G > 0.15) return false;
    return true;
}

Vector3 correctAcceleration(const Vector3& raw, const AccelCalibration& c)
{
    Vector3 corrected{};
    for (size_t axis = 0; axis < 3; ++axis)
        corrected[axis] = (raw[axis] - c.bias_counts[axis]) / c.counts_per_g[axis];
    return corrected;
}

double accelerationNorm(const Vector3& raw, const AccelCalibration& c)
{
    const auto a = correctAcceleration(raw, c);
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

bool matchesTiltedPose(const Vector3& raw, size_t pose)
{
    constexpr size_t pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
    if (pose >= 3) return false;
    for (size_t axis = 0; axis < 3; ++axis)
    {
        const double value = std::abs(raw[axis]) / ACCEL_COUNTS_PER_G;
        if (!std::isfinite(value)) return false;
        const bool in_plane = axis == pairs[pose][0] || axis == pairs[pose][1];
        if (in_plane ? (value < 0.30 || value > 1.05) : value > 0.30) return false;
    }
    return true;
}

namespace
{
// Normalize count değerleriyle çalış: bilinmeyen bias yaklaşık 0, scale yaklaşık 1.
// Altı parametre: üç bias ve üç hassasiyet. Kusursuz yön vektörü yerine
// (x-bx)^2/sx^2 + (y-by)^2/sy^2 + (z-bz)^2/sz^2 = 1 kullanılır.
using Parameters = std::array<double, 6>;
double normCost(const std::array<Vector3, FACE_COUNT>& data, const Parameters& p)
{
    double cost = 0;
    for (const auto& sample : data)
    {
        double residual = -1;
        for (size_t axis = 0; axis < 3; ++axis)
        {
            const double q = (sample[axis] / ACCEL_COUNTS_PER_G - p[axis]) / p[axis + 3];
            residual += q * q;
        }
        cost += residual * residual;
    }
    return cost;
}

bool solveStep(double matrix[6][7], Parameters& step)
{
    // Partial pivoting; tekillik/bozuk yön kapsamı başarı sayılmaz.
    for (size_t column = 0; column < 6; ++column)
    {
        size_t pivot = column;
        for (size_t row = column + 1; row < 6; ++row)
            if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) pivot = row;
        if (!std::isfinite(matrix[pivot][column]) || std::abs(matrix[pivot][column]) < 1e-6)
            return false;
        for (size_t j = column; j < 7; ++j) std::swap(matrix[column][j], matrix[pivot][j]);
        for (size_t row = column + 1; row < 6; ++row)
        {
            const double factor = matrix[row][column] / matrix[column][column];
            for (size_t j = column; j < 7; ++j) matrix[row][j] -= factor * matrix[column][j];
        }
    }
    for (int row = 5; row >= 0; --row)
    {
        double value = matrix[row][6];
        for (size_t j = static_cast<size_t>(row + 1); j < 6; ++j) value -= matrix[row][j] * step[j];
        step[static_cast<size_t>(row)] = value / matrix[row][row];
        if (!std::isfinite(step[static_cast<size_t>(row)])) return false;
    }
    return true;
}

bool refineNormModel(const std::array<Vector3, FACE_COUNT>& train, AccelCalibration& model)
{
    Parameters p{};
    for (size_t axis = 0; axis < 3; ++axis)
    {
        p[axis] = model.bias_counts[axis] / ACCEL_COUNTS_PER_G;
        p[axis + 3] = model.counts_per_g[axis] / ACCEL_COUNTS_PER_G;
    }
    // Sınırlı Newton + backtracking. Sonsuz iterasyon veya fizik dışı fit yok.
    for (unsigned iteration = 0; iteration < 25; ++iteration)
    {
        const double cost = normCost(train, p);
        if (!std::isfinite(cost)) return false;
        if (cost < 1e-20)
        {
            for (size_t axis = 0; axis < 3; ++axis)
            {
                model.bias_counts[axis] = p[axis] * ACCEL_COUNTS_PER_G;
                model.counts_per_g[axis] = p[axis + 3] * ACCEL_COUNTS_PER_G;
            }
            return validModel(model);
        }
        double matrix[6][7]{};
        for (size_t face = 0; face < FACE_COUNT; ++face)
        {
            double residual = -1;
            for (size_t axis = 0; axis < 3; ++axis)
            {
                const double d = train[face][axis] / ACCEL_COUNTS_PER_G - p[axis];
                const double s = p[axis + 3];
                residual += d * d / (s * s);
                matrix[face][axis] = -2 * d / (s * s);
                matrix[face][axis + 3] = -2 * d * d / (s * s * s);
            }
            matrix[face][6] = -residual;
        }
        Parameters step{};
        if (!solveStep(matrix, step)) return false;
        bool accepted = false;
        double fraction = 1;
        for (unsigned trial = 0; trial < 16; ++trial, fraction *= 0.5)
        {
            Parameters next{};
            bool bounded = true;
            for (size_t i = 0; i < 6; ++i) next[i] = p[i] + fraction * step[i];
            for (size_t axis = 0; axis < 3; ++axis)
                bounded &= std::abs(next[axis]) <= 0.15 && next[axis + 3] >= 0.85 && next[axis + 3] <= 1.15;
            if (bounded && normCost(train, next) < cost) { p = next; accepted = true; break; }
        }
        if (!accepted) return false;
    }
    return false;
}
}

bool validModel(const AccelCalibration& c)
{
    for (size_t axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(c.bias_counts[axis]) || !std::isfinite(c.counts_per_g[axis]) ||
            std::abs(c.bias_counts[axis]) > 0.15 * ACCEL_COUNTS_PER_G ||
            c.counts_per_g[axis] < 0.85 * ACCEL_COUNTS_PER_G ||
            c.counts_per_g[axis] > 1.15 * ACCEL_COUNTS_PER_G) return false;
    }
    return std::isfinite(c.reference_temperature_c) && c.reference_temperature_c >= -40 &&
           c.reference_temperature_c <= 85 && std::isfinite(c.validation_rms_g) &&
           std::isfinite(c.validation_max_g) && c.validation_rms_g >= 0 &&
           c.validation_max_g + 1e-9 >= c.validation_rms_g && c.validation_max_g <= 0.10;
}

Status fitAccelerometer(const std::array<Vector3, FACE_COUNT>& train,
                        const std::array<Vector3, FACE_COUNT>& validation,
                        const Config& config, AccelCalibration& output)
{
    if (!validConfig(config)) return Status::InvalidConfiguration;
    AccelCalibration candidate{};
    for (size_t face = 0; face < FACE_COUNT; ++face)
        if (!matchesFace(train[face], static_cast<Face>(face)) ||
            !matchesFace(validation[face], static_cast<Face>(face))) return Status::WrongPose;

    // +1g/-1g uçlarının ortası bias, yarı farkı count/g hassasiyetidir.
    // Ortalamayı int16_t'ye yuvarlamıyoruz; kesirli bilgi korunuyor.
    for (size_t axis = 0; axis < 3; ++axis)
    {
        candidate.bias_counts[axis] = (train[2 * axis][axis] + train[2 * axis + 1][axis]) / 2;
        candidate.counts_per_g[axis] = (train[2 * axis][axis] - train[2 * axis + 1][axis]) / 2;
    }
    if (!validModel(candidate)) return Status::InvalidModel;
    if (!refineNormModel(train, candidate)) return Status::InvalidModel;

    double squared_error = 0;
    for (size_t face = 0; face < FACE_COUNT; ++face)
    {
        // Eğik konumda diğer eksenlerin sıfır olması beklenmez. Gravity normunu
        // eğitimden ayrı pencerede doğrula; bu bir yön/açı doğrulaması değildir.
        const double error = std::abs(accelerationNorm(validation[face], candidate) - 1.0);
        if (!std::isfinite(error)) return Status::ValidationFailed;
        candidate.validation_max_g = std::max(candidate.validation_max_g, error);
        squared_error += error * error;
    }
    candidate.validation_rms_g = std::sqrt(squared_error / FACE_COUNT);
    if (candidate.validation_max_g > config.max_accel_validation_g || !validModel(candidate))
        return Status::ValidationFailed;
    candidate.calibrated = true;
    output = candidate;
    return Status::Ok;
}
} // namespace imu_calibration
