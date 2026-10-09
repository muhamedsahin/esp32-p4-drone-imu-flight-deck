#include "imu_calibration_text.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <locale>
#include <sstream>
#include <vector>

namespace imu_calibration
{
namespace
{
struct Field
{
    const char *name;
    double *values;
    size_t count;
    bool *present;
};
template <size_t N> Field field(const char *name, TextValue<N> &v)
{
    return {name, v.value.data(), N, &v.present};
}
std::vector<Field> fields(TextCalibration &s)
{
    return {field("x", s.axes[0]),
            field("y", s.axes[1]),
            field("z", s.axes[2]),
            field("positive_x", s.faces[0]),
            field("negative_x", s.faces[1]),
            field("positive_y", s.faces[2]),
            field("negative_y", s.faces[3]),
            field("positive_z", s.faces[4]),
            field("negative_z", s.faces[5]),
            field("accel_temperature", s.accel_temperature),
            field("accel_quality", s.accel_quality),
            field("validation_xy", s.tilts[0]),
            field("validation_xz", s.tilts[1]),
            field("validation_yz", s.tilts[2]),
            field("gyro_x", s.gyro_axes[0]),
            field("gyro_y", s.gyro_axes[1]),
            field("gyro_z", s.gyro_axes[2]),
            field("gyro_temperature", s.gyro_temperature),
            field("gyro_noise", s.gyro_noise),
            field("gyro_error", s.gyro_error)};
}
std::string trim(const std::string &s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    return first == std::string::npos ? "" : s.substr(first, s.find_last_not_of(" \t\r\n") - first + 1);
}
bool numbers(const std::string &text, double *values, size_t count)
{
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    for (size_t i = 0; i < count; ++i)
    {
        if (!(input >> values[i]) || !std::isfinite(values[i]))
            return false;
        if (i + 1 < count)
        {
            char comma = 0;
            if (!(input >> comma) || comma != ',')
                return false;
        }
    }
    input >> std::ws;
    return input.eof();
}
} // namespace

bool parseCalibrationText(const std::string &text, TextCalibration &output, std::string &error)
{
    if (text.size() > 8192)
    {
        error = "Dosya 8192 byte sinirini asti.";
        return false;
    }
    TextCalibration candidate{};
    auto list = fields(candidate);
    std::vector<std::string> seen;
    std::istringstream lines(text);
    std::string line;
    bool version = false, sensor = false;
    size_t number = 0;
    while (std::getline(lines, line))
    {
        ++number;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty())
            continue;
        const auto equal = line.find('=');
        if (equal == std::string::npos)
        {
            error = "Eksik '='; satir " + std::to_string(number);
            return false;
        }
        const auto key = trim(line.substr(0, equal)), value = trim(line.substr(equal + 1));
        if (std::find(seen.begin(), seen.end(), key) != seen.end())
        {
            error = "Tekrarlanan anahtar: " + key;
            return false;
        }
        seen.push_back(key);
        if (key == "format" || key == "sensor")
        {
            double v = 0;
            if (!numbers(value, &v, 1) || (key == "format" ? v != 1 : (v != 104 && v != 112)))
            {
                error = "Gecersiz format/sensor: " + key;
                return false;
            }
            if (key == "format")
                version = true;
            else
            {
                sensor = true;
                candidate.identity = static_cast<uint8_t>(v);
            }
            continue;
        }
        const auto f =
            std::find_if(list.begin(), list.end(), [&key](const Field &item) { return key == item.name; });
        if (f == list.end())
        {
            error = "Bilinmeyen anahtar: " + key;
            return false;
        }
        if (value == "null")
            continue;
        if (!numbers(value, f->values, f->count))
        {
            error = "Gecersiz sayi/vektor: " + key;
            return false;
        }
        *f->present = true;
    }
    if (!version || !sensor)
    {
        error = "format ve sensor zorunludur.";
        return false;
    }
    output = candidate;
    error.clear();
    return true;
}

std::string calibrationText(const TextCalibration &state)
{
    auto copy = state;
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << "# x/y/z: bias[count], hassasiyet[count/g]. null = eksik.\n"
        << "# positive/negative: ham X,Y,Z [count]. gyro: bias[count].\n"
        << "# accel_quality: RMS,max [g]. validation: iki pencere hatasi [g].\n"
        << "# Sicaklik [C], gyro_noise/gyro_error [dps].\n"
        << "format = 1\nsensor = " << unsigned(state.identity) << '\n'
        << std::setprecision(17);
    for (const auto &f : fields(copy))
    {
        out << f.name << " = ";
        if (!*f.present)
            out << "null";
        else
            for (size_t i = 0; i < f.count; ++i)
                out << (i ? ", " : "") << f.values[i];
        out << '\n';
    }
    return out.str();
}

bool completedAccelerometer(const TextCalibration &s, AccelCalibration &output)
{
    if (!s.accel_quality.present || !s.accel_temperature.present)
        return false;
    AccelCalibration c{};
    for (size_t i = 0; i < 3; ++i)
    {
        if (!s.axes[i].present)
            return false;
        c.bias_counts[i] = s.axes[i].value[0];
        c.counts_per_g[i] = s.axes[i].value[1];
    }
    c.reference_temperature_c = s.accel_temperature.value[0];
    c.validation_rms_g = s.accel_quality.value[0];
    c.validation_max_g = s.accel_quality.value[1];
    if (!validModel(c) || c.validation_max_g > Config{}.max_accel_validation_g)
        return false;
    c.calibrated = true;
    output = c;
    return true;
}

bool completedGyroscope(const TextCalibration &s, GyroCalibration &output)
{
    if (!s.gyro_temperature.present || !s.gyro_noise.present || !s.gyro_error.present)
        return false;
    GyroCalibration c{};
    c.reference_temperature_c = s.gyro_temperature.value[0];
    c.validation_max_dps = s.gyro_error.value[0];
    if (c.reference_temperature_c < -40 || c.reference_temperature_c > 85 || c.validation_max_dps < 0 ||
        c.validation_max_dps > Config{}.max_gyro_validation_dps)
        return false;
    for (size_t i = 0; i < 3; ++i)
    {
        if (!s.gyro_axes[i].present ||
            std::abs(s.gyro_axes[i].value[0]) / GYRO_COUNTS_PER_DPS > Config{}.max_gyro_bias_dps ||
            s.gyro_noise.value[i] < 0 || s.gyro_noise.value[i] > Config{}.max_gyro_std_dps)
            return false;
        c.bias_counts[i] = s.gyro_axes[i].value[0];
        c.noise_dps[i] = s.gyro_noise.value[i];
    }
    c.calibrated = true;
    output = c;
    return true;
}

void applyCalibrationEdits(TextCalibration &s)
{
    // Tamamlanmış modelin bir katsayısını null yapmak, ilgili yüz çiftini yeniler.
    // İlk log kurtarmasında quality zaten null: mevcut yüzler korunur.
    if (!s.accel_quality.present)
        return;
    bool changed = false;
    for (size_t i = 0; i < 3; ++i)
        if (!s.axes[i].present)
        {
            s.faces[2 * i].present = s.faces[2 * i + 1].present = false;
            changed = true;
        }
    for (const auto &face : s.faces)
        if (!face.present)
            changed = true;
    if (changed)
    {
        s.accel_quality.present = false;
        s.accel_temperature.present = false;
        for (auto &axis : s.axes)
            axis.present = false; // Katsayılar birlikte yeniden fit edilir.
        for (auto &tilt : s.tilts)
            tilt.present = false;
    }
}

uint32_t textChecksum(const std::string &text)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (unsigned char byte : text)
    {
        crc ^= byte;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320U : 0U);
    }
    return ~crc;
}
} // namespace imu_calibration
