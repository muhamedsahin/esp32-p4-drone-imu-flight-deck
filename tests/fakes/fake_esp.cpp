#include "fake_esp.hpp"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

namespace fake
{
State state{};
void reset() { state = {}; }
}
int64_t esp_timer_get_time() { return fake::state.time_us; }
void vTaskDelay(TickType_t ticks) { fake::state.time_us += ticks * 10000LL; }
const char* esp_err_to_name(esp_err_t error) { return error == ESP_OK ? "OK" : "TEST_ERROR"; }
void testLog(const char*, const char* format, ...)
{
    char text[512]{};
    va_list args;
    va_start(args, format);
    std::vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (fake::state.automatic_faces && text[0] == '[' && std::strstr(text, "yonunu YUKARI"))
    {
        ++fake::state.face_requests;
        fake::state.face = text[1] - '1'; // Testte kullanıcı terminal talebini yerine getirir.
        if (fake::state.blocked_face == fake::state.face)
            fake::state.face = fake::state.face == 5 ? 0 : 5;
    }
    if (fake::state.automatic_faces && text[0] == '[' && text[1] == 'D')
    {
        ++fake::state.tilt_requests;
        fake::state.face = 6 + text[2] - '1';
        if (fake::state.blocked_tilt_pose == text[2] - '1') fake::state.face = 5;
    }
}
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t* handle)
{
    *handle = &fake::state;
    return ESP_OK;
}
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t) { ++fake::state.released_buses; return ESP_OK; }
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t) { ++fake::state.released_devices; return ESP_OK; }
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t* config,
                                  i2c_master_dev_handle_t* handle)
{
    if (fake::state.add_failure) return ESP_FAIL;
    fake::state.scl_speed_hz = config->scl_speed_hz;
    *handle = &fake::state;
    return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t* bytes, size_t size, int timeout)
{
    auto& s = fake::state;
    s.time_us += 100;
    if (s.io_failure || size != 2 || timeout < 0) return ESP_FAIL;
    ++s.register_writes[bytes[0]];
    if (bytes[0] == 0x6B && bytes[1] == 0x80) s.registers = {};
    else s.registers[bytes[0]] = bytes[1];
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t* reg, size_t write_size,
                                    uint8_t* buffer, size_t size, int timeout)
{
    auto& s = fake::state;
    s.time_us += 100;
    if (s.io_failure || write_size != 1 || timeout < 0) return ESP_FAIL;
    if (*reg == 0x75)
    {
        const size_t index = s.identity_reads++;
        buffer[0] = index < s.identity_sequence.size() ? s.identity_sequence[index] : s.identity;
        return ESP_OK;
    }
    if (*reg == 0x3A)
    {
        const int64_t sequence = s.time_us / 10000;
        buffer[0] = !s.no_ready && sequence > s.last_status_sequence ? 1 : 0;
        s.last_status_sequence = sequence;
        return ESP_OK;
    }
    if (*reg == 0x3B && size == 14)
    {
        if (s.last_burst_us)
            s.minimum_burst_gap_us = std::min(s.minimum_burst_gap_us, s.time_us - s.last_burst_us);
        s.last_burst_us = s.time_us;
        ++s.bursts;
        std::array<int16_t, 7> words{s.bias[0], s.bias[1], s.bias[2], s.temperature,
                                     s.gyro[0], s.gyro[1], s.gyro[2]};
        if (s.face < 6)
            words[static_cast<size_t>(s.face / 2)] += (s.face % 2 == 0 ? 1 : -1) * s.scale[static_cast<size_t>(s.face / 2)];
        else
        {
            constexpr size_t pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
            for (size_t axis : pairs[s.face - 6])
                words[axis] += static_cast<int16_t>(std::lround(s.scale[axis] / std::sqrt(2.0) *
                    (s.tilted_validation_failure ? 1.15 : 1.0)));
        }
        if (s.motion) words[4] = s.bursts % 2 ? 300 : -300;
        for (size_t i = 0; i < words.size(); ++i)
        {
            const auto bits = static_cast<uint16_t>(words[i]);
            buffer[2 * i] = static_cast<uint8_t>(bits >> 8);
            buffer[2 * i + 1] = static_cast<uint8_t>(bits & 0xFF);
        }
        return ESP_OK;
    }
    buffer[0] = s.registers[*reg];
    if (*reg == s.mismatch_register) buffer[0] ^= 1;
    return ESP_OK;
}
esp_err_t nvs_flash_init() { return fake::state.nvs_failure ? ESP_FAIL : ESP_OK; }
esp_err_t nvs_open(const char*, int mode, nvs_handle_t* handle)
{
    if (fake::state.nvs_failure) return ESP_FAIL;
    if (mode == NVS_READONLY && fake::state.stored.empty() && fake::state.other_stored.empty()) return ESP_ERR_NVS_NOT_FOUND;
    *handle = 1;
    return ESP_OK;
}
void nvs_close(nvs_handle_t) {}
esp_err_t nvs_get_blob(nvs_handle_t, const char* key, void* output, size_t* length)
{
    const auto& bytes = std::strcmp(key, "accel_v1") == 0 ? fake::state.stored : fake::state.other_stored[key];
    if (bytes.empty()) return ESP_ERR_NVS_NOT_FOUND;
    if (*length < bytes.size()) return ESP_FAIL;
    *length = bytes.size();
    std::memcpy(output, bytes.data(), *length);
    return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t, const char* key, const void* bytes, size_t size)
{
    const auto* start = static_cast<const uint8_t*>(bytes);
    auto& target = std::strcmp(key, "accel_v1") == 0 ? fake::state.pending : fake::state.other_pending[key];
    target.assign(start, start + size);
    return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t)
{
    if (fake::state.commit_failure) return ESP_FAIL;
    fake::state.stored = fake::state.pending;
    for (const auto& entry : fake::state.other_pending) fake::state.other_stored[entry.first] = entry.second;
    return ESP_OK;
}
