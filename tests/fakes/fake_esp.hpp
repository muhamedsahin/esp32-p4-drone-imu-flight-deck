#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <map>
#include <string>

// Sadece host testleri: gerçek firmware derlemesi gerçek ESP-IDF başlıklarını kullanır.
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0, ESP_FAIL = -1, ESP_ERR_NVS_NOT_FOUND = 2;
using TickType_t = uint32_t;
constexpr TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms / 10; }
void vTaskDelay(TickType_t ticks);
int64_t esp_timer_get_time();
const char* esp_err_to_name(esp_err_t error);
void testLog(const char* tag, const char* format, ...);
#define ESP_LOGI(...) testLog(__VA_ARGS__)
#define ESP_LOGW(...) testLog(__VA_ARGS__)
#define ESP_LOGE(...) testLog(__VA_ARGS__)

using i2c_master_bus_handle_t = void*;
using i2c_master_dev_handle_t = void*;
constexpr int I2C_NUM_0 = 0, GPIO_NUM_7 = 7, GPIO_NUM_8 = 8;
constexpr int I2C_CLK_SRC_DEFAULT = 0, I2C_ADDR_BIT_LEN_7 = 0;
struct i2c_master_bus_config_t
{
    int i2c_port = 0, sda_io_num = 0, scl_io_num = 0, clk_source = 0;
    uint8_t glitch_ignore_cnt = 0;
    struct { bool enable_internal_pullup = false; } flags;
};
struct i2c_device_config_t
{
    int dev_addr_length = 0;
    uint16_t device_address = 0;
    uint32_t scl_speed_hz = 0;
};
esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t*, i2c_master_bus_handle_t*);
esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t);
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t*);
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t*, size_t, int);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t*, size_t, uint8_t*, size_t, int);

using nvs_handle_t = uint32_t;
constexpr int NVS_READONLY = 0, NVS_READWRITE = 1;
esp_err_t nvs_flash_init();
esp_err_t nvs_open(const char*, int, nvs_handle_t*);
void nvs_close(nvs_handle_t);
esp_err_t nvs_get_blob(nvs_handle_t, const char*, void*, size_t*);
esp_err_t nvs_set_blob(nvs_handle_t, const char*, const void*, size_t);
esp_err_t nvs_commit(nvs_handle_t);

namespace fake
{
struct State
{
    int64_t time_us = 1000000;
    int64_t last_status_sequence = 0;
    int64_t last_burst_us = 0;
    int64_t minimum_burst_gap_us = 100000000;
    std::array<uint8_t, 256> registers{};
    std::array<unsigned, 256> register_writes{};
    std::array<int16_t, 3> bias{120, -80, 200}, scale{16400, 16300, 16500};
    std::array<int16_t, 3> gyro{120, -30, 10};
    int face = 4, released_buses = 0, released_devices = 0, bursts = 0;
    int mismatch_register = -1;
    bool add_failure = false, io_failure = false, no_ready = false;
    bool automatic_faces = true, motion = false;
    bool tilted_validation_failure = false;
    int blocked_tilt_pose = -1, face_requests = 0, tilt_requests = 0;
    int blocked_face = -1;
    bool nvs_failure = false, commit_failure = false;
    uint8_t identity = 0x68;
    std::vector<uint8_t> identity_sequence{};
    size_t identity_reads = 0;
    uint32_t scl_speed_hz = 0;
    int16_t temperature = -2200;
    std::vector<uint8_t> stored{}, pending{};
    std::map<std::string, std::vector<uint8_t>> other_stored{}, other_pending{};
};
extern State state;
void reset();
}
