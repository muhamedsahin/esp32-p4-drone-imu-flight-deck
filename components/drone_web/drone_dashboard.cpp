#include "drone_dashboard.hpp"
#include "wifi_manager.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr char TAG[] = "DRONE_WEB";
extern const char wifi_text[] asm("_binary_wifi_txt_start");
#define WEB_ASSET(name, symbol) \
    extern const unsigned char name##_start[] asm("_binary_" symbol "_start"); \
    extern const unsigned char name##_end[] asm("_binary_" symbol "_end")
WEB_ASSET(index, "index_html");
WEB_ASSET(styles, "styles_css");
WEB_ASSET(app, "app_js");
WEB_ASSET(math, "math_js");
WEB_ASSET(renderer, "renderer_js");
struct Asset { const char* path; const char* type; const unsigned char* begin; const unsigned char* end; };
const Asset ASSETS[]{
    {"/", "text/html; charset=utf-8", index_start, index_end},
    {"/styles.css", "text/css; charset=utf-8", styles_start, styles_end},
    {"/app.js", "text/javascript; charset=utf-8", app_start, app_end},
    {"/math.js", "text/javascript; charset=utf-8", math_start, math_end},
    {"/renderer.js", "text/javascript; charset=utf-8", renderer_start, renderer_end}
};
void responseHeaders(httpd_req_t* request) {
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    httpd_resp_set_hdr(request, "X-Content-Type-Options", "nosniff");
    httpd_resp_set_hdr(request, "Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; connect-src 'self'; img-src 'self' data:; object-src 'none'; frame-ancestors 'none'");
}
esp_err_t assetHandler(httpd_req_t* request) {
    const auto& asset = *static_cast<const Asset*>(request->user_ctx);
    responseHeaders(request);
    httpd_resp_set_type(request, asset.type);
    // Flash'tan parça parça: büyük HTML/JS için heap kopyası yaratılmaz.
    const auto* cursor = asset.begin;
    while (cursor < asset.end) {
        const size_t remaining = static_cast<size_t>(asset.end - cursor);
        const size_t size = remaining > 2048 ? 2048 : remaining;
        if (httpd_resp_send_chunk(request, reinterpret_cast<const char*>(cursor), size) != ESP_OK) return ESP_FAIL;
        cursor += size;
    }
    return httpd_resp_send_chunk(request, nullptr, 0);
}
double number(float value) { return std::isfinite(value) ? static_cast<double>(value) : 0.0; }
}

bool DroneDashboard::start() {
    if (network_task_) return true;
    // Ölçüm görevinden daha düşük öncelik. Ağ başlatma ana programı bekletmez.
    return xTaskCreate(networkTask, "drone_web", 8192, this, 3, &network_task_) == pdPASS;
}
void DroneDashboard::publish(const OrientationData& orientation) {
    portENTER_CRITICAL(&lock_);
    latest_ = orientation;
    ++sequence_;
    portEXIT_CRITICAL(&lock_);
}
void DroneDashboard::networkTask(void* context) {
    auto& self = *static_cast<DroneDashboard*>(context);
    // Olay geri çağrıları bu nesneyi kullanır; başarısız başlangıçta da yaşar.
    static drone_web::WifiManager wifi;
    drone_web::NetworkConfig config;
    std::string error;
    if (!drone_web::parseNetworkConfig(wifi_text, config, error)) {
        ESP_LOGE(TAG, "wifi.txt gecersiz: %s Varsayilan guvenli AP kullaniliyor.", error.c_str());
        config = drone_web::NetworkConfig{};
    }
    if (wifi.begin(config) && self.startServer()) {
        for (;;) { wifi.maintain(); vTaskDelay(pdMS_TO_TICKS(250)); }
    }
    ESP_LOGE(TAG, "Web paneli baslatilamadi. IMU ve kalibrasyon calismaya devam eder.");
    vTaskDelete(nullptr);
}
bool DroneDashboard::startServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    config.task_priority = 3;
    config.max_uri_handlers = 8;
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 2;
    config.send_wait_timeout = 2;
    if (httpd_start(&server_, &config) != ESP_OK) return false;
    for (const auto& asset : ASSETS) {
        httpd_uri_t route{};
        route.uri = asset.path;
        route.method = HTTP_GET;
        route.handler = assetHandler;
        route.user_ctx = const_cast<Asset*>(&asset); // HTTP API void* ister; veri değişmez.
        if (httpd_register_uri_handler(server_, &route) != ESP_OK) {
            httpd_stop(server_); server_ = nullptr; return false;
        }
    }
    httpd_uri_t telemetry{};
    telemetry.uri = "/api/telemetry";
    telemetry.method = HTTP_GET;
    telemetry.handler = telemetryHandler;
    telemetry.user_ctx = this;
    if (httpd_register_uri_handler(server_, &telemetry) != ESP_OK) {
        httpd_stop(server_); server_ = nullptr; return false;
    }
    ESP_LOGI(TAG, "Salt okunur 3D panel hazir. API: /api/telemetry (~20 Hz istemci).");
    return true;
}
esp_err_t DroneDashboard::telemetryHandler(httpd_req_t* request) {
    auto& self = *static_cast<DroneDashboard*>(request->user_ctx);
    OrientationData sample;
    uint32_t sequence;
    // Hesaplama ve HTTP gönderimi kritik bölümün DIŞINDA yapılır.
    portENTER_CRITICAL(&self.lock_);
    sample = self.latest_;
    sequence = self.sequence_;
    portEXIT_CRITICAL(&self.lock_);
    const auto& imu = sample.measurement;
    const auto& q = sample.quaternion;
    const auto& a = sample.angles;
    const int64_t now = esp_timer_get_time();
    const int64_t age = imu.timestamp_us > 0 && imu.timestamp_us <= now ? (now - imu.timestamp_us) / 1000 : -1;
    const bool finite = std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
        std::isfinite(a.roll_rad) && std::isfinite(a.pitch_rad) && std::isfinite(a.yaw_rad) &&
        std::isfinite(imu.ax) && std::isfinite(imu.ay) && std::isfinite(imu.az) &&
        std::isfinite(imu.gx) && std::isfinite(imu.gy) && std::isfinite(imu.gz) && std::isfinite(imu.temperature_c);
    const bool valid = sample.valid && imu.valid && finite && age >= 0 && age <= 500;
    // Sabit buffer: her HTTP isteğinde JSON heap tahsisi yapmayız. NaN JSON'a yazılmaz.
    char json[1024];
    const int size = std::snprintf(json, sizeof(json),
        "{\"valid\":%s,\"sequence\":%lu,\"timestamp_us\":%lld,\"age_ms\":%lld,"
        "\"quaternion\":[%.7g,%.7g,%.7g,%.7g],\"angles_rad\":[%.7g,%.7g,%.7g],"
        "\"accel_g\":[%.7g,%.7g,%.7g],\"gyro_rad_s\":[%.7g,%.7g,%.7g],\"temperature_c\":%.5g,"
        "\"accel_calibrated\":%s,\"gyro_calibrated\":%s,\"gyro_temperature_valid\":%s}",
        valid ? "true" : "false", static_cast<unsigned long>(sequence),
        static_cast<long long>(imu.timestamp_us), static_cast<long long>(age),
        number(q.w), number(q.x), number(q.y), number(q.z),
        number(a.roll_rad), number(a.pitch_rad), number(a.yaw_rad),
        number(imu.ax), number(imu.ay), number(imu.az), number(imu.gx), number(imu.gy), number(imu.gz),
        number(imu.temperature_c), imu.accel_calibrated ? "true" : "false",
        imu.gyro_calibrated ? "true" : "false", imu.gyro_temperature_valid ? "true" : "false");
    if (size < 0 || size >= static_cast<int>(sizeof(json))) return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "Telemetry buffer");
    responseHeaders(request);
    httpd_resp_set_type(request, "application/json");
    return httpd_resp_send(request, json, size);
}
