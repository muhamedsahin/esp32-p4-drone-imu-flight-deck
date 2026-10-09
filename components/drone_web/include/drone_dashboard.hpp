#pragma once
#include "orientation_system.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"

/**
 * Salt okunur web paneli. start() yalnız bir ağ görevini başlatır.
 * publish() IMU görevinden çağrılır; Wi-Fi/HTTP beklemeden kısa bir kopya alır.
 * Nesne uygulama ömrü boyunca yaşamalıdır (main'de static kullanılır).
 */
class DroneDashboard {
public:
    bool start();
    void publish(const OrientationData& orientation);
    DroneDashboard() = default;
    DroneDashboard(const DroneDashboard&) = delete;
    DroneDashboard& operator=(const DroneDashboard&) = delete;
private:
    static void networkTask(void* context);
    static esp_err_t telemetryHandler(httpd_req_t* request);
    bool startServer();
    portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
    OrientationData latest_{};
    uint32_t sequence_ = 0;
    TaskHandle_t network_task_ = nullptr;
    httpd_handle_t server_ = nullptr;
};
