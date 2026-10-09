#pragma once
#include "network_config.hpp"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

namespace drone_web {
/** Tek ağ görevi kullanır. Olay geri çağrısı sadece durum biti koyar. */
class WifiManager {
public:
    bool begin(const NetworkConfig& config);
    void maintain();
private:
    static void event(void* context, esp_event_base_t base, int32_t id, void* data);
    bool startAccessPoint();
    NetworkConfig config_{};
    EventGroupHandle_t events_ = nullptr;
    esp_netif_t* station_ = nullptr;
    esp_netif_t* ap_ = nullptr;
    bool ap_mode_ = false;
    int64_t disconnected_since_us_ = 0;
    int64_t next_retry_us_ = 0;
};
}
