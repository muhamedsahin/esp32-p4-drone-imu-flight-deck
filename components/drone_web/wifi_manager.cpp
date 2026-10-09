#include "wifi_manager.hpp"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <cstring>

namespace drone_web {
namespace {
constexpr char TAG[] = "DRONE_WIFI";
constexpr EventBits_t CONNECTED = BIT0;
bool check(esp_err_t error, const char* operation) {
    if (error == ESP_OK) return true;
    ESP_LOGE(TAG, "%s: %s. IMU olcumu devam eder.", operation, esp_err_to_name(error));
    return false;
}
}
void WifiManager::event(void* context, esp_event_base_t base, int32_t id, void* data) {
    auto& self = *static_cast<WifiManager*>(context);
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto& event_data = *static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Panel adresi: http://" IPSTR, IP2STR(&event_data.ip_info.ip));
        xEventGroupSetBits(self.events_, CONNECTED);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(self.events_, CONNECTED);
    }
    // RPC/esp_wifi_connect çağrısını olay görevinden yapmıyoruz: ağ görevi yapar.
}
bool WifiManager::begin(const NetworkConfig& config) {
    config_ = config;
    // Kalibrasyon aynı NVS'yi kullanır. Hata halinde ASLA flash erase yapma.
    if (!check(nvs_flash_init(), "NVS") || !check(esp_netif_init(), "Netif")) return false;
    auto result = esp_event_loop_create_default();
    if (result != ESP_ERR_INVALID_STATE && !check(result, "Event loop")) return false;
    events_ = xEventGroupCreate();
    if (!events_) return false;
    station_ = esp_netif_create_default_wifi_sta();
    ap_ = esp_netif_create_default_wifi_ap();
    if (!station_ || !ap_) return false;
    if (!check(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event, this), "Wi-Fi events") ||
        !check(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event, this), "IP events")) return false;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    // ESP-Hosted, P4'ün üzerindeki C6'yı SDIO üzerinden bu API'ye bağlar.
    if (!check(esp_wifi_init(&init), "C6 Wi-Fi init")) {
        ESP_LOGE(TAG, "C6 ESP-Hosted firmware/surum ve SDIO pinlerini kontrol et; C6'yi rastgele flashlama.");
        return false;
    }
    if (!check(esp_wifi_set_storage(WIFI_STORAGE_RAM), "Wi-Fi RAM storage")) return false;
    if (!config_.stationEnabled()) return startAccessPoint();
    wifi_config_t wifi{};
    std::memcpy(wifi.sta.ssid, config_.ssid.data(), config_.ssid.size());
    std::memcpy(wifi.sta.password, config_.password.data(), config_.password.size());
    wifi.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi.sta.pmf_cfg.capable = true;
    if (!check(esp_wifi_set_mode(WIFI_MODE_STA), "STA mode") ||
        !check(esp_wifi_set_config(WIFI_IF_STA, &wifi), "STA config") ||
        !check(esp_wifi_start(), "STA start")) return false;
    disconnected_since_us_ = esp_timer_get_time();
    ESP_LOGI(TAG, "Kayitli 2.4 GHz agina baglaniliyor; %u sn sonra AP yedegi.", config_.connect_timeout_s);
    // Ağ hazır olmasa da HTTP sunucusu açılır; IP alınca erişilebilir hale gelir.
    maintain();
    return true;
}
bool WifiManager::startAccessPoint() {
    wifi_config_t wifi{};
    std::memcpy(wifi.ap.ssid, config_.ap_ssid.data(), config_.ap_ssid.size());
    std::memcpy(wifi.ap.password, config_.ap_password.data(), config_.ap_password.size());
    wifi.ap.ssid_len = static_cast<uint8_t>(config_.ap_ssid.size());
    wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi.ap.channel = 6;
    wifi.ap.max_connection = 4;
    // STA'dan AP'ye geçerken önce radyoyu durdur. STA yeniden başlatılmaz.
    if (config_.stationEnabled() && !check(esp_wifi_stop(), "STA stop")) return false;
    if (!check(esp_wifi_set_mode(WIFI_MODE_AP), "AP mode") ||
        !check(esp_wifi_set_config(WIFI_IF_AP, &wifi), "AP config") ||
        !check(esp_wifi_start(), "AP start")) return false;
    ap_mode_ = true;
    esp_netif_ip_info_t info{};
    esp_netif_get_ip_info(ap_, &info);
    ESP_LOGI(TAG, "Kendi agi: %s | Panel: http://" IPSTR, config_.ap_ssid.c_str(), IP2STR(&info.ip));
    return true;
}
void WifiManager::maintain() {
    if (ap_mode_) return;
    const int64_t now = esp_timer_get_time();
    if (xEventGroupGetBits(events_) & CONNECTED) { disconnected_since_us_ = 0; return; }
    if (!disconnected_since_us_) disconnected_since_us_ = now;
    if (now - disconnected_since_us_ >= static_cast<int64_t>(config_.connect_timeout_s) * 1'000'000) {
        ESP_LOGW(TAG, "STA zaman asimi; kendi agina geciliyor.");
        if (!startAccessPoint()) disconnected_since_us_ = now; // Hata fırtınası olmasın.
    } else if (now >= next_retry_us_) {
        const auto error = esp_wifi_connect();
        if (error != ESP_OK) ESP_LOGW(TAG, "Baglanti denemesi: %s", esp_err_to_name(error));
        next_retry_us_ = now + 5'000'000;
    }
}
}
