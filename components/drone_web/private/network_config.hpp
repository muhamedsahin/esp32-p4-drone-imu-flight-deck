#pragma once
#include <string>
#include <string_view>

namespace drone_web {
struct NetworkConfig {
    std::string ssid;
    std::string password;
    std::string ap_ssid = "Drone-IMU";
    std::string ap_password = "drone6050";
    unsigned connect_timeout_s = 30;
    bool stationEnabled() const { return !ssid.empty() && !password.empty(); }
};
/** Katı TXT okuyucu. Hata halinde çıktı değişmez, hata metni sır içermez. */
bool parseNetworkConfig(std::string_view text, NetworkConfig& output, std::string& error);
}
