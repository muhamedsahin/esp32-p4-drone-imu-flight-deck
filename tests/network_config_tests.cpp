#include "network_config.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
unsigned checks = 0;
void check(bool condition, const char* message) {
    ++checks;
    if (!condition) { std::printf("FAIL: %s\n", message); std::exit(1); }
}
}
int main() {
    drone_web::NetworkConfig config;
    std::string error;
    auto parse = [&](const std::string& text) { return drone_web::parseNetworkConfig(text, config, error); };
    check(parse("ssid=null\r\npassword=null\r\n"), "Windows CRLF null config");
    check(!config.stationEnabled(), "null selects AP");
    check(config.ap_ssid == "Drone-IMU" && config.ap_password == "drone6050", "AP defaults");
    check(parse("\xEF\xBB\xBFssid = \"Home WiFi\" # comment\npassword = \"a#b=c123\"\n"), "UTF8 BOM, spaces and literal #");
    check(config.stationEnabled() && config.password == "a#b=c123", "STA chosen without corrupting password");
    check(parse("ssid=\"Home\"\npassword=null\n"), "partial null accepted");
    check(!config.stationEnabled(), "one null selects AP");
    check(parse("ssid=null\npassword=\"abcdefgh\"\n"), "other partial null accepted");
    check(!config.stationEnabled(), "SSID null selects AP");
    check(parse("ssid=\"" + std::string(32, 's') + "\"\npassword=\"" + std::string(63, 'p') + "\"\nconnect_timeout_s=120"), "boundary lengths");
    const auto old_ssid = config.ssid;
    check(!parse("ssid=\"" + std::string(33, 's') + "\"\npassword=null"), "oversize SSID rejected");
    check(config.ssid == old_ssid, "output unchanged on failure");
    check(!parse("ssid=null\npassword=\"short\""), "short password rejected");
    check(!parse("ssid=null\npassword=null\nap_password=null"), "insecure AP rejected");
    check(!parse("ssid=null\npassword=null\nap_ssid=null"), "empty AP SSID rejected");
    check(!parse("ssid=null\npassword=null\nssid=null"), "duplicate key rejected");
    check(!parse("ssid=null\npassword=null\nsecret_key=\"SECRET\""), "unknown key rejected");
    check(error.find("SECRET") == std::string::npos, "errors do not disclose values");
    check(!parse("ssid=null"), "missing password rejected");
    check(!parse("ssid=null\npassword=null\nconnect_timeout_s=4"), "timeout too short rejected");
    check(!parse("ssid=null\npassword=null\nconnect_timeout_s=30oops"), "trailing numeric garbage rejected");
    check(!parse("ssid=Home\npassword=null"), "unquoted SSID rejected");
    check(!parse("ssid=\"null\"\npassword=\"bad\npassword\""), "multiline injection rejected");
    check(parse("ssid=null\npassword=null\nap_ssid=\"Test\"\nap_password=\"newpass12\"\nconnect_timeout_s=5"), "AP configuration override");
    check(config.ap_ssid == "Test" && config.connect_timeout_s == 5, "override applied");
    std::printf("PASS: %u Wi-Fi configuration checks\n", checks);
}
