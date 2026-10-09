idf_component_register(
    SRCS "drone_dashboard.cpp" "wifi_manager.cpp" "network_config.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private"
    REQUIRES orientation esp_http_server
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer nvs_flash
)
# Dosyalar firmware'e gömülür: SD kart, internet veya harici CDN gerekmez.
target_add_binary_data(${COMPONENT_LIB} "${PROJECT_DIR}/wifi.txt" TEXT)
foreach(asset index.html styles.css app.js math.js renderer.js)
    target_add_binary_data(${COMPONENT_LIB} "web/${asset}" BINARY)
endforeach()
