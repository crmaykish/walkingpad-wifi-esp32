idf_component_register(
    SRCS "walkingpad_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_wifi esp_netif esp_event nvs_flash esp_http_server esp_timer app_update
)
