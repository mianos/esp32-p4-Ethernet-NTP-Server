idf_component_register(SRCS "main.c" "settings.c" "web_server.c"
                       PRIV_REQUIRES esp_eth esp_netif esp_http_server nvs_flash
                                     esp_app_format app_update esp_timer
                       INCLUDE_DIRS ".")
