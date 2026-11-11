idf_component_register(SRCS "main.cpp" "net_state.cpp" "ntp_time.cpp" "settings.cpp" "web_server.cpp"
                       REQUIRES settingsbase webserver jsonwrapper nvsstoragemanager
                       PRIV_REQUIRES esp_eth esp_netif esp_http_server nvs_flash
                                     esp_app_format app_update esp_timer lwip
                       INCLUDE_DIRS ".")
