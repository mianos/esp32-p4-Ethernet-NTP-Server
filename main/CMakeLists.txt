idf_component_register(SRCS "clock_source.cpp" "eth_link.cpp" "gps_time.cpp" "main.cpp" "net_log.cpp" "ntp_control.cpp" "net_state.cpp" "ntp_server.cpp" "ntp_time.cpp" "settings.cpp" "tx_lag.cpp" "ubx.cpp" "web_server.cpp"
                       REQUIRES settingsbase webserver jsonwrapper nvsstoragemanager
                       PRIV_REQUIRES esp_eth esp_netif esp_http_server nvs_flash
                                     esp_app_format app_update esp_timer lwip
                                     esp_driver_uart esp_driver_gpio esp_driver_mcpwm
                       INCLUDE_DIRS ".")

# IDF's -Wall leaves these as warnings, and dead code then only surfaces when a
# change happens to sit next to it. Members and published fields stay invisible
# to the compiler either way; this catches the function-and-variable half.
target_compile_options(${COMPONENT_LIB} PRIVATE
                       -Werror=unused-function -Werror=unused-variable
                       -Werror=unused-but-set-variable -Werror=unused-const-variable=1)
