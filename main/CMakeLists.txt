idf_component_register(SRCS "app/config_apply.cpp"
                            "app/console.cpp"
                            "app/main.cpp"
                            "app/settings.cpp"
                            "clock/clock_source.cpp"
                            "clock/ntp_time.cpp"
                            "gps/gps_bridge.cpp"
                            "gps/gps_frames.cpp"
                            "gps/gps_receiver.cpp"
                            "gps/gps_servo.cpp"
                            "gps/gps_status.cpp"
                            "gps/gps_time.cpp"
                            "gps/ubx.cpp"
                            "http/web_server.cpp"
                            "net/eth_link.cpp"
                            "net/net_addr.cpp"
                            "net/net_log.cpp"
                            "net/net_state.cpp"
                            "ntp/ntp_control.cpp"
                            "ntp/ntp_server.cpp"
                            "ntp/serve_path.cpp"
                            "ntp/tx_lag.cpp"
                            "ptp/ptp.cpp"
                       REQUIRES settingsbase webserver jsonwrapper nvsstoragemanager
                       PRIV_REQUIRES console esp_eth esp_netif esp_http_server nvs_flash
                                     esp_app_format app_update esp_timer lwip
                                     esp_driver_uart esp_driver_gpio esp_driver_mcpwm
                       INCLUDE_DIRS ".")

# IDF's -Wall leaves these as warnings, and dead code then only surfaces when a
# change happens to sit next to it. Members and published fields stay invisible
# to the compiler either way; this catches the function-and-variable half.
target_compile_options(${COMPONENT_LIB} PRIVATE
                       -Werror=unused-function -Werror=unused-variable
                       -Werror=unused-but-set-variable -Werror=unused-const-variable=1)
