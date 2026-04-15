set(srcs
    "src/ax25_address.c"
    "src/ax25_config.c"
    "src/ax25_config_defaults.c"
    "src/ax25_digipeater.c"
    "src/ax25_agwpe.c"
    "src/ax25_agwpe_client.c"
    "src/ax25_beacon.c"
    "src/ax25_buffer.c"
    "src/ax25_conn.c"
    "src/ax25_conn_dispatcher.c"
    "src/ax25_frame.c"
    "src/ax25_kiss.c"
    "src/ax25_kiss_tcp_server.c"
    "src/ax25_log.c"
    "src/ax25_log_tcp_server.c"
    "src/ax25_monitor_tcp_server.c"
    "src/ax25_phy_kiss_tcp_client.c"
    "src/ax25_phy_tcp_server.c"
    "src/ax25_phy_kiss_uart.c"
    "src/ax25_print.c"
    "src/ax25_agwpe_server.c"
    "src/ax25_router.c"
    "src/ax25_uart.c"
    "src/ax25_wifi.c"
)

if(CONFIG_BT_ENABLED)
    list(APPEND srcs "src/ax25_phy_kiss_bt.c")
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES driver lwip esp_timer esp_wifi esp_event esp_netif nvs_flash
    PRIV_REQUIRES bt
)
