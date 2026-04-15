idf_component_register(
    SRCS
        "test_main.c"
        "../test_ax25_address.c"
        "../test_ax25_agwpe.c"
        "../test_ax25_agwpe_client.c"
        "../test_ax25_agwpe_server.c"
        "../test_ax25_beacon.c"
        "../test_ax25_config.c"
        "../test_ax25_frame.c"
        "../test_ax25_conn.c"
        "../test_ax25_kiss.c"
        "../test_ax25_log_tcp_server.c"
        "../test_ax25_monitor_tcp_server.c"
        "../test_ax25_phy_kiss_tcp_client.c"
        "../test_ax25_phy_tcp_server.c"
        "../test_ax25_phy_kiss_uart.c"
        "../test_ax25_phy_kiss_tcp_server.c"
        "../test_ax25_buffer.c"
        "../test_ax25_print.c"
        "../test_ax25_digipeater.c"
        "../test_ax25_router.c"
        "../test_ax25_uart.c"
        "../test_ax25_wifi.c"
    INCLUDE_DIRS "."
    REQUIRES esp-ax25 unity esp_timer
    WHOLE_ARCHIVE
)
