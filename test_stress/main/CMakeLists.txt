idf_component_register(
    SRCS
        "test_main.c"
        "../../test/test_ax25_stress.c"
    INCLUDE_DIRS "."
    REQUIRES esp-ax25 unity esp_timer
    WHOLE_ARCHIVE
)
