idf_component_register(
    SRCS "main.c" "rc522.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_spi esp_driver_gpio
    PRIV_REQUIRES esp_timer
)
