idf_component_register(SRCS "main.c" "max30102.c" "oled.c" "algorithm.c" "ble.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt driver)
