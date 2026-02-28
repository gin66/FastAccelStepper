idf_component_register(
    SRC_DIRS "src" "src/pd_esp32"
    INCLUDE_DIRS "src" "src/pd_esp32"
    REQUIRES 
        esp_driver_gpio
        esp_driver_rmt
        esp_driver_pcnt
)
