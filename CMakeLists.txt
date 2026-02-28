idf_component_register(
    SRC_DIRS "src" "src/pd_esp32" "src/fas_ramp" "src/log2" "src/fas_queue"
    INCLUDE_DIRS "src"
    REQUIRES 
        esp_driver_gpio
        esp_driver_rmt
        esp_driver_pcnt
)
