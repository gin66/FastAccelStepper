idf_component_register(
    SRC_DIRS "src" "src/pd_esp32" "src/fas_ramp" "src/log2" "src/fas_queue"
    INCLUDE_DIRS "src"
    REQUIRES
        esp_driver_gpio
        esp_driver_rmt
        esp_driver_mcpwm
        esp_driver_pcnt
        esp_driver_i2s
)

option(FAS_STRICT_WARNINGS "Treat warnings in FastAccelStepper as errors" ON)
if(FAS_STRICT_WARNINGS)
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
endif()
