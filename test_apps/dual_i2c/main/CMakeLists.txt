idf_component_register(SRCS "main.c" INCLUDE_DIRS "."
    REQUIRES esp-mt6701 esp_driver_i2c esp_driver_gptimer esp_timer
             esp_rt_diagnostics)
