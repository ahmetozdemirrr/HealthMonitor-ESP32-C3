idf_component_register(
    SRCS "main.c" "i2c_manager.c" "mlx90614.c" "max30102.c" "mpu6050.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c esp_timer nvs_flash bt freertos
)