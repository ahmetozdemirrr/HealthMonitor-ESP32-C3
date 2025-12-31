#include "mlx90614.h"
#include "project_config.h"
#include "driver/i2c.h"

#define MLX90614_ADDR 0x5A
#define MLX90614_TA   0x06
#define MLX90614_TOBJ 0x07

static esp_err_t mlx90614_read_reg(uint8_t reg, float *temp) {
    uint8_t data[3];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MLX90614_ADDR, &reg, 1, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) return err;

    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (raw & 0x8000) return ESP_FAIL; // Sensör hata bayrağı

    *temp = (raw * 0.02) - 273.15;
    return ESP_OK;
}

esp_err_t mlx90614_get_temperatures(float *temp_ambient, float *temp_object) {
    esp_err_t err;
    err = mlx90614_read_reg(MLX90614_TA, temp_ambient);
    if (err != ESP_OK) return err;
    
    err = mlx90614_read_reg(MLX90614_TOBJ, temp_object);
    return err;
}