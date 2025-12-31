#ifndef MLX90614_H
#define MLX90614_H

#include "esp_err.h"

// Sensörden sıcaklık okuyan fonksiyon
// temp_ambient: Ortam sıcaklığı pointer'ı
// temp_object: Nesne sıcaklığı pointer'ı
esp_err_t mlx90614_get_temperatures(float *temp_ambient, float *temp_object);

#endif