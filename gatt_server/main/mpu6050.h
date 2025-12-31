#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_vector_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read_accel(mpu6050_vector_t *accel);

// Yardımcılar
int mpu6050_get_step_count(void);
float mpu6050_get_magnitude(void);
bool mpu6050_is_stationary(void); // YENİ: Masada mı?

#endif