#include "mpu6050.h"
#include "project_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "esp_timer.h" 

#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_WHO_AM_I        0x75

// --- ALGORİTMA AYARLARI ---
#define STEP_THRESHOLD      1.25f  // Adım saymak için gereken darbe gücü (1.25G)
#define STEP_DEBOUNCE_MS    350    // İki adım arası minimum süre (İnsan 300ms'den hızlı koşamaz)
#define STATIONARY_TOLERANCE 0.05f // Masada durma toleransı (0.95 - 1.05 arası)

static const char *TAG = "MPU_LIB";
static int step_count = 0;         
static int64_t last_step_time = 0; 
static float current_magnitude = 0; 

// I2C Yardımcıları
static esp_err_t read_bytes(uint8_t reg, uint8_t *buffer, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(void) {
    uint8_t who_am_i;
    esp_err_t err = read_bytes(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) return err;
    return write_reg(REG_PWR_MGMT_1, 0x00); // Uyandır
}

// Bu fonksiyonu her çağırdığında sensörü okur ve analiz yapar
esp_err_t mpu6050_read_accel(mpu6050_vector_t *accel) {
    uint8_t buffer[6];
    esp_err_t err = read_bytes(REG_ACCEL_XOUT_H, buffer, 6);
    if (err != ESP_OK) return err;

    int16_t raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    // G kuvvetine çevir
    accel->x = raw_x / 16384.0;
    accel->y = raw_y / 16384.0;
    accel->z = raw_z / 16384.0;

    // Vektör Büyüklüğü (Total G Kuvveti)
    current_magnitude = sqrt(accel->x*accel->x + accel->y*accel->y + accel->z*accel->z);

    // --- ADIM SAYMA MANTIĞI ---
    int64_t now = esp_timer_get_time() / 1000; // ms cinsinden zaman

    // 1. Eşik Değeri Geçildi mi? (Darbe var mı?)
    // 2. Son adımdan bu yana yeterli süre geçti mi? (Çift saymayı önle)
    if (current_magnitude > STEP_THRESHOLD && (now - last_step_time > STEP_DEBOUNCE_MS)) {
        step_count++;
        last_step_time = now;
        ESP_LOGI(TAG, ">> ADIM YAKALANDI! (Siddet: %.2f)", current_magnitude);
    }

    return ESP_OK;
}

// Yardımcı Fonksiyonlar
float mpu6050_get_magnitude(void) {
    return current_magnitude;
}

int mpu6050_get_step_count(void) {
    return step_count;
}

// Yeni: Masada mı kontrolünü kütüphaneye aldık
bool mpu6050_is_stationary(void) {
    // 1.00G'den farkı toleransın altındaysa (0.95 - 1.05 arasındaysa)
    if (fabs(current_magnitude - 1.00f) < STATIONARY_TOLERANCE) {
        return true;
    }
    return false;
}