#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// --- I2C BUS AYARLARI ---
#define I2C_MASTER_SCL_IO           7      // SCL Pini
#define I2C_MASTER_SDA_IO           10     // SDA Pini
#define I2C_MASTER_NUM              0      // I2C Port Numarası

// DÜZELTME: 400000 (Hızlı) yerine 100000 (Standart) yapıyoruz.
// Bu hızda direnç olmasa bile MLX90614 rahat çalışır.
// Diğer sensörler de bu hızı destekler.
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TIMEOUT_MS       1000

#endif // PROJECT_CONFIG_H