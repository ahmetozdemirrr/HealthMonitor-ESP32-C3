#ifndef MAX30102_H
#define MAX30102_H
#include <stdint.h>
#include "esp_err.h"

// Sensörü başlatır
void max30102_init(void);

// SADECE IR verisi okur (Eski kod uyumluluğu için)
uint32_t max30102_get_ir_value(void);

// --- YENİ EKLENENLER (Hata verenler bunlardı) ---

// Hem Kırmızı hem IR verisini okur (SpO2 için)
void max30102_read_raw(uint32_t *red, uint32_t *ir);

// SpO2 Hesaplar
float max30102_calculate_spo2(uint32_t red, uint32_t ir);

#endif