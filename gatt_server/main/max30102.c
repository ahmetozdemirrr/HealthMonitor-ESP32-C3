#include "max30102.h"
#include "project_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX30102_ADDR 0x57

// Register Adresleri
#define REG_FIFO_WR_PTR     0x04
#define REG_FIFO_OVF_CTR    0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PART_ID         0xFF

static const char *TAG = "MAX30102";

// SpO2 Algoritması için Değişkenler
static float dc_red = 0;
static float dc_ir = 0;
static float filtered_spo2 = 98.0; 
#define ALPHA 0.95 

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30102_ADDR, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void max30102_init(void) {
    // 1. Reset
    write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // 2. FIFO Ayarları
    write_reg(REG_FIFO_WR_PTR, 0x00);
    write_reg(REG_FIFO_OVF_CTR, 0x00);
    write_reg(REG_FIFO_RD_PTR, 0x00);
    write_reg(REG_FIFO_CONFIG, 0x4F); 

    // 3. Mod Ayarı: SpO2 Modu (Kırmızı + IR aktif)
    write_reg(REG_MODE_CONFIG, 0x03); 
    
    // 4. Örnekleme Ayarı (100Hz, 411us)
    write_reg(REG_SPO2_CONFIG, 0x27); 
    
    // 5. LED Parlaklığı 
    // Not: SpO2 için kırmızı ışığın biraz güçlü olması iyidir.
    // 0x9F (159) değeri hem iyi okuma sağlar hem de pili sömürmez.
    write_reg(REG_LED1_PA, 0x9F); // Kırmızı
    write_reg(REG_LED2_PA, 0x9F); // IR
    
    ESP_LOGI(TAG, "MAX30102 SpO2 Modunda Baslatildi.");
}

// --- YENİ EKLENEN FONKSİYON 1 ---
// Hem Kırmızı hem IR verisini aynı anda okur.
void max30102_read_raw(uint32_t *red, uint32_t *ir) {
    uint8_t data[6];
    
    // Taze veri için pointerları sıfırla
    write_reg(REG_FIFO_WR_PTR, 0x00);
    write_reg(REG_FIFO_OVF_CTR, 0x00);
    write_reg(REG_FIFO_RD_PTR, 0x00);
    
    // Sensörün ölçüm yapması için minik bekleme
    vTaskDelay(15 / portTICK_PERIOD_MS);
    
    uint8_t reg = REG_FIFO_DATA;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MAX30102_ADDR, &reg, 1, data, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (err == ESP_OK) {
        // İlk 3 byte Kırmızı, Sonraki 3 byte IR
        *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
        *red &= 0x03FFFF; // 18-bit maskeleme
        
        *ir = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];
        *ir &= 0x03FFFF;
    } else {
        *red = 0;
        *ir = 0;
    }
}

// --- ESKİ FONKSİYONUN KORUNMUŞ HALİ ---
// Bu fonksiyonu değiştirmedik, sadece altyapısını yeni fonksiyona bağladık.
// Böylece eski kodların (sadece nabız okuyan kısımlar) bozulmadan çalışır.
uint32_t max30102_get_ir_value(void) {
    uint32_t red, ir;
    max30102_read_raw(&red, &ir); // Yeni fonksiyonu kullanarak işi halleder
    return ir; // Sadece IR'yi döner (Eskisi gibi)
}

// --- YENİ EKLENEN FONKSİYON 2 ---
// Basit SpO2 Hesaplama
float max30102_calculate_spo2(uint32_t red, uint32_t ir) {
    if (ir < 50000) return 0.0; // Sensör boşta/takılı değil

    // DC (Ortalama) Bileşeni Filtrele
    if (dc_red == 0) dc_red = red;
    if (dc_ir == 0) dc_ir = ir;

    dc_red = (ALPHA * dc_red) + ((1.0 - ALPHA) * red);
    dc_ir = (ALPHA * dc_ir) + ((1.0 - ALPHA) * ir);

    // AC (Nabız) Bileşenini Bul
    float ac_red = (float)red - dc_red;
    float ac_ir = (float)ir - dc_ir;

    // Oran Hesapla (Ratio of Ratios)
    float R = (ac_red * dc_ir) / (ac_ir * dc_red);

    // Standart SpO2 Formülü (Lineer Yaklaşım)
    float current_spo2 = 110.0 - (25.0 * R);

    // Mantıksız değerleri kırp
    if (current_spo2 > 100) current_spo2 = 99.9;
    if (current_spo2 < 70) current_spo2 = 70.0;

    // Sonucu yumuşat (Low Pass Filter)
    filtered_spo2 = (0.95 * filtered_spo2) + (0.05 * current_spo2);

    return filtered_spo2;
}