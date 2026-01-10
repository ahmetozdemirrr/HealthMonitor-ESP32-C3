// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_system.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "esp_bt.h"
// #include "esp_gap_ble_api.h"
// #include "esp_gatts_api.h"
// #include "esp_bt_defs.h"
// #include "esp_bt_main.h"
// #include "esp_timer.h"

// // Sensör Kütüphaneleri
// #include "i2c_manager.h"
// #include "mpu6050.h"
// #include "max30102.h"
// #include "mlx90614.h"

// #define TAG "HEALTH_BAND"

// // --- BLE YAPILANDIRMASI ---
// #define PROFILE_NUM                 1
// #define PROFILE_APP_ID              0
// #define SVC_INST_ID                 0

// #define GATTS_SERVICE_UUID_TEST     0x00FF
// #define GATTS_CHAR_UUID_TEST        0xFF01
// #define GATTS_NUM_HANDLE_TEST       4

// typedef struct __attribute__((packed)) {
//     uint8_t flags;
//     int32_t steps;
//     float temp;
//     float spo2;
//     uint32_t ir_val;
// } ble_data_packet_t;

// ble_data_packet_t packet;
// uint16_t char_handle = 0;
// bool device_connected = false;

// #define INTERVAL_SPO2          50000    
// #define INTERVAL_MOTION        100000   
// #define INTERVAL_TEMP          1000000  
// #define INTERVAL_NOTIFY        200000   
// #define THRESHOLD_WRIST_IR     50000    
// #define TIMEOUT_IDLE_MODE      60000000 

// static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// static struct gatts_profile_inst {
//     esp_gatts_cb_t gatts_cb;
//     uint16_t gatts_if;
//     uint16_t app_id;
//     uint16_t conn_id;
//     uint16_t service_handle;
//     esp_gatt_srvc_id_t service_id;
//     uint16_t char_handle;
//     esp_bt_uuid_t char_uuid;
//     esp_gatt_perm_t perm;
//     esp_gatt_char_prop_t property;
//     uint16_t descr_handle;
//     esp_bt_uuid_t descr_uuid;
// } gl_profile_tab[PROFILE_NUM] = {
//     [PROFILE_APP_ID] = {
//         .gatts_cb = gatts_profile_event_handler,
//         .gatts_if = ESP_GATT_IF_NONE,
//     },
// };

// static esp_ble_adv_params_t adv_params = {
//     .adv_int_min        = 0x20,
//     .adv_int_max        = 0x40,
//     .adv_type           = ADV_TYPE_IND,
//     .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
//     .channel_map        = ADV_CHNL_ALL,
//     .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// static uint8_t raw_adv_data[] = {
//     0x02, 0x01, 0x06, 
//     0x02, 0x0a, 0xeb, 
//     0x0E, 0x09, 'H', 'e', 'a', 'l', 't', 'h', 'M', 'o', 'n', 'i', 't', 'o', 'r',
//     0x03, 0x03, 0xFF, 0x00
// };

// static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
//     switch (event) {
//         case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
//             esp_ble_gap_start_advertising(&adv_params);
//             break;
//         case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//             if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGE(TAG, "Advertising start failed");
//             }
//             break;
//         default:
//             break;
//     }
// }

// static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
//     switch (event) {
//         case ESP_GATTS_REG_EVT:
//             esp_ble_gap_set_device_name("HealthMonitor");
//             esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
//             gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
//             gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
//             gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
//             gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST;
//             esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, GATTS_NUM_HANDLE_TEST);
//             break;
//         case ESP_GATTS_CREATE_EVT:
//             gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
//             gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
//             gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST;
//             esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle, 
//                                    &gl_profile_tab[PROFILE_APP_ID].char_uuid,
//                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//                                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
//                                    NULL, NULL);
//             break;
//         case ESP_GATTS_ADD_CHAR_EVT:
//             gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
//             char_handle = param->add_char.attr_handle; 
//             gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
//             gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//             esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle,
//                                          &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
//                                          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
//             break;
//         // !!! İŞTE EKSİK OLAN KISIM BURASIYDI !!!
//         case ESP_GATTS_ADD_CHAR_DESCR_EVT:
//             // Açıklayıcı (Descriptor) eklendikten sonra servisi başlatmalıyız!
//             esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);
//             ESP_LOGI(TAG, "Servis BASLATILDI (00FF)"); 
//             break;
//         case ESP_GATTS_START_EVT:
//             ESP_LOGI(TAG, "GATT Server Hazir, Baglanti Bekleniyor...");
//             break;
//         case ESP_GATTS_CONNECT_EVT:
//             device_connected = true;
//             gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
//             break;
//         case ESP_GATTS_DISCONNECT_EVT:
//             device_connected = false;
//             esp_ble_gap_start_advertising(&adv_params);
//             break;
//         case ESP_GATTS_READ_EVT: 
//              {
//                  esp_gatt_rsp_t rsp;
//                  memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//                  rsp.attr_value.handle = param->read.handle;
//                  rsp.attr_value.len = sizeof(ble_data_packet_t);
//                  memcpy(rsp.attr_value.value, &packet, sizeof(ble_data_packet_t));
//                  esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
//              }
//              break;
//         case ESP_GATTS_WRITE_EVT:
//              esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
//              break;
//         default:
//             break;
//     }
// }

// static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
//     if (event == ESP_GATTS_REG_EVT) {
//         if (param->reg.status == ESP_GATT_OK) {
//             gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
//         }
//     }
//     do {
//         int idx;
//         for (idx = 0; idx < PROFILE_NUM; idx++) {
//             if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
//                 if (gl_profile_tab[idx].gatts_cb) {
//                     gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
//                 }
//             }
//         }
//     } while (0);
// }

// void sensor_task(void *pvParameters) {
//     i2c_master_init();
//     max30102_init();
//     mpu6050_init();
    
//     float temp_dummy_ambient;
//     float temp_dummy_body;

//     if (mlx90614_get_temperatures(&temp_dummy_ambient, &temp_dummy_body) == ESP_OK) {
//         ESP_LOGI(TAG, "MLX90614 Hazir.");
//     }

//     int64_t t_spo2 = 0;
//     int64_t t_motion = 0;
//     int64_t t_temp = 0;
//     int64_t t_notify = 0;
//     int64_t t_last_move = esp_timer_get_time();

//     bool is_worn = false;
//     bool is_idle = false;
//     uint32_t raw_red = 0, raw_ir = 0;

//     while (1) {
//         int64_t now = esp_timer_get_time();

//         // 1. Nabız ve SpO2
//         if (now - t_spo2 > INTERVAL_SPO2) {
//             max30102_read_raw(&raw_red, &raw_ir);
//             if (raw_ir > THRESHOLD_WRIST_IR) {
//                 is_worn = true;
//                 packet.spo2 = max30102_calculate_spo2(raw_red, raw_ir);
//             } else {
//                 is_worn = false;
//                 is_idle = true;
//                 packet.spo2 = 0;
//             }
//             packet.ir_val = raw_ir;
//             t_spo2 = now;
//         }

//         // 2. Hareket ve Adım
//         if (now - t_motion > INTERVAL_MOTION) {
//             if (is_worn) {
//                 mpu6050_vector_t accel;
//                 if (mpu6050_read_accel(&accel) == ESP_OK) {
//                     packet.steps = mpu6050_get_step_count();
//                     if (!mpu6050_is_stationary()) {
//                         t_last_move = now;
//                         is_idle = false;
//                     }
//                 }
//             }
//             if (now - t_last_move > TIMEOUT_IDLE_MODE) is_idle = true;
//             t_motion = now;
//         }

//         // 3. Sıcaklık
//         if (now - t_temp > INTERVAL_TEMP) {
//             if (is_worn) {
//                 float local_temp = 0.0;
//                 float local_ambient = 0.0;
//                 mlx90614_get_temperatures(&local_ambient, &local_temp);
//                 packet.temp = local_temp; 
//             } else {
//                 packet.temp = 0;
//             }
//             t_temp = now;
//         }

//         // 4. BLE Paketi Hazırla ve Gönder
//         if (now - t_notify > INTERVAL_NOTIFY) {
//             packet.flags = 0;
//             if (is_worn) packet.flags |= 1; 
//             if (is_idle) packet.flags |= 2; 

//             // LOG
//             ESP_LOGI(TAG, "Durum:%s | Adim:%ld | Temp:%.2f | SpO2:%.1f | IR:%lu", 
//                      device_connected ? "BAGLI " : "BEKLIYOR",
//                      packet.steps, packet.temp, packet.spo2, packet.ir_val);

//             if (device_connected) {
//                 esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_APP_ID].gatts_if, 
//                                             gl_profile_tab[PROFILE_APP_ID].conn_id, 
//                                             char_handle, 
//                                             sizeof(ble_data_packet_t), 
//                                             (uint8_t *)&packet, 
//                                             false);
//             }
//             t_notify = now;
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void app_main(void)
// {
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     if (ret) { ESP_LOGE(TAG, "BT init failed"); return; }
//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     if (ret) { ESP_LOGE(TAG, "BT enable failed"); return; }

//     ret = esp_bluedroid_init();
//     if (ret) { ESP_LOGE(TAG, "Bluedroid init failed"); return; }
//     ret = esp_bluedroid_enable();
//     if (ret) { ESP_LOGE(TAG, "Bluedroid enable failed"); return; }

//     ret = esp_ble_gatts_register_callback(gatts_event_handler);
//     if (ret) { ESP_LOGE(TAG, "GATTS register failed"); return; }
//     ret = esp_ble_gap_register_callback(gap_event_handler);
//     if (ret) { ESP_LOGE(TAG, "GAP register failed"); return; }

//     ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
//     if (ret) { ESP_LOGE(TAG, "GATTS app register failed"); return; }

//     xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mpu6050.h"
#include "mlx90614.h"

#define TAG "HEALTH_BAND"

// --- I2C AYARLARI ---
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_SCL_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000  
#define I2C_MASTER_TIMEOUT_MS       1000

// --- BLE AYARLARI ---
#define PROFILE_NUM                 1
#define PROFILE_APP_ID              0
#define SVC_INST_ID                 0
#define GATTS_SERVICE_UUID_TEST     0x00FF
#define GATTS_CHAR_UUID_TEST        0xFF01
#define GATTS_NUM_HANDLE_TEST       4

// SpO2 KALDIRILDI (Android ile uyumlu 13 Byte)
typedef struct __attribute__((packed)) {
    uint8_t flags;      // 1 byte
    int32_t steps;      // 4 byte
    float temp;         // 4 byte
    uint32_t ir_val;    // 4 byte
} ble_data_packet_t;

ble_data_packet_t packet;
uint16_t char_handle = 0;
bool device_connected = false;

// --- I2C FONKSİYONLARI ---
void i2c_bus_recovery() {
    gpio_reset_pin(I2C_MASTER_SDA_IO);
    gpio_reset_pin(I2C_MASTER_SCL_IO);
    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_OUTPUT);
    for(int i=0; i<9; i++) {
        gpio_set_level(I2C_MASTER_SCL_IO, 0); esp_rom_delay_us(10);
        gpio_set_level(I2C_MASTER_SCL_IO, 1); esp_rom_delay_us(10);
    }
    gpio_set_level(I2C_MASTER_SDA_IO, 0); esp_rom_delay_us(10);
    gpio_set_level(I2C_MASTER_SCL_IO, 1); esp_rom_delay_us(10);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
}

void my_i2c_init() {
    i2c_bus_recovery(); 
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// MLX90614 MANUEL OKUMA (Repeated Start için gerekli)
static esp_err_t mlx90614_read_manual(float *object) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x5A << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true);
    i2c_master_start(cmd); 
    i2c_master_write_byte(cmd, (0x5A << 1) | I2C_MASTER_READ, true);
    uint8_t low = 0, high = 0, pec = 0;
    i2c_master_read_byte(cmd, &low, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &high, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &pec, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        uint16_t temp_raw = (high << 8) | low;
        if (temp_raw == 0xFFFF) return ESP_FAIL; 
        *object = (float)temp_raw * 0.02 - 273.15;
        return ESP_OK;
    } 
    return ret;
}

// --- BLE CALLBACKLERİ (ESKİ HALİNE DÖNDÜRÜLDÜ) ---
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// ESKİ RAW DATA (HealthMonitor ismiyle)
static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06, 
    0x02, 0x0a, 0xeb, 
    0x0E, 0x09, 'H', 'e', 'a', 'l', 't', 'h', 'M', 'o', 'n', 'i', 't', 'o', 'r',
    0x03, 0x03, 0xFF, 0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name("HealthMonitor");
            esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST;
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, GATTS_NUM_HANDLE_TEST);
            break;
        case ESP_GATTS_CREATE_EVT:
            gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST;
            esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle, 
                                   &gl_profile_tab[PROFILE_APP_ID].char_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
                                   NULL, NULL);
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
            char_handle = param->add_char.attr_handle; 
            gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle,
                                         &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            device_connected = true;
            gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            device_connected = false;
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_READ_EVT: 
             {
                 esp_gatt_rsp_t rsp;
                 memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                 rsp.attr_value.handle = param->read.handle;
                 rsp.attr_value.len = sizeof(ble_data_packet_t);
                 memcpy(rsp.attr_value.value, &packet, sizeof(ble_data_packet_t));
                 esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
             }
             break;
        case ESP_GATTS_WRITE_EVT:
             esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
             break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
    }
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
            if (gl_profile_tab[idx].gatts_cb) {
                gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

// --- ANA SENSÖR DÖNGÜSÜ ---
void sensor_task(void *pvParameters) {
    my_i2c_init(); 
    vTaskDelay(pdMS_TO_TICKS(500));

    if (mpu6050_init() == ESP_OK) {
        ESP_LOGI(TAG, "MPU6050 Baslatildi.");
    } else {
        ESP_LOGE(TAG, "MPU6050 Bulunamadi!");
    }
    
    int64_t t_motion = 0, t_temp = 0, t_notify = 0;
    
    while(1) {
        int64_t now = esp_timer_get_time();

        // 1. MPU6050 Okuma
        if (now - t_motion > 100000) {
            mpu6050_vector_t accel;
            if (mpu6050_read_accel(&accel) == ESP_OK) {
                packet.steps = mpu6050_get_step_count();
            }
            t_motion = now;
        }

        // 2. MLX90614 Okuma
        if (now - t_temp > 500000) {
            float loc_t = 0;
            esp_err_t res = mlx90614_read_manual(&loc_t);
            if (res == ESP_OK) {
                packet.temp = loc_t; 
                ESP_LOGI(TAG, "SICAKLIK: %.2f C | ADIM: %ld", loc_t, (long)packet.steps);
            }
            t_temp = now;
        }

        // 3. BLE Veri Gönderimi
        if (now - t_notify > 200000) {
            packet.flags = 1; 
            packet.ir_val = 0;
            // SpO2 ataması YAPILMIYOR (Struct'tan silindi)
            
            if (device_connected) {
                esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_APP_ID].gatts_if, 
                                            gl_profile_tab[PROFILE_APP_ID].conn_id, 
                                            char_handle, 
                                            sizeof(ble_data_packet_t), 
                                            (uint8_t *)&packet, 
                                            false);
            }
            t_notify = now;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(PROFILE_APP_ID);
    
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}