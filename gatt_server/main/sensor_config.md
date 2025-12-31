# ESP32-C3 & MAX30102 - Sistem Konfigürasyon Belgesi (v1.0)
**Tarih:** 27.12.2025
**Durum:** Final Lehim Öncesi Doğrulama

## 1. Donanım ve Pin Haritası (Kritik)
Bu bağlantılar yazılımsal olarak `Software I2C` için özel tanımlanmıştır. Değiştirilemez.

| Bileşen | Pin Adı | ESP32-C3 Pini | Notlar / Uyarılar |
|:--- |:--- |:--- |:--- |
| **MAX30102** | **VCC** | **3.3V** | 🔴 **DİKKAT:** ASLA 5V'a bağlama. Sensör ve I2C hattı yanar. |
| **MAX30102** | **GND** | **GND (G)** | Ortak toprak hattı. |
| **MAX30102** | **SDA** | **GPIO 10** | Standart (8) değil, özel atandı. |
| **MAX30102** | **SCL** | **GPIO 7** | Standart (9) değil, özel atandı. |
| **MAX30102** | **INT** | *NC* | Bağlı değil (Polling modu kullanılıyor). |

## 2. Yazılım Konfigürasyonu (Register & Drivers)
Klon sensörlerin çalışması için belirlenen zorunlu parametrelerdir.

* **I2C Frekansı:** `400000 Hz` (400kHz)
* **Driver:** `driver/i2c.h` (ESP-IDF Native)
* **Çalışma Modu:** `Multi-LED Mode` (Reg: `0x07`) - **Standart SpO2 değil!**
* **Sequencer (Slot) Ayarı:** * Slot 1: `LED1 (Red)`
    * Slot 2: `LED2 (IR)`
    * Reg Adresi: `0x11` -> Değer: `0x21`
* **Akım Ayarı:** `0x24` (~7.2mA) - Düşük güç ve güvenlik için.

## 3. Montaj ve Lehim Kuralları
1.  **Sıcaklık:** Havya ucu 300°C - 350°C arasında olmalı. MAX30102 padleri hassastır, 2 saniyeden fazla ısıtma.
2.  **İzolasyon:** ESP32 ve Sensör arka yüzeyleri birbirine değmemeli (Araya Kapton bant veya sünger koy).
3.  **Kablo:** Çok damarlı, esnek silikon kablo (28 AWG veya 30 AWG) kullanılmalı. Bilek hareketiyle kopmaması için "Strain Relief" (Gerilim azaltıcı) bırakılmalı.