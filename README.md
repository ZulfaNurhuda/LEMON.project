# 🍋 LEMON! Live Emission MONitor 

Sistem deteksi real-time gas berbahaya **CO** (MQ-7) dan **NOx** (MQ-135) di kabin kendaraan. Jika kondisi bahaya berlangsung lebih dari 120 detik, sistem mengambil koordinat GPS dan mengirim notifikasi ke Telegram secara otomatis.

---

## Hardware

| Board | Peran | Sketch |
|-------|-------|--------|
| ESP32-S3 | Master — FreeRTOS, aktuator, Telegram/GPS | `src/FinalProjectMaster/` |
| ESP32 (Box A1) | Sensor CO (MQ-7), 3 LED indikator | `src/FinalProjectMQ7/` |
| ESP32 (Box B1) | Sensor NOx (MQ-135), 3 LED indikator | `src/FinalProjectMQ135/` |
| Arduino Nano | Level shifter 3.3V → 5V (relay + buzzer) | `src/FinalProjectNano3v3To5vRepeater/` |

## Cara Flash

Buka masing-masing folder di **Arduino IDE**, pilih board yang sesuai, lalu upload.

Urutan yang disarankan:
1. `FinalProjectNano3v3To5vRepeater` → Arduino Nano
2. `FinalProjectMQ7` → ESP32 (Box A1)
3. `FinalProjectMQ135` → ESP32 (Box B1)
4. `FinalProjectMaster` → ESP32-S3 (Box 2) — upload terakhir

## Konfigurasi

Sebelum flash Master, edit bagian ini di `FinalProjectMaster.ino`:

```cpp
#define USING      WIFI          // ganti ke SIM untuk mode GPRS

#define WIFI_SSID  "NamaWiFi"
#define WIFI_PASS  "Password"

#define TG_BOT_TOKEN  "token_dari_BotFather"
#define TG_CHAT_ID    "id_dari_userinfobot"

#define SIM_APN    "internet"    // Telkomsel — sesuaikan operator
```

## Library yang Dibutuhkan

Install via Arduino IDE Library Manager:

| Library | Dipakai oleh |
|---------|-------------|
| `ArduinoJson` ≥ 7.x | Master |
| `ESP32Servo` | Master |
| `EspSoftwareSerial` ≥ 6.16.1 | Master |

## Dokumentasi Lengkap

→ [`docs/TECHNICAL_GUIDE.md`](docs/TECHNICAL_GUIDE.md)

Mencakup rangkaian per komponen (ASCII), proses kalibrasi sensor, pipeline preprocessing sinyal, manajemen daya, dua mode koneksi internet, dan rencana pengembangan lanjutan.
