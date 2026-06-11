# Sistem Pemantauan dan Peringatan Dini Konsentrasi Gas CO dan NOx pada Kabin Kendaraan Bermotor

**Dokumentasi Teknis Implementasi — Versi 10**

---

## Bagian 1 — Gambaran Sistem

### 1.1 Latar Belakang

Paparan gas berbahaya di dalam kabin kendaraan bermotor merupakan risiko kesehatan yang sering tidak disadari. Gas karbon monoksida (CO) yang dihasilkan dari pembakaran tidak sempurna bersifat tidak berwarna dan tidak berbau sehingga tidak terdeteksi secara indrawi. Gas nitrogen oksida (NOx) dari emisi mesin dapat mengiritasi saluran pernapasan dan meningkatkan risiko penyakit paru. Pada konsentrasi melebihi ambang batas di ruang tertutup seperti kabin kendaraan, keduanya dapat menyebabkan penurunan konsentrasi, kehilangan kesadaran, hingga kematian.

Nilai ambang batas yang ditetapkan Permenaker No. 5 Tahun 2018 (Nilai Ambang Batas Faktor Kimia) menjadi acuan sistem ini dalam menentukan level risiko.

### 1.2 Kemampuan Sistem

| Kemampuan | Deskripsi |
|-----------|-----------|
| Deteksi CO real-time | Sensor MQ-7 dengan preprocessing EMA dan outlier rejection |
| Deteksi NOx real-time | Sensor MQ-135 dengan preprocessing Median Filter dan EMA |
| Indikator lokal | 3 LED (hijau/kuning/merah) per board sensor |
| Kontrol aktuator | Servo (visual), relay+fan (ventilasi), buzzer (alarm audio) |
| Notifikasi darurat | Telegram + koordinat GPS Maps setelah 120 detik bahaya |
| Dua mode koneksi | WiFi (development) dan SIM/GPRS (deployment kendaraan) |
| Arsitektur non-blocking | FreeRTOS 5 task — operasi jaringan tidak menghentikan sensor |

### 1.3 Arsitektur Sistem

```
┌─────────────┐     UART      ┌─────────────────────────────────────────┐
│  Box A1     │ ─────────────►│            Box 2                        │
│  ESP32-A    │  GPIO25→16    │            ESP32-S3 Master              │
│  MQ-7 (CO)  │               │  FreeRTOS: Task_SensorA, Task_SensorB,  │
└─────────────┘               │  Task_Control, Task_Network, Task_Log   │
                              │                                         │
┌─────────────┐     UART      │  Aktuator: Servo, Relay+Fan, Buzzer     │
│  Box B1     │ ─────────────►│  Komunikasi: SIM808 (GPRS + GPS)        │
│  ESP32-B    │  GPIO25→15    │  Notifikasi: Telegram Bot               │
│  MQ-135(NOx)│               └─────────────────────────────────────────┘
└─────────────┘
```

### 1.4 Threshold Bahaya

| Status   | CO (ppm) | NOx (ppm) | Servo  | Fan    | Buzzer         |
|----------|----------|-----------|--------|--------|----------------|
| AMAN     | 0 – 9    | 0 – 9     | 0°     | Mati   | Diam           |
| WASPADA  | 10 – 24  | 10 – 24   | 90°    | Mati   | 100ms/2000ms   |
| BAHAYA   | ≥ 25     | ≥ 25      | 180°   | Nyala  | 200ms/200ms    |
| DARURAT  | `BAHAYA` berlangsung > 120 detik → kirim GPS + Telegram   |

Status gabungan diambil dari nilai tertinggi antara CO dan NOx.

---

## Bagian 2 — Kebutuhan Komponen

| No | Komponen                        | Jml | Keterangan                                                     |
|----|---------------------------------|-----|----------------------------------------------------------------|
| 1  | ESP32-S3 Dev Module             | 1   | Master controller, FreeRTOS, WiFi/SIM                          |
| 2  | ESP32 Dev Module                | 2   | Board sensor (Box A1 = CO, Box B1 = NOx)                       |
| 3  | Sensor MQ-7                     | 1   | CO, VCC wajib 5V, RL terukur 0.95 kΩ                           |
| 4  | Sensor MQ-135                   | 1   | NOx, VCC wajib 5V, RL terukur 0.95 kΩ                          |
| 5  | SIM808 Module                   | 1   | GSM/GPRS/GPS, VIN 12V langsung dari adaptor                    |
| 6  | SIM Card aktif data             | 1   | Untuk mode GPRS — sesuaikan APN operator di kode               |
| 7  | Servo SG90                      | 1   | Indikator visual sudut (0°/90°/180°)                           |
| 8  | Relay Module 5V Active LOW      | 1   | Kontrol fan, dipicu via Arduino Nano *                         |
| 9  | Buzzer Module Low Level Trigger | 1   | Alarm audio, dipicu via Arduino Nano *                         |
| 10 | DC Fan 5V                       | 1   | Ventilasi darurat saat status BAHAYA                           |
| 11 | Arduino Nano (ATmega328P)       | 1   | Level shifter 3.3V→5V untuk relay dan buzzer *                 |
| 12 | LM2596 Buck Converter           | 1   | Step-down 12V→5.1V untuk Rail 5V seluruh sistem                |
| 13 | Adaptor 12V 2A                  | 1   | Sumber daya utama                                              |
| 14 | Kapasitor 1000 µF elektrolit    | 3   | 2× output LM2596 (Rail 5V), 1× pin 3.3V ESP32-S3               |
| 15 | Resistor 330 Ω                  | 6   | Current limiting 6 LED (3 per board sensor)                    |
| 16 | Resistor 10 kΩ                  | 2   | Pull-up 3.3V ESP32-S3 ke Nano D2 dan D3                        |
| 17 | LED Hijau                       | 2   | Status AMAN (satu per board)                                   |
| 18 | LED Kuning                      | 2   | Status WASPADA                                                 |
| 19 | LED Merah                       | 2   | Status BAHAYA                                                  |

\* **Catatan komponen Relay, Buzzer, dan Arduino Nano:**
Arduino Nano digunakan sebagai level shifter karena relay module menginterpretasikan sinyal HIGH 3.3V (GPIO ESP32-S3) sebagai LOW — relay membutuhkan tegangan > 4.2V untuk transisi OFF. Nano (5V logic) membaca sinyal 3.3V dari ESP32 dan meneruskannya dalam level 5V.

Solusi yang lebih efisien untuk implementasi production:
- **MOSFET IRLZ44N** — logic-level N-channel, V_GS threshold 1–2V, fully ON di 3.3V, konsumsi ~1 mA (vs Nano ~50 mA)
- **Transistor NPN S8050/2N2222** — solusi sederhana per channel, konsumsi ~1 mA

---

## Bagian 3 — Rangkaian Per Komponen

**Notasi:**
- `───►` : kabel daya (VCC, GND, Rail Power)
- `═══►` : kabel sinyal/data (GPIO, UART, PWM, ADC)
- `╌╌╌►` : kabel referensi arus kecil (pull-up, anti-floating)

---

### 3.1 Sumber Daya

```
┌────────────────────────────────────────────────┐
│  ADAPTOR 12V 2A                                │
├────────────────────────────────────────────────┤
│                                                │
│  [+]   ──────────────────────────────────┬───► IN+  LM2596
│                                          └───► VIN  SIM808 (langsung!)
│  [-]   ──────────────────────────────────┬───► IN-  LM2596
│                                          └───► GND  SIM808
│                                                │
└────────────────────────────────────────────────┘

┌────────────────────────────────────────────────┐
│  LM2596 BUCK CONVERTER                         │
│  Set output: 5.1V                              │
├────────────────────────────────────────────────┤
│                                                │
│  [IN+]   ──────────────────────────────────► 12V dari adaptor
│  [IN-]   ──────────────────────────────────► GND adaptor
│  [OUT+]  ══════════════════════════════════► Rail 5V  (Bus distribusi)
│  [OUT-]  ──────────────────────────────────► Rail GND (Bus distribusi)
│                                                │
└────────────────────────────────────────────────┘

  Di titik OUT+ dan OUT− dipasang paralel:
  ┌────────────────┐   ┌────────────────┐
  │ Kapasitor      │   │ Kapasitor      │
  │ 1000 µF  (+)──────── Rail 5V        │
  │          (-)──────── Rail GND       │
  └────────────────┘   └────────────────┘
  Total = 2000 µF — menyerap spike arus saat WiFi init.
```

---

### 3.2 Sensor MQ-7 (CO)

```
┌────────────────────────────────────────────────┐
│  SENSOR MQ-7                                   │
│  Gas target : Carbon Monoxide (CO)             │
│  RL terukur : 0.95 kΩ                          │
├────────────────────────────────────────────────┤
│                                                │
│  [VCC]  ───────────────────────────────────► Rail 5V
│  [GND]  ───────────────────────────────────► Rail GND
│  [A0]   ═══════════════════════════════════► GPIO34 ESP32-A  (ADC)
│  [D0]   tidak dipakai                          │
│                                                │
└────────────────────────────────────────────────┘

  ⚠  VCC wajib 5V. Heater dan lapisan sensitif MQ-7
     membutuhkan 5V untuk karakteristik sensitivitas
     sesuai kurva datasheet. Tegangan lebih rendah
     menggeser baseline RS dan menghasilkan R0 tidak akurat.
```

---

### 3.3 Sensor MQ-135 (NOx)

```
┌────────────────────────────────────────────────┐
│  SENSOR MQ-135                                 │
│  Gas target : Nitrogen Oxide (NOx)             │
│  RL terukur : 0.95 kΩ                          │
├────────────────────────────────────────────────┤
│                                                │
│  [VCC]  ───────────────────────────────────► Rail 5V
│  [GND]  ───────────────────────────────────► Rail GND
│  [A0]   ═══════════════════════════════════► GPIO34 ESP32-B  (ADC)
│  [D0]   tidak dipakai                          │
│                                                │
└────────────────────────────────────────────────┘

  ⚠  VCC wajib 5V — alasan identik dengan MQ-7.
     MQ-135 bersifat cross-sensitive terhadap berbagai
     gas (NOx, NH₃, CO₂, alkohol, benzena). Preprocessing
     Median Filter diterapkan untuk menekan spike akibat
     paparan gas-gas tersebut.
```

---

### 3.4 LED Indikator (3 LED × 2 board = 6 total)

```
┌────────────────────────────────────────────────┐
│  LED HIJAU  — Status AMAN                      │
├────────────────────────────────────────────────┤
│  [Anoda  (+)] ══════════[330Ω]════════════════► GPIO13 ESP32-A / GPIO33 ESP32-B
│  [Katoda (−)] ─────────────────────────────► Rail GND
└────────────────────────────────────────────────┘

┌────────────────────────────────────────────────┐
│  LED KUNING  — Status WASPADA                  │
├────────────────────────────────────────────────┤
│  [Anoda  (+)] ══════════[330Ω]════════════════► GPIO14 ESP32-A / GPIO26 ESP32-B
│  [Katoda (−)] ─────────────────────────────► Rail GND
└────────────────────────────────────────────────┘

┌────────────────────────────────────────────────┐
│  LED MERAH  — Status BAHAYA                    │
├────────────────────────────────────────────────┤
│  [Anoda  (+)] ══════════[330Ω]════════════════► GPIO27 ESP32-A/B
│  [Katoda (−)] ─────────────────────────────► Rail GND
└────────────────────────────────────────────────┘

  Resistor 330Ω membatasi arus LED ≈ (3.3V − 2.0V) / 330Ω ≈ 4 mA
  GPIO ESP32 maksimum 40 mA — tiga LED aktif bersama masih aman.
  Hanya satu LED aktif HIGH pada satu waktu (fungsi updateLEDs).
```

---

### 3.5 ESP32-A — Box A1 (CO Monitor)

```
┌────────────────────────────────────────────────┐
│  ESP32-A  (Box A1)                             │
│  Board  : ESP32 Dev Module                     │
│  Fungsi : Baca MQ-7, preprocessing CO,         │
│           kirim JSON ke Master via UART        │
├────────────────────────────────────────────────┤
│                                                │
│  [VIN]    ──────────────────────────────────► Rail 5V
│  [GND]    ──────────────────────────────────► Rail GND
│  [GPIO34] ═══════════════════════════════════► MQ-7  A0  (ADC input only)
│  [GPIO13] ═══════════════════════════════════► LED Hijau  via 330Ω
│  [GPIO14] ═══════════════════════════════════► LED Kuning via 330Ω
│  [GPIO27] ═══════════════════════════════════► LED Merah  via 330Ω
│  [GPIO25] ═══════════════════════════════════► GPIO16 ESP32-S3 Master
│           TX UART 9600 baud, satu arah          │
│  [GND]    ──────────────────────────────────► GND ESP32-S3 Master (wajib!)
│                                                │
└────────────────────────────────────────────────┘
```

---

### 3.6 ESP32-B — Box B1 (NOx Monitor)

```
┌────────────────────────────────────────────────┐
│  ESP32-B  (Box B1)                             │
│  Board  : ESP32 Dev Module                     │
│  Fungsi : Baca MQ-135, preprocessing NOx,      │
│           kirim JSON ke Master via UART        │
├────────────────────────────────────────────────┤
│                                                │
│  [VIN]    ──────────────────────────────────► Rail 5V
│  [GND]    ──────────────────────────────────► Rail GND
│  [GPIO34] ═══════════════════════════════════► MQ-135 A0  (ADC input only)
│  [GPIO33] ═══════════════════════════════════► LED Hijau  via 330Ω
│  [GPIO26] ═══════════════════════════════════► LED Kuning via 330Ω
│  [GPIO27] ═══════════════════════════════════► LED Merah  via 330Ω
│  [GPIO25] ═══════════════════════════════════► GPIO15 ESP32-S3 Master
│           TX UART 9600 baud  (BUKAN GPIO16!)    │
│  [GND]    ──────────────────────────────────► GND ESP32-S3 Master (wajib!)
│                                                │
└────────────────────────────────────────────────┘
```

---

### 3.7 ESP32-S3 Master — Box 2

```
┌────────────────────────────────────────────────┐
│  ESP32-S3 MASTER  (Box 2)                      │
│  Board     : ESP32-S3 Dev Module               │
│  Arsitektur: FreeRTOS 5 task, 2 core           │
├────────────────────────────────────────────────┤
│                                                │
│  [VIN]    ──────────────────────────────────► Rail 5V
│  [GND]    ──────────────────────────────────► Rail GND
│  [3V3]    ╌╌[10kΩ]╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌► Nano D2  (anti-floating)
│  [3V3]    ╌╌[10kΩ]╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌► Nano D3  (anti-floating)
│  [GPIO16] ═══════════════════════════════════► RX ← GPIO25 ESP32-A
│           HardwareSerial2, INPUT                │
│  [GPIO15] ═══════════════════════════════════► SW-RX ← GPIO25 ESP32-B
│           SoftwareSerial, INPUT_PULLUP          │
│  [GPIO4]  ═══════════════════════════════════► TX → SIM808 RX
│  [GPIO5]  ═══════════════════════════════════► RX ← SIM808 TX
│  [GPIO12] ═══════════════════════════════════► PWRKEY SIM808
│  [GPIO6]  ═══════════════════════════════════► SIG Servo SG90  (PWM)
│  [GPIO7]  ═══════════════════════════════════► Nano D2 → Relay IN
│           RELAY_ON = LOW, RELAY_OFF = HIGH      │
│  [GPIO8]  ═══════════════════════════════════► Nano D3 → Buzzer IO
│           BUZZER_ON = LOW, BUZZER_OFF = HIGH    │
│                                                │
└────────────────────────────────────────────────┘

  Kapasitor 1000 µF menempel langsung di pin 3V3 → GND ESP32-S3.
  Menstabilkan tegangan 3.3V saat WiFi cold start calibration.
```

---

### 3.8 SIM808

```
┌────────────────────────────────────────────────┐
│  SIM808                                        │
│  Fungsi : GPRS internet + koordinat GPS        │
│  Power  : 12V LANGSUNG — bukan dari Rail 5V!   │
├────────────────────────────────────────────────┤
│                                                │
│  [VIN]     ─────────────────────────────────► 12V adaptor langsung
│  [GND]     ─────────────────────────────────► Rail GND (common!)
│  [TX]      ═════════════════════════════════► GPIO5  ESP32-S3 (Serial1 RX)
│  [RX]      ═════════════════════════════════► GPIO4  ESP32-S3 (Serial1 TX)
│  [PWRKEY]  ═════════════════════════════════► GPIO12 ESP32-S3
│                                                │
└────────────────────────────────────────────────┘

  SIM808 dipisah ke jalur 12V karena dapat menarik arus
  hingga 2 A saat transmisi GPRS. Jika digabung Rail 5V,
  spike ini menyebabkan voltage drop yang memicu Brownout
  Reset (BOD) pada ESP32-S3.
  GND tetap harus terhubung ke Rail GND (common ground).
```

---

### 3.9 Arduino Nano — Level Shifter *

```
┌────────────────────────────────────────────────┐
│  ARDUINO NANO  (ATmega328P)         *          │
│  Fungsi : Level shifter 3.3V → 5V              │
│  Solusi darurat — lihat catatan di bawah       │
├────────────────────────────────────────────────┤
│                                                │
│  [5V pin]  ─────────────────────────────────► Rail 5V
│            WAJIB pin 5V, BUKAN pin VIN!        │
│            VIN melewati regulator onboard      │
│            sehingga output HIGH tidak penuh 5V │
│  [GND]     ─────────────────────────────────► Rail GND
│  [D2]      ═════════════════════════════════► GPIO7  ESP32-S3
│            + 10kΩ dari 3V3 ESP32-S3 ke D2      │
│  [D3]      ═════════════════════════════════► GPIO8  ESP32-S3
│            + 10kΩ dari 3V3 ESP32-S3 ke D3      │
│  [D4]      ═════════════════════════════════► Relay Module IN
│  [D5]      ═════════════════════════════════► Buzzer Module IO
│                                                │
└────────────────────────────────────────────────┘

  Cara kerja — tidak ada inversi logic:
    ESP32 LOW  (0V)    → Nano output LOW  (0V)   → device ON
    ESP32 HIGH (3.3V)  → Nano output HIGH (5V)   → device OFF

  Pull-up anti-floating ke 3V3 ESP32-S3, BUKAN ke 5V Nano.
  Pull-up ke 5V Nano memberi tegangan balik 5V ke GPIO
  ESP32-S3 yang hanya toleran 3.3V — dapat merusak chip.

* Solusi proper: MOSFET IRLZ44N per channel (~1 mA, no code needed)
```

---

### 3.10 Relay Module

```
┌────────────────────────────────────────────────┐
│  RELAY MODULE 5V  (Active LOW)                 │
│  Kontrol : Fan ventilasi darurat               │
├────────────────────────────────────────────────┤
│                                                │
│  [VCC]  ────────────────────────────────────► Rail 5V
│  [GND]  ────────────────────────────────────► Rail GND
│  [IN]   ════════════════════════════════════► Nano D4  (← GPIO7 Master)
│         RELAY_ON = LOW, RELAY_OFF = HIGH       │
│  [COM]  ────────────────────────────────────► Rail 5V  (sumber arus fan)
│  [NO]   ════════════════════════════════════► Fan (+)
│         Normally Open — terhubung saat ON      │
│  [NC]   tidak dipakai                          │
│                                                │
└────────────────────────────────────────────────┘

  IN=LOW  → relay ON  → COM terhubung NO → Fan mendapat 5V → nyala
  IN=HIGH → relay OFF → COM–NO terputus  → Fan mati
```

---

### 3.11 Buzzer Module

```
┌────────────────────────────────────────────────┐
│  BUZZER MODULE  (Low Level Trigger)            │
│  Transistor internal : PNP S8550, VCC = 5V     │
├────────────────────────────────────────────────┤
│                                                │
│  [VCC]  ────────────────────────────────────► Rail 5V
│  [GND]  ────────────────────────────────────► Rail GND
│  [I/O]  ════════════════════════════════════► Nano D5  (← GPIO8 Master)
│         BUZZER_ON = LOW, BUZZER_OFF = HIGH     │
│                                                │
└────────────────────────────────────────────────┘

  I/O = LOW  (0V) → PNP base = 0V  → V_EB = 5V  → transistor ON  → bunyi
  I/O = HIGH (5V) → PNP base = 5V  → V_EB = 0V  → transistor OFF → diam

  GPIO ESP32-S3 HIGH = 3.3V tidak cukup — V_EB = 5 − 3.3 = 1.7V > 0.7V
  threshold, transistor masih sebagian ON. Nano menghasilkan 5V sehingga
  transistor mati sempurna.

  Pola buzzer:
    BUZZ_SLOW : 100ms ON / 2000ms OFF → status WASPADA
    BUZZ_FAST : 200ms ON /  200ms OFF → status BAHAYA
```

---

### 3.12 DC Fan

```
┌────────────────────────────────────────────────┐
│  DC FAN  5V                                    │
│  Ventilasi darurat kabin                       │
├────────────────────────────────────────────────┤
│                                                │
│  [(+)]  ════════════════════════════════════► Relay NO
│          mendapat 5V hanya saat relay ON       │
│  [(−)]  ────────────────────────────────────► Rail GND
│                                                │
└────────────────────────────────────────────────┘
```

---

### 3.13 Servo SG90

```
┌────────────────────────────────────────────────┐
│  SERVO SG90                                    │
│  Indikator visual derajat status gas           │
│  Kabel: merah=VCC, hitam/coklat=GND,           │
│         oranye/kuning=SIG                      │
├────────────────────────────────────────────────┤
│                                                │
│  [VCC]  (merah)         ────────────────────► Rail 5V
│  [GND]  (hitam/coklat)  ────────────────────► Rail GND
│  [SIG]  (oranye/kuning) ════════════════════► GPIO6 ESP32-S3  (PWM)
│                                                │
└────────────────────────────────────────────────┘

  Posisi sudut berdasarkan status gabungan:
    AMAN     →   0°  (servo ke posisi minimum)
    WASPADA  →  90°  (servo ke tengah)
    BAHAYA   → 180°  (servo ke posisi maksimum)
```

---

## Bagian 4 — Kode

---

### 4.1 FinalProjectMQ7.ino — ESP32-A (CO Monitor)

```cpp
/*
 * ================================================================
 * ESP32-A: Sensor MQ-7 — Carbon Monoxide (CO) Monitor
 * ================================================================
 * Preprocessing : EMA (α=0.10) + Outlier Rejection
 * Output        : 3 LED + JSON via UART ke Master ESP32-S3
 *
 * Mengapa EMA untuk MQ-7?
 *   MQ-7 lebih stabil dari MQ-135. EMA memberi bobot lebih ke
 *   data terbaru → responsive tapi tetap smooth. α=0.10 dipilih
 *   karena CO cenderung naik perlahan (tidak perlu terlalu reactive).
 *
 * Wiring:
 *   MQ-7 A0    → GPIO34 (ADC input only, tidak bisa output!)
 *   MQ-7 VCC   → Rail 5V (WAJIB 5V — heater sensor butuh 5V!)
 *   LED Hijau  → GPIO13 via 330Ω (AMAN)
 *   LED Kuning → GPIO14 via 330Ω (WASPADA)
 *   LED Merah  → GPIO27 via 330Ω (BAHAYA)
 *   UART TX    → GPIO25 → GPIO16 Master (BUKAN GPIO15!)
 *   GND        → Rail GND common
 *
 * Board: ESP32 Dev Module
 * ================================================================
 */

#include <Arduino.h>

// ─── PIN DEFINITIONS ─────────────────────────────────────────────
#define PIN_MQ7       34   // ADC input only — tidak bisa dipakai sebagai output
#define PIN_LED_H     13   // LED Hijau  = status AMAN
#define PIN_LED_K     14   // LED Kuning = status WASPADA
#define PIN_LED_M     27   // LED Merah  = status BAHAYA
#define UART_TX_PIN   25   // TX ke GPIO16 Master (BUKAN GPIO15!)

// ─── SENSOR CONSTANTS ────────────────────────────────────────────
// RL_MQ7: Resistor beban di PCB modul MQ-7
// Ukur dengan multimeter di PCB modul — nilai datasheet 10kΩ
// tapi modul murah sering beda. Hasil ukur modul ini: 0.95 kΩ (950 Ω)
#define RL_MQ7         0.95f

#define VCC_ADC         3.3f   // Referensi ADC ESP32 = 3.3V

// Rs/Ro di udara bersih untuk MQ-7 (dari kurva sensitivity datasheet)
// Digunakan untuk menghitung R0: R0 = RS_clean / RSRO_CLEAN
#define RSRO_CLEAN_MQ7  9.6f

// Rumus konversi Rs/R0 → ppm CO (power law dari grafik datasheet)
// Sumber: Andhika et al., JTECE Vol.7 No.1, 2025
// Formula: ppm = PPM_A × (Rs/R0)^PPM_B
#define PPM_A_CO       96.7924f
#define PPM_B_CO       -1.5277f   // negatif: makin tinggi Rs/R0, makin kecil ppm

// ─── THRESHOLD (ppm CO) ──────────────────────────────────────────
// Sumber: Permenaker No. 5 Tahun 2018 (Nilai Ambang Batas CO = 25 ppm)
//         WHO Guideline: 9 ppm (8 jam), 26 ppm (1 jam)
#define THR_WASPADA_CO  10.0f   // mulai waspada — mendekati NAB
#define THR_BAHAYA_CO   25.0f   // level  berbahaya — NAB Permenaker

// ─── PREPROCESSING: EMA + OUTLIER REJECTION ──────────────────────
// EMA (Exponential Moving Average):
//   Formula: EMA_baru = α × nilai_raw + (1-α) × EMA_lama
//   α = 0.10: lebih smooth, cocok untuk CO yang naik perlahan
// Outlier Rejection:
//   Tolak spike jika delta > 50% dari EMA — noise elektrik
#define EMA_ALPHA_CO    0.10f
#define OUTLIER_MAX_CO  0.50f

float g_ema_co  = -1.0f;   // -1 = belum diinisialisasi
float g_prev_co =  0.0f;   // nilai raw sebelumnya (outlier reference)

// ─── KALIBRASI R0 ────────────────────────────────────────────────
// R0 = resistansi sensor di udara bersih (titik referensi kalibrasi)
float g_R0_co = 1.0f;   // diisi saat calibrateR0() dipanggil

// ─── WARMUP ──────────────────────────────────────────────────────
// First use (sensor baru): 24-48 jam di power
// Session berikutnya: 3 menit sudah cukup
#define WARMUP_MS  (3UL * 60UL * 1000UL)

// ════════════════════════════════════════════════════════════════

float readADC_avg() {
  // Rata-rata 5 sample — kurangi noise ADC ESP32 yang notoriously noisy
  long sum = 0;
  for (int i = 0; i < 5; i++) { sum += analogRead(PIN_MQ7); delay(2); }
  return (float)(sum / 5);
}

float adcToVoltage(float adc) {
  return (adc / 4095.0f) * VCC_ADC;
}

float voltageToRS(float v) {
  // RS = ((VCC - Vout) / Vout) × RL
  if (v < 0.01f) v = 0.01f;   // hindari pembagian dengan nol
  return ((VCC_ADC - v) / v) * RL_MQ7;
}

float rsToPpm_CO(float ratio) {
  if (ratio <= 0.0f) return 0.0f;
  return max(0.0f, PPM_A_CO * pow(ratio, PPM_B_CO));
}

void calibrateR0() {
  // Kalibrasi 60 detik: ukur RS di udara bersih, hitung R0
  // PENTING: jauhkan sensor dari gas apapun saat kalibrasi!
  Serial.println("[CAL] Kalibrasi R0 MQ-7 (60 detik)...");
  Serial.println("[CAL] Pastikan udara di sekitar sensor BERSIH!");
  float rs_sum = 0.0f;
  for (int i = 0; i < 60; i++) {
    rs_sum += voltageToRS(adcToVoltage(readADC_avg()));
    if ((i + 1) % 15 == 0)
      Serial.printf("[CAL] %d/60 selesai\n", i + 1);
    delay(1000);
  }
  g_R0_co = (rs_sum / 60.0f) / RSRO_CLEAN_MQ7;
  Serial.printf("[CAL] R0 CO = %.4f kΩ\n\n", g_R0_co);
}

float readCO() {
  // Pipeline: ADC → voltage → RS → ppm raw → outlier check → EMA
  float raw = rsToPpm_CO(
    voltageToRS(adcToVoltage(readADC_avg())) / g_R0_co
  );
  raw = constrain(raw, 0.0f, 5000.0f);

  // Outlier rejection: spike > 50% dari EMA → tolak
  if (g_ema_co >= 0.0f && g_ema_co > 1.0f)
    if (fabs(raw - g_ema_co) / g_ema_co > OUTLIER_MAX_CO)
      raw = g_prev_co;
  g_prev_co = raw;

  // EMA smoothing
  if (g_ema_co < 0.0f) g_ema_co = raw;   // inisialisasi pertama kali
  g_ema_co = EMA_ALPHA_CO * raw + (1.0f - EMA_ALPHA_CO) * g_ema_co;
  return g_ema_co;
}

String classifyCO(float ppm) {
  if (ppm >= THR_BAHAYA_CO)  return "bahaya";
  if (ppm >= THR_WASPADA_CO) return "waspada";
  return "aman";
}

void updateLEDs(const String& s) {
  // Hanya satu LED HIGH pada satu waktu
  digitalWrite(PIN_LED_H, s == "aman"    ? HIGH : LOW);
  digitalWrite(PIN_LED_K, s == "waspada" ? HIGH : LOW);
  digitalWrite(PIN_LED_M, s == "bahaya"  ? HIGH : LOW);
}

void blinkAll(int n, int on_ms, int off_ms) {
  // Kedip semua LED — dipakai sebagai indikator "sistem siap"
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED_H, HIGH); digitalWrite(PIN_LED_K, HIGH); digitalWrite(PIN_LED_M, HIGH);
    delay(on_ms);
    digitalWrite(PIN_LED_H, LOW);  digitalWrite(PIN_LED_K, LOW);  digitalWrite(PIN_LED_M, LOW);
    delay(off_ms);
  }
}

void sendToMaster(float ppm, const String& status) {
  // Kirim JSON setiap 1 detik via UART GPIO25 → GPIO16 Master
  // Format: {"src":"A","gas":"CO","ppm":23.45,"status":"waspada"}
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"A\",\"gas\":\"CO\",\"ppm\":%.2f,\"status\":\"%s\"}",
    ppm, status.c_str());
  Serial2.println(buf);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-A: CO Monitor (MQ-7) ===\n");
  Serial2.begin(9600, SERIAL_8N1, -1, UART_TX_PIN);   // TX only ke Master
  pinMode(PIN_LED_H, OUTPUT); pinMode(PIN_LED_K, OUTPUT); pinMode(PIN_LED_M, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // range ADC 0-3.3V

  // Warmup — LED Kuning nyala selama pemanasan sensor
  Serial.printf("[INIT] Warmup %lu detik...\n", WARMUP_MS / 1000);
  digitalWrite(PIN_LED_K, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < WARMUP_MS) {
    delay(15000);
    Serial.printf("[INIT] %lu/%lu detik\n", (millis()-t0)/1000, WARMUP_MS/1000);
  }
  digitalWrite(PIN_LED_K, LOW);

  calibrateR0();
  g_ema_co = rsToPpm_CO(voltageToRS(adcToVoltage(readADC_avg())) / g_R0_co);
  blinkAll(5, 100, 100);   // kedip 5x = siap!
  Serial.println("[INIT] Siap!\n");
}

void loop() {
  float  ppm    = readCO();
  String status = classifyCO(ppm);
  updateLEDs(status);
  sendToMaster(ppm, status);
  Serial.printf("[CO] %.2f ppm | %s\n", ppm, status.c_str());
  delay(1000);
}
```

---

### 4.2 FinalProjectMQ135.ino — ESP32-B (NOx Monitor)

```cpp
/*
 * ================================================================
 * ESP32-B: Sensor MQ-135 — Nitrogen Oxide (NOx) Monitor
 * ================================================================
 * Preprocessing : Median Filter (window=5) + EMA (α=0.15)
 * Output        : 3 LED + JSON via UART ke Master ESP32-S3
 *
 * Mengapa Median + EMA untuk MQ-135?
 *   MQ-135 lebih noisy dari MQ-7 karena cross-sensitive terhadap
 *   banyak gas (NOx, NH3, CO2, alkohol, benzol). Spike tajam sering
 *   terjadi. Median filter lebih tahan spike dibanding SMA/EMA saja.
 *   EMA setelah median untuk smoothing akhir.
 *
 * Wiring:
 *   MQ-135 A0  → GPIO34 (ADC input only)
 *   MQ-135 VCC → Rail 5V (WAJIB 5V!)
 *   LED Hijau  → GPIO13 via 330Ω (AMAN)
 *   LED Kuning → GPIO14 via 330Ω (WASPADA)
 *   LED Merah  → GPIO27 via 330Ω (BAHAYA)
 *   UART TX    → GPIO25 → GPIO15 Master (BUKAN GPIO16!)
 *   GND        → Rail GND common
 *
 * Board: ESP32 Dev Module
 * ================================================================
 */

#include <Arduino.h>

// ─── PIN DEFINITIONS ─────────────────────────────────────────────
#define PIN_MQ135     34
#define PIN_LED_H     33   // Hijau  = AMAN
#define PIN_LED_K     26   // Kuning = WASPADA
#define PIN_LED_M     27   // Merah  = BAHAYA
#define UART_TX_PIN   25   // TX ke GPIO15 Master (BUKAN GPIO16!)

// ─── SENSOR CONSTANTS ────────────────────────────────────────────
// Hasil ukur RL modul ini: 0.95 kΩ
#define RL_MQ135        0.95f
#define VCC_ADC         3.3f
// Rs/Ro udara bersih MQ-135 dari datasheet
#define RSRO_CLEAN_135  3.6f
// Rumus NOx: ppm = 78.2986 × (Rs/R0)^(-3.2482)
// Sumber: Andhika et al., JTECE Vol.7 No.1, 2025
#define PPM_A_NOX      78.2986f
#define PPM_B_NOX      -3.2482f

// ─── THRESHOLD (ppm NOx) ─────────────────────────────────────────
// Permenaker No. 5 Tahun 2018: NAB NO = 25 ppm
// Catatan: MQ-135 ukur NOx total (NO+NO2), tidak bisa bedakan jenisnya
#define THR_WASPADA_NOX  10.0f
#define THR_BAHAYA_NOX   25.0f

// ─── PREPROCESSING: MEDIAN FILTER ────────────────────────────────
// Circular buffer window=5 — ambil nilai tengah setelah sorting
// Lebih tahan impulse noise (spike tajam) dibanding moving average
#define MEDIAN_SIZE  5
float g_med_buf[MEDIAN_SIZE] = {0};
int   g_med_idx  = 0;
bool  g_med_full = false;

void pushMedian(float val) {
  g_med_buf[g_med_idx] = val;
  g_med_idx = (g_med_idx + 1) % MEDIAN_SIZE;
  if (g_med_idx == 0) g_med_full = true;
}

float getMedian() {
  int n = g_med_full ? MEDIAN_SIZE : g_med_idx;
  if (n == 0) return 0.0f;
  float s[MEDIAN_SIZE];
  for (int i = 0; i < n; i++) s[i] = g_med_buf[i];
  // Insertion sort — efisien untuk n kecil (≤ 10)
  for (int i = 1; i < n; i++) {
    float key = s[i]; int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
    s[j+1] = key;
  }
  return s[n / 2];   // elemen tengah = median
}

// ─── PREPROCESSING: EMA ──────────────────────────────────────────
// α = 0.15: sedikit lebih reactive dari MQ-7 (NOx bisa naik lebih cepat)
#define EMA_ALPHA_NOX   0.15f
float g_ema_nox = -1.0f;

// ─── KALIBRASI R0 ────────────────────────────────────────────────
// R0 = resistansi sensor di udara bersih (titik referensi kalibrasi)
float g_R0_nox = 1.0f;   // diisi saat calibrateR0() dipanggil

// ─── WARMUP ──────────────────────────────────────────────────────
// First use (sensor baru): 24-48 jam di power
// Session berikutnya: 3 menit sudah cukup
#define WARMUP_MS  (3UL * 60UL * 1000UL)

// ════════════════════════════════════════════════════════════════

float readADC_avg() {
  // Rata-rata 5 sample — kurangi noise ADC ESP32 yang notoriously noisy
  long sum = 0;
  for (int i = 0; i < 5; i++) { sum += analogRead(PIN_MQ135); delay(2); }
  return (float)(sum / 5);
}

float adcToVoltage(float adc) { return (adc / 4095.0f) * VCC_ADC; }

float voltageToRS(float v) {
  // RS = ((VCC - Vout) / Vout) × RL
  if (v < 0.01f) v = 0.01f;   // hindari pembagian dengan nol
  return ((VCC_ADC - v) / v) * RL_MQ135;
}

float rsToPpm_NOx(float ratio) {
  if (ratio <= 0.0f) return 0.0f;
  return max(0.0f, PPM_A_NOX * pow(ratio, PPM_B_NOX));
}

void calibrateR0() {
  // Kalibrasi 60 detik: ukur RS di udara bersih, hitung R0
  // PENTING: jauhkan sensor dari gas apapun saat kalibrasi!
  Serial.println("[CAL] Kalibrasi R0 MQ-135 (60 detik)...");
  Serial.println("[CAL] Pastikan udara di sekitar sensor BERSIH!");
  float rs_sum = 0.0f;
  for (int i = 0; i < 60; i++) {
    rs_sum += voltageToRS(adcToVoltage(readADC_avg()));
    if ((i + 1) % 15 == 0)
      Serial.printf("[CAL] %d/60 selesai\n", i + 1);
    delay(1000);
  }
  g_R0_nox = (rs_sum / 60.0f) / RSRO_CLEAN_135;
  Serial.printf("[CAL] R0 NOx = %.4f kΩ\n\n", g_R0_nox);
}

float readNOx() {
  // Pipeline: ADC → ppm raw → Median filter → EMA
  float raw = rsToPpm_NOx(voltageToRS(adcToVoltage(readADC_avg())) / g_R0_nox);
  raw = constrain(raw, 0.0f, 5000.0f);
  pushMedian(raw);                // masukkan ke circular buffer
  float filtered = getMedian();   // ambil median
  if (g_ema_nox < 0.0f) g_ema_nox = filtered;
  g_ema_nox = EMA_ALPHA_NOX * filtered + (1.0f - EMA_ALPHA_NOX) * g_ema_nox;
  return g_ema_nox;
}

String classifyNOx(float ppm) {
  if (ppm >= THR_BAHAYA_NOX)  return "bahaya";
  if (ppm >= THR_WASPADA_NOX) return "waspada";
  return "aman";
}

void updateLEDs(const String& s) {
  // Hanya satu LED HIGH pada satu waktu
  digitalWrite(PIN_LED_H, s == "aman"    ? HIGH : LOW);
  digitalWrite(PIN_LED_K, s == "waspada" ? HIGH : LOW);
  digitalWrite(PIN_LED_M, s == "bahaya"  ? HIGH : LOW);
}

void blinkAll(int n, int on_ms, int off_ms) {
  // Kedip semua LED — dipakai sebagai indikator "sistem siap"
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED_H, HIGH); digitalWrite(PIN_LED_K, HIGH); digitalWrite(PIN_LED_M, HIGH);
    delay(on_ms);
    digitalWrite(PIN_LED_H, LOW);  digitalWrite(PIN_LED_K, LOW);  digitalWrite(PIN_LED_M, LOW);
    delay(off_ms);
  }
}

void sendToMaster(float ppm, const String& status) {
  // Kirim JSON setiap 1 detik via UART GPIO25 → GPIO15 Master
  // Format: {"src":"B","gas":"NOx","ppm":12.34,"status":"waspada"}
  char buf[80];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"B\",\"gas\":\"NOx\",\"ppm\":%.2f,\"status\":\"%s\"}",
    ppm, status.c_str());
  Serial2.println(buf);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32-B: NOx Monitor (MQ-135) ===\n");
  Serial2.begin(9600, SERIAL_8N1, -1, UART_TX_PIN);   // TX only ke Master
  pinMode(PIN_LED_H, OUTPUT); pinMode(PIN_LED_K, OUTPUT); pinMode(PIN_LED_M, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // range ADC 0-3.3V

  // Warmup — LED Kuning nyala selama pemanasan sensor
  Serial.printf("[INIT] Warmup %lu detik...\n", WARMUP_MS / 1000);
  digitalWrite(PIN_LED_K, HIGH);
  unsigned long t0 = millis();
  while (millis() - t0 < WARMUP_MS) {
    delay(15000);
    Serial.printf("[INIT] %lu/%lu detik\n", (millis()-t0)/1000, WARMUP_MS/1000);
  }
  digitalWrite(PIN_LED_K, LOW);

  calibrateR0();
  // Inisialisasi buffer median dengan nilai pertama supaya tidak ada garbage
  float init_ppm = rsToPpm_NOx(voltageToRS(adcToVoltage(readADC_avg())) / g_R0_nox);
  for (int i = 0; i < MEDIAN_SIZE; i++) g_med_buf[i] = init_ppm;
  g_ema_nox = init_ppm;
  blinkAll(5, 100, 100);   // kedip 5x = siap!
  Serial.println("[INIT] Siap!\n");
}

void loop() {
  float  ppm    = readNOx();
  String status = classifyNOx(ppm);
  updateLEDs(status);
  sendToMaster(ppm, status);
  Serial.printf("[NOx] %.2f ppm | %s\n", ppm, status.c_str());
  delay(1000);
}
```

---

### 4.3 FinalProjectMaster.ino — ESP32-S3 Master

```cpp
/*
 * ================================================================
 * ESP32-S3 MASTER CONTROLLER v10 — FreeRTOS Architecture
 * ================================================================
 *
 * ─── ARSITEKTUR TASK ────────────────────────────────────────────
 *
 *  CORE 1 (Application):
 *    Task_SensorA  Prio 4  4096B  — baca Serial2 (ESP32-A, CO)
 *    Task_SensorB  Prio 4  4096B  — baca serialB (ESP32-B, NOx)
 *    Task_Control  Prio 5  6144B  — state + aktuator + emergency
 *
 *  CORE 0 (WiFi stack + Network):
 *    Task_Network  Prio 3 10240B  — WiFi/SIM808 + Telegram + GPS
 *    Task_Log      Prio 1  3072B  — Serial output thread-safe
 *
 * ─── KOMUNIKASI ANTAR TASK ──────────────────────────────────────
 *
 *  SensorA ─┐
 *            ├──► [qSensor 10×]  ──► Task_Control
 *  SensorB ─┘
 *
 *  Task_Control ──► [qTelegram 3×] ──► Task_Network
 *  Semua task   ──► [qLog 20×]     ──► Task_Log → Serial.print
 *
 *  Task_Network ──► semNetReady ──► Task_Control (tunggu network)
 *
 * ─── PRINSIP DESAIN ─────────────────────────────────────────────
 *  1. Serial.print() TIDAK thread-safe → semua via qLog
 *  2. vTaskDelay() BUKAN delay() → yield ke task lain
 *  3. State gas hanya dimiliki Task_Control (tidak perlu mutex)
 *  4. Serial1/SIM808 hanya diakses Task_Network (tidak perlu mutex)
 *  5. serialB hanya diakses Task_SensorB (tidak perlu mutex)
 *  6. Non-blocking char-by-char UART reading (bukan readStringUntil)
 *  7. Semua peripheral diinit di setup() SEBELUM task dibuat
 *
 * ─── TELEGRAM ────────────────────────────────────────────────────
 *  WiFi: POST application/x-www-form-urlencoded (lebih reliable dari JSON)
 *  SIM : GET dengan urlEncode() lengkap (termasuk karakter / ? = di Maps URL)
 *  urlEncode() helper dipakai kedua mode — konsisten, tidak ada duplikasi
 *
 * ─── SIM808 PWRKEY ────────────────────────────────────────────────
 *  isSIM808On(): strategi berlapis — boot messages + 5x AT retry
 *  SIM808 modul ini auto power-on saat VCC terhubung; PWRKEY adalah
 *  toggle, bukan 'always on'. Cek status dulu sebelum trigger PWRKEY.
 *
 * Library: ArduinoJson, ESP32Servo, EspSoftwareSerial (>=v6.16.1)
 * Board  : ESP32S3 Dev Module, USB CDC On Boot: Disabled
 * ================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <stdarg.h>  // untuk va_list di LOG()

// FreeRTOS — sudah built-in di ESP32 Arduino core
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ─── MODE SELECTION ──────────────────────────────────────────────
#define WIFI 0
#define SIM 1
#define USING WIFI

// ─── WIFI CONFIG (hanya dikompilasi jika USING == WIFI) ──────────
#if USING == WIFI
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#define WIFI_SSID "changeme"
#define WIFI_PASS "change_me"
#endif

// ─── TELEGRAM CONFIG ─────────────────────────────────────────────
// Cara dapat token: chat @BotFather → /newbot
// Cara dapat Chat ID: chat @userinfobot
#define TG_BOT_TOKEN "change:me"  // ← ganti!
#define TG_CHAT_ID "changeme"

// ─── SIM CONFIG (hanya dikompilasi jika USING == SIM) ────────────
#if USING == SIM
// APN operator: Telkomsel="internet", Indosat="indosatgprs", XL="internet"
#define SIM_APN "internet"  // ← sesuaikan dengan operator SIM card!
#endif

// ─── PIN DEFINITIONS ─────────────────────────────────────────────
#define PIN_RX_A 16
#define PIN_RX_B 15
#define PIN_SIM_TX 4
#define PIN_SIM_RX 5
#define PIN_PWRKEY 12
#define PIN_SERVO 6
#define PIN_RELAY 7
#define PIN_BUZZER 8

// ─── LOGIC LEVEL ─────────────────────────────────────────────────
// Relay Module 5V Active HIGH — kontrol fan
//   IN = LOW  → relay ON  → fan nyala
//   IN = HIGH → relay OFF → fan mati
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// Buzzer Module Low Level Trigger via TXS0108E Level Shifter
//   GPIO8 (3.3V) → TXS0108E A1 → B1 (5V) → Buzzer I/O
//   I/O = LOW  → transistor PNP ON  → bunyi
//   I/O = HIGH → transistor PNP OFF → diam (butuh 5V, bukan 3.3V!)
//   TXS0108E memastikan B1 = 5V saat GPIO HIGH → PNP mati sempurna
#define BUZZER_ON LOW    // GPIO LOW  → TXS0108E → I/O 0V  → bunyi
#define BUZZER_OFF HIGH  // GPIO HIGH → TXS0108E → I/O 5V  → diam

// ─── SERVO POSITIONS ─────────────────────────────────────────────
#define SERVO_AMAN 0
#define SERVO_WASPADA 90
#define SERVO_BAHAYA 180

// ─── EMERGENCY TIMER ─────────────────────────────────────────────
#define EMERGENCY_MS (120UL * 1000UL)

// ─── RTOS CONFIG ─────────────────────────────────────────────────
// Stack sizes (bytes) — tune dengan uxTaskGetStackHighWaterMark()
#define STACK_SENSOR_A 4096
#define STACK_SENSOR_B 4096
#define STACK_CONTROL 6144
#define STACK_NETWORK 10240  // besar untuk HTTPS + JSON + GPS
#define STACK_LOG 3072

// Priorities (1=lowest, 5=highest, jangan pakai 0 atau >5)
#define PRIO_LOG 1
#define PRIO_NETWORK 3
#define PRIO_SENSOR 4
#define PRIO_CONTROL 5

// ─── STRUCTS ─────────────────────────────────────────────────────

// Data sensor dari ESP32-A atau B → Task_Control
typedef struct {
  char src;  // 'A' (CO) atau 'B' (NOx)
  float ppm;
  char status[10];  // "aman" / "waspada" / "bahaya"
} SensorData_t;

// Pesan Telegram dari Task_Control → Task_Network
// Pesan sudah diformat lengkap — Task_Network tinggal kirim + GPS
typedef struct {
  char text[480];   // pesan darurat (tanpa GPS)
  bool requestGPS;  // true = ambil GPS dulu sebelum kirim
} TelegramMsg_t;

// Pesan log dari semua task → Task_Log → Serial
typedef struct {
  char text[128];
} LogMsg_t;

// ─── RTOS HANDLES ────────────────────────────────────────────────
// Queues
static QueueHandle_t qSensor = nullptr;    // SensorA/B → Control
static QueueHandle_t qTelegram = nullptr;  // Control → Network
static QueueHandle_t qLog = nullptr;       // semua → Log

// Semaphore
static SemaphoreHandle_t semNetReady = nullptr;  // Network → Control

// Task handles (untuk uxTaskGetStackHighWaterMark diagnostics)
static TaskHandle_t hSensorA = nullptr;
static TaskHandle_t hSensorB = nullptr;
static TaskHandle_t hControl = nullptr;
static TaskHandle_t hNetwork = nullptr;
static TaskHandle_t hLog = nullptr;

// ─── OBJECTS ─────────────────────────────────────────────────────
static Servo g_servo;
static SoftwareSerial serialB(PIN_RX_B, -1);  // RX only, Core 1 only!

// ─── GAS STATE (owned by Task_Control — no mutex needed) ─────────
static float g_co_ppm = 0.0f;
static float g_nox_ppm = 0.0f;
static char g_co_status[10] = "aman";
static char g_nox_status[10] = "aman";

static unsigned long g_bahaya_since = 0;
static bool g_in_bahaya = false;
static bool g_emergency_sent = false;

static char g_gps_lat[20] = "";
static char g_gps_lon[20] = "";

// ─── BUZZER STATE (owned by Task_Control) ────────────────────────
enum BuzzerMode { BUZZ_OFF,
                  BUZZ_SLOW,
                  BUZZ_FAST };
static BuzzerMode g_buzz_mode = BUZZ_OFF;
static bool g_buzz_state = false;
static unsigned long g_buzz_last = 0;

// ════════════════════════════════════════════════════════════════
// LOG HELPER — thread-safe, semua task pakai ini
// Pakai vsnprintf untuk format string, kirim ke qLog
// Tidak blocking (0 timeout) — drop jika queue penuh
// ════════════════════════════════════════════════════════════════
static void LOG(const char* fmt, ...) {
  if (qLog == nullptr) return;
  LogMsg_t msg;
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg.text, sizeof(msg.text), fmt, args);
  va_end(args);
  // pdMS_TO_TICKS(0) = non-blocking, drop jika queue penuh
  xQueueSend(qLog, &msg, 0);
}

// ─── URL ENCODER — identik di WiFi dan SIM mode ──────────────────
static String urlEncode(const String& msg) {
  String enc = msg;
  enc.replace("%", "%25");  // HARUS pertama!
  enc.replace(" ", "%20");
  enc.replace("\n", "%0A");
  enc.replace("\r", "");
  enc.replace("!", "%21");
  enc.replace("#", "%23");
  enc.replace("&", "%26");
  enc.replace("(", "%28");
  enc.replace(")", "%29");
  enc.replace("+", "%2B");
  enc.replace(",", "%2C");
  enc.replace("/", "%2F");
  enc.replace(":", "%3A");
  enc.replace(";", "%3B");
  enc.replace("<", "%3C");
  enc.replace("=", "%3D");
  enc.replace(">", "%3E");
  enc.replace("?", "%3F");
  enc.replace("@", "%40");
  return enc;
}

// ─── NON-BLOCKING LINE READER ────────────────────────────────────
// Baca char by char dari HardwareSerial/SoftwareSerial tanpa blokir
// Simpan state di buf + pos yang persisten antar panggilan
// Return true ketika satu baris lengkap tersedia
static bool readLineNB(Stream& ser, char* buf, size_t maxLen, size_t& pos) {
  while (ser.available()) {
    char c = (char)ser.read();
    if (c == '\n') {
      buf[pos] = '\0';
      size_t len = pos;
      pos = 0;
      return len > 5;  // valid jika lebih dari 5 char (JSON minimal)
    }
    if (c != '\r' && pos < maxLen - 1) {
      buf[pos++] = c;
    }
  }
  return false;  // baris belum lengkap
}

// ─── JSON PARSER ─────────────────────────────────────────────────
// Parse JSON dari sensor: {"src":"A","gas":"CO","ppm":23.45,"status":"waspada"}
// Return false jika JSON invalid atau field hilang
static bool parseJSON(const char* line, SensorData_t* out) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return false;
  const char* src = doc["src"] | "";
  float ppm = doc["ppm"] | -1.0f;
  const char* status = doc["status"] | "";
  if (src[0] == '\0' || ppm < 0.0f || status[0] == '\0') return false;
  out->src = src[0];
  out->ppm = ppm;
  strncpy(out->status, status, sizeof(out->status) - 1);
  out->status[sizeof(out->status) - 1] = '\0';
  return true;
}

// ─── STATUS LOGIC ────────────────────────────────────────────────
static int statusRank(const char* s) {
  if (strcmp(s, "bahaya") == 0) return 2;
  if (strcmp(s, "waspada") == 0) return 1;
  return 0;
}

static const char* combinedStatus() {
  int r = max(statusRank(g_co_status), statusRank(g_nox_status));
  if (r == 2) return "bahaya";
  if (r == 1) return "waspada";
  return "aman";
}

// ─── BUZZER NON-BLOCKING (dipanggil tiap 10ms dari Task_Control) ──
static void updateBuzzer() {
  if (g_buzz_mode == BUZZ_OFF) {
    digitalWrite(PIN_BUZZER, BUZZER_OFF);
    g_buzz_state = false;
    return;
  }
  // SLOW: 100ms ON / 2000ms OFF | FAST: 200ms ON / 200ms OFF
  unsigned long on_ms = (g_buzz_mode == BUZZ_FAST) ? 200UL : 100UL;
  unsigned long off_ms = (g_buzz_mode == BUZZ_FAST) ? 200UL : 2000UL;
  unsigned long interval = g_buzz_state ? on_ms : off_ms;
  if (millis() - g_buzz_last >= interval) {
    g_buzz_state = !g_buzz_state;
    digitalWrite(PIN_BUZZER, g_buzz_state ? BUZZER_ON : BUZZER_OFF);
    g_buzz_last = millis();
  }
}

// ─── ACTUATOR CONTROL ────────────────────────────────────────────
static void controlActuators(const char* status) {
  if (strcmp(status, "bahaya") == 0) {
    g_servo.write(SERVO_BAHAYA);
    digitalWrite(PIN_RELAY, RELAY_ON);
    g_buzz_mode = BUZZ_FAST;
  } else if (strcmp(status, "waspada") == 0) {
    g_servo.write(SERVO_WASPADA);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    g_buzz_mode = BUZZ_SLOW;
  } else {
    g_servo.write(SERVO_AMAN);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    g_buzz_mode = BUZZ_OFF;
  }
}

// ════════════════════════════════════════════════════════════════
// SIM808 FUNCTIONS — hanya dipanggil dari Task_Network (Core 0)
// Serial1 owned by Task_Network, tidak perlu mutex
// ════════════════════════════════════════════════════════════════

static String simSend(const String& cmd, unsigned long timeout_ms = 3000UL) {
  while (Serial1.available()) Serial1.read();
  Serial1.println(cmd);
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    while (Serial1.available()) resp += (char)Serial1.read();
    if (resp.indexOf("OK") >= 0 || resp.indexOf("ERROR") >= 0) break;
    vTaskDelay(pdMS_TO_TICKS(10));  // yield ke task lain saat tunggu
  }
  LOG("[SIM] %s -> %s\n", cmd.c_str(), resp.c_str());
  return resp;
}

// Deteksi status SIM808 dengan strategi berlapis:
//   SIM808 modul ini auto power-on saat VCC terhubung sehingga
//   pada saat ESP32-S3 selesai boot, SIM808 mungkin sedang
//   mid-boot (5-8 detik). Cek AT langsung tanpa tunggu boot
//   messages akan gagal dan menyebabkan PWRKEY toggle (matikan SIM808).
//
//   Strategi 1: dengarkan boot messages (RDY/+CFUN/+CPIN) 8 detik
//   Strategi 2: jika tidak ada boot message, coba AT 5× retry
static bool isSIM808On() {
  String buf = "";
  while (Serial1.available()) buf += (char)Serial1.read();

  // Strategi 1: dengarkan boot messages
  LOG("[SIM808] Mendengarkan boot messages (maks 8 detik)...\n");
  unsigned long t1 = millis();
  while (millis() - t1 < 8000) {
    while (Serial1.available()) buf += (char)Serial1.read();
    if (buf.indexOf("RDY") >= 0 || buf.indexOf("Call Ready") >= 0 || buf.indexOf("+CFUN") >= 0 || buf.indexOf("+CPIN") >= 0 || buf.indexOf("SMS Ready") >= 0) {
      LOG("[SIM808] Boot message terdeteksi -> ON\n");
      vTaskDelay(pdMS_TO_TICKS(2000));  // tunggu sisa URC selesai
      while (Serial1.available()) Serial1.read();
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // Strategi 2: AT command retry
  LOG("[SIM808] Coba AT command (5 percobaan)...\n");
  for (int i = 0; i < 5; i++) {
    while (Serial1.available()) Serial1.read();
    Serial1.println("AT");
    String resp = "";
    unsigned long t2 = millis();
    while (millis() - t2 < 2000) {
      while (Serial1.available()) resp += (char)Serial1.read();
      if (resp.indexOf("OK") >= 0) {
        LOG("[SIM808] AT OK (%d/5) -> ON\n", i + 1);
        return true;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    LOG("[SIM808] AT percobaan %d/5 tidak ada respons\n", i + 1);
    if (i < 4) vTaskDelay(pdMS_TO_TICKS(500));
  }

  LOG("[SIM808] Tidak ada respons -> OFF\n");
  return false;
}

static void powerOnSIM808() {
  LOG("[SIM808] Cek status...\n");
  if (isSIM808On()) {
    LOG("[SIM808] Sudah ON - skip PWRKEY\n");
    return;
  }
  LOG("[SIM808] OFF - trigger PWRKEY...\n");
  pinMode(PIN_PWRKEY, OUTPUT);
  digitalWrite(PIN_PWRKEY, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(PIN_PWRKEY, LOW);
  vTaskDelay(pdMS_TO_TICKS(1500));
  pinMode(PIN_PWRKEY, INPUT);
  vTaskDelay(pdMS_TO_TICKS(5000));
  LOG("[SIM808] Boot selesai.\n");
}

static bool getGPS() {
  simSend("AT+CGNSPWR=1", 2000);
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (int i = 0; i < 20; i++) {
    String resp = simSend("AT+CGNSINF", 2000);
    int idx = resp.indexOf("+CGNSINF:");
    if (idx < 0) {
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    String data = resp.substring(idx + 9);
    String fields[6];
    int fi = 0, prev = 0;
    for (int c = 0; c < (int)data.length() && fi < 5; c++)
      if (data[c] == ',') {
        fields[fi++] = data.substring(prev, c);
        prev = c + 1;
      }
    if (fields[1] == "1") {
      fields[3].trim();
      fields[4].trim();
      strncpy(g_gps_lat, fields[3].c_str(), sizeof(g_gps_lat) - 1);
      strncpy(g_gps_lon, fields[4].c_str(), sizeof(g_gps_lon) - 1);
      LOG("[GPS] Fix! Lat=%s Lon=%s\n", g_gps_lat, g_gps_lon);
      return true;
    }
    LOG("[GPS] Belum fix (%d/20)...\n", i + 1);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
  LOG("[GPS] Gagal fix.\n");
  return false;
}

// ════════════════════════════════════════════════════════════════
// WIFI MODE
// ════════════════════════════════════════════════════════════════
#if USING == WIFI
static void initWiFi() {
  LOG("[WiFi] Konek ke %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000)
    vTaskDelay(pdMS_TO_TICKS(500));
  if (WiFi.status() == WL_CONNECTED)
    LOG("[WiFi] Terhubung! IP: %s\n", WiFi.localIP().toString().c_str());
  else
    LOG("[WiFi] GAGAL!\n");
}

static bool sendTelegram(const String& msg) {
  if (WiFi.status() != WL_CONNECTED) {
    LOG("[WiFi-TG] No WiFi!\n");
    return false;
  }
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, "https://api.telegram.org/bot" + String(TG_BOT_TOKEN) + "/sendMessage");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  // form-urlencoded: tidak perlu JSON escaping, jauh lebih simpel dan reliable
  String body = "chat_id=" + String(TG_CHAT_ID) + "&text=" + urlEncode(msg);
  int code = http.POST(body);
  http.end();
  LOG("[WiFi-TG] HTTP %d\n", code);
  return code == 200;
}
#endif

// ════════════════════════════════════════════════════════════════
// SIM MODE
// ════════════════════════════════════════════════════════════════
#if USING == SIM
static bool connectGPRS() {
  simSend("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  simSend("AT+SAPBR=3,1,\"APN\",\"" + String(SIM_APN) + "\"");
  return simSend("AT+SAPBR=1,1", 10000UL).indexOf("ERROR") < 0;
}

static void disconnectGPRS() {
  simSend("AT+SAPBR=0,1", 5000UL);
}

static bool sendTelegram(const String& msg) {
  if (!connectGPRS()) {
    LOG("[SIM-TG] GPRS gagal!\n");
    return false;
  }
  String url = "https://api.telegram.org/bot" + String(TG_BOT_TOKEN)
               + "/sendMessage?chat_id=" + String(TG_CHAT_ID)
               + "&text=" + urlEncode(msg);  // urlEncode sama dengan WiFi mode
  simSend("AT+HTTPINIT");
  simSend("AT+HTTPPARA=\"CID\",1");
  simSend("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  simSend("AT+HTTPSSL=1");
  simSend("AT+HTTPACTION=0", 15000UL);
  vTaskDelay(pdMS_TO_TICKS(5000));
  String resp = simSend("AT+HTTPREAD", 5000UL);
  simSend("AT+HTTPTERM");
  disconnectGPRS();
  bool ok = resp.indexOf("\"ok\":true") >= 0;
  LOG("[SIM-TG] %s\n", ok ? "Terkirim!" : "Gagal!");
  return ok;
}
#endif

// ════════════════════════════════════════════════════════════════
// TASK 1 & 2: SENSOR A & B
// Core 1, Priority 4
// Non-blocking char-by-char UART reading — tidak ada timeout blocking!
// State reader (buf + pos) persisten di stack task masing-masing
// ════════════════════════════════════════════════════════════════
static void Task_SensorA(void* pv) {
  char buf[128];
  size_t pos = 0;
  SensorData_t data;

  for (;;) {
    if (readLineNB(Serial2, buf, sizeof(buf), pos)) {
      if (parseJSON(buf, &data)) {
        xQueueSend(qSensor, &data, pdMS_TO_TICKS(10));
        LOG("[RX-A] CO=%.2f | %s\n", data.ppm, data.status);
      }
    }
    // 5ms yield — tidak blocking, tapi tetap beri giliran task lain
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void Task_SensorB(void* pv) {
  char buf[128];
  size_t pos = 0;
  SensorData_t data;

  for (;;) {
    // serialB (SoftwareSerial) — hanya diakses dari task ini!
    if (readLineNB(serialB, buf, sizeof(buf), pos)) {
      if (parseJSON(buf, &data)) {
        xQueueSend(qSensor, &data, pdMS_TO_TICKS(10));
        LOG("[RX-B] NOx=%.2f | %s\n", data.ppm, data.status);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ════════════════════════════════════════════════════════════════
// TASK 3: CONTROL
// Core 1, Priority 5 (tertinggi di Core 1)
// Owns: gas state, actuators, buzzer, emergency timer
// Waits for network ready before triggering emergency
// ════════════════════════════════════════════════════════════════
static void Task_Control(void* pv) {
  SensorData_t incoming;
  TelegramMsg_t tgMsg;

  // Tunggu network siap sebelum bisa kirim emergency
  // (semNetReady di-give oleh Task_Network setelah init selesai)
  LOG("[Control] Menunggu network siap...\n");
  xSemaphoreTake(semNetReady, portMAX_DELAY);
  LOG("[Control] Network siap - mulai kontrol sistem\n");

  for (;;) {
    // Drain semua data sensor di queue (non-blocking, take all available)
    while (xQueueReceive(qSensor, &incoming, 0) == pdTRUE) {
      if (incoming.src == 'A') {
        g_co_ppm = incoming.ppm;
        strncpy(g_co_status, incoming.status, sizeof(g_co_status) - 1);
      } else if (incoming.src == 'B') {
        g_nox_ppm = incoming.ppm;
        strncpy(g_nox_status, incoming.status, sizeof(g_nox_status) - 1);
      }
    }

    const char* combined = combinedStatus();

    // Update aktuator
    controlActuators(combined);

    // Update buzzer — 10ms precision berkat vTaskDelay(10) di bawah
    updateBuzzer();

    // Emergency timer
    if (strcmp(combined, "bahaya") == 0) {
      if (!g_in_bahaya) {
        g_in_bahaya = true;
        g_bahaya_since = millis();
        g_emergency_sent = false;
        LOG("[Timer] Status BAHAYA - timer dimulai\n");
      }
      if (!g_emergency_sent && millis() - g_bahaya_since >= EMERGENCY_MS) {
        // Format pesan darurat (tanpa GPS — Task_Network yang ambil GPS)
        snprintf(tgMsg.text, sizeof(tgMsg.text),
                 "DARURAT! Gas berbahaya di kabin kendaraan!\n"
                 "CO  : %.1f ppm (%s)\n"
                 "NOx : %.1f ppm (%s)\n"
                 "Servo/Fan tidak efektif selama >120 detik.",
                 g_co_ppm, g_co_status,
                 g_nox_ppm, g_nox_status);
        tgMsg.requestGPS = true;

        // Kirim ke Task_Network — tidak blocking (timeout 100ms)
        // Task_Control TIDAK berhenti! Network yang handle GPS + Telegram
        if (xQueueSend(qTelegram, &tgMsg, pdMS_TO_TICKS(100)) == pdTRUE) {
          g_emergency_sent = true;  // akan di-retry jika Network gagal? flag ini cukup
          LOG("[Control] Emergency dikirim ke Task_Network\n");
        } else {
          LOG("[Control] qTelegram penuh, retry...\n");
        }
      }
    } else {
      if (g_in_bahaya) LOG("[Timer] Aman. Timer reset.\n");
      g_in_bahaya = false;
      g_bahaya_since = 0;
    }

    // Log status sistem setiap 1 detik menggunakan millis()-based timing.
    // Lebih akurat dari counter iterasi karena vTaskDelay() tidak selalu
    // persis 10ms — scheduler bisa sedikit jitter tergantung beban Core 1.
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 1000) {
      LOG("[MASTER] CO=%.1f(%s) | NOx=%.1f(%s) | %s\n",
          g_co_ppm, g_co_status,
          g_nox_ppm, g_nox_status,
          combined);
      lastLog = millis();
    }

    // 10ms delay — cukup untuk buzzer presisi, yield ke sensor tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ════════════════════════════════════════════════════════════════
// TASK 4: NETWORK
// Core 0 (bersama WiFi stack ESP32) — BOLEH blocking lama!
// Handles: init WiFi/SIM808, GPS, Telegram
// Semua operasi network yang potentially long-blocking ada di sini
// Task_Control tidak pernah berhenti meski Network sedang sibuk
// ════════════════════════════════════════════════════════════════
static void Task_Network(void* pv) {
  // ─── INIT NETWORK ────────────────────────────────────────────
  Serial1.begin(9600, SERIAL_8N1, PIN_SIM_RX, PIN_SIM_TX);

  // SIM808 selalu diinit untuk GPS (tidak tergantung mode)
  LOG("[Network] Init SIM808...\n");
  powerOnSIM808();
  simSend("AT");
  simSend("AT+CPIN?");
  simSend("AT+CREG?");
  simSend("AT+CGNSPWR=1", 2000);
  LOG("[Network] SIM808 siap.\n");

#if USING == WIFI
  LOG("[Network] Mempersiapkan WiFi...\n");
  vTaskDelay(pdMS_TO_TICKS(10000));
  initWiFi();
#endif

  // Beri sinyal ke Task_Control bahwa network sudah siap
  xSemaphoreGive(semNetReady);
  LOG("[Network] semNetReady diberikan ke Task_Control\n");

  // ─── MAIN LOOP ───────────────────────────────────────────────
  TelegramMsg_t msg;
  for (;;) {
    // Tunggu pesan dari Task_Control (blocking — hemat CPU)
    if (xQueueReceive(qTelegram, &msg, portMAX_DELAY) == pdTRUE) {
      LOG("[Network] Terima pesan emergency, proses...\n");

      // Ambil GPS jika diminta (bisa blocking sampai 100 detik!)
      // Task_Control TIDAK terpengaruh — mereka di Core 1
      String loc = "GPS tidak tersedia";
      if (msg.requestGPS && getGPS()) {
        loc = "https://maps.google.com/?q=";
        loc += g_gps_lat;
        loc += ",";
        loc += g_gps_lon;
      }

      // Gabungkan pesan + lokasi GPS
      String fullMsg = String(msg.text) + "\nLokasi: " + loc;

      // Kirim Telegram
      bool ok = sendTelegram(fullMsg);
      LOG("[Network] Telegram: %s\n", ok ? "TERKIRIM" : "GAGAL");
    }
  }
}

// ════════════════════════════════════════════════════════════════
// TASK 5: LOG
// Core 0, Priority 1 (terendah — tidak ganggu task penting)
// Satu-satunya task yang boleh akses Serial.print()
// Semua task lain kirim via qLog
// Tambahan: stack diagnostic tiap 30 detik
// ════════════════════════════════════════════════════════════════
static void Task_Log(void* pv) {
  LogMsg_t msg;
  unsigned long lastDiag = 0;

  for (;;) {
    // Drain semua log yang pending (timeout 100ms jika kosong)
    while (xQueueReceive(qLog, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
      Serial.print(msg.text);
    }

    // Stack diagnostic tiap 30 detik — bantu tuning stack size
    if (millis() - lastDiag >= 30000) {
      Serial.printf("[DIAG] Stack HWM (words): SensorA=%u SensorB=%u Control=%u Network=%u Log=%u\n",
                    uxTaskGetStackHighWaterMark(hSensorA),
                    uxTaskGetStackHighWaterMark(hSensorB),
                    uxTaskGetStackHighWaterMark(hControl),
                    uxTaskGetStackHighWaterMark(hNetwork),
                    uxTaskGetStackHighWaterMark(hLog));
      Serial.printf("[DIAG] Queue spaces: Sensor=%u Telegram=%u Log=%u\n",
                    uxQueueSpacesAvailable(qSensor),
                    uxQueueSpacesAvailable(qTelegram),
                    uxQueueSpacesAvailable(qLog));
      lastDiag = millis();
    }
  }
}

// ════════════════════════════════════════════════════════════════
// SETUP
// Semua peripheral diinit DI SINI sebelum task dibuat!
// Setelah tasks berjalan, setup() tidak perlu melakukan apa-apa lagi
// ════════════════════════════════════════════════════════════════
void setup() {
  // Buzzer dan Relay diinisialisasi sebagai baris pertama mutlak,
  // sebelum Serial.begin() maupun delay() sekalipun.
  //
  // Buzzer modul Low Level Trigger: saat GPIO belum diinisialisasi (INPUT
  // mode di boot), tegangan pin tidak terdefinisi dan bisa memicu transistor
  // PNP internal → buzzer berbunyi saat startup. Dengan langsung set HIGH
  // (BUZZER_OFF) di sini, kondisi ini dicegah sejak detik pertama.
  // Relay Active LOW: alasan yang sama — set HIGH = fan mati dari awal.
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_OFF);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ESP32-S3 MASTER v9 — FreeRTOS ===");
#if USING == WIFI
  Serial.println("[MODE] WIFI\n");
#else
  Serial.println("[MODE] SIM\n");
#endif

  // UART sensor
  Serial2.begin(9600, SERIAL_8N1, PIN_RX_A, -1);

  // Pull-up internal pada GPIO15 sebelum serialB diinisialisasi.
  // EspSoftwareSerial pasang ISR pada pin RX saat begin() dipanggil.
  // Tanpa pull-up, GPIO15 floating ketika Box B belum dicolok →
  // noise elektrik terus memicu ISR → Task_SensorB tidak bisa yield →
  // scheduler Core 1 overloaded → Task_Control tidak pernah jalan.
  // Pull-up menstabilkan pin ke HIGH (idle UART). Ketika Box B dicolok,
  // sinyal TX ESP32-B secara otomatis mengambil alih — transparan.
  pinMode(PIN_RX_B, INPUT_PULLUP);
  serialB.begin(9600);

  // Servo
  g_servo.attach(PIN_SERVO);
  g_servo.write(SERVO_AMAN);

  // ─── BUAT RTOS PRIMITIVES ─────────────────────────────────────
  // Queues — buat sebelum task supaya task langsung bisa pakai
  qSensor = xQueueCreate(10, sizeof(SensorData_t));
  qTelegram = xQueueCreate(3, sizeof(TelegramMsg_t));
  qLog = xQueueCreate(20, sizeof(LogMsg_t));

  // Semaphore — network ready signal
  semNetReady = xSemaphoreCreateBinary();

  // Validasi — crash dengan pesan jelas jika gagal alokasi
  if (!qSensor || !qTelegram || !qLog || !semNetReady) {
    Serial.println("[FATAL] Gagal buat RTOS primitives! Cek RAM.");
    while (true) delay(1000);
  }

  Serial.println("[RTOS] Queues + semaphore dibuat.");

  // ─── BUAT TASKS ───────────────────────────────────────────────
  // Semua peripheral sudah diinit di atas — aman untuk buat task sekarang

  xTaskCreatePinnedToCore(Task_SensorA, "SensorA",
                          STACK_SENSOR_A, nullptr, PRIO_SENSOR, &hSensorA, 1);

  xTaskCreatePinnedToCore(Task_SensorB, "SensorB",
                          STACK_SENSOR_B, nullptr, PRIO_SENSOR, &hSensorB, 1);

  xTaskCreatePinnedToCore(Task_Control, "Control",
                          STACK_CONTROL, nullptr, PRIO_CONTROL, &hControl, 1);

  xTaskCreatePinnedToCore(Task_Network, "Network",
                          STACK_NETWORK, nullptr, PRIO_NETWORK, &hNetwork, 0);

  xTaskCreatePinnedToCore(Task_Log, "Log",
                          STACK_LOG, nullptr, PRIO_LOG, &hLog, 0);

  Serial.println("[RTOS] 5 tasks dibuat:");
  Serial.println("  Core 1: SensorA(P4) SensorB(P4) Control(P5)");
  Serial.println("  Core 0: Network(P3) Log(P1)");
  Serial.println("[INIT] Setup selesai - FreeRTOS scheduler aktif\n");

  // setup() selesai — FreeRTOS scheduler ambil alih
  // loop() hanya ada sebagai placeholder kosong
}

// ════════════════════════════════════════════════════════════════
// LOOP — KOSONG
// Semua pekerjaan ada di tasks.
// Hapus task idle default (loop task) dengan suspend diri sendiri.
// ════════════════════════════════════════════════════════════════
void loop() {
  // Suspend loop task supaya tidak buang CPU dengan idle spinning
  vTaskSuspend(nullptr);
}
```

---

### 4.4 FinalProjectNano3v3To5vRepeater.ino — Arduino Nano *

```cpp
/*
 * ================================================================
 * ARDUINO NANO — 3.3V TO 5V LOGIC FOLLOWER
 * ================================================================
 *
 * Fungsi:
 *   Arduino Nano dipakai sebagai penerjemah level tegangan dari ESP32
 *   ke modul relay dan buzzer 5V.
 *
 *   ESP32 mengeluarkan sinyal logic 3.3V.
 *   Arduino Nano membaca sinyal tersebut, lalu mengeluarkan sinyal
 *   logic yang sama pada level 5V.
 *
 * ─── HARDWARE YANG DIKONTROL ────────────────────────────────────
 *
 *   Relay/Fan:
 *     IN = LOW  / 0V → relay ON  → fan nyala
 *     IN = HIGH / 5V → relay OFF → fan mati
 *
 *   Buzzer:
 *     IO = LOW  / 0V → buzzer bunyi
 *     IO = HIGH / 5V → buzzer diam
 *
 *   Jadi relay dan buzzer sama-sama Active LOW.
 *
 * ─── PRINSIP LOGIC ──────────────────────────────────────────────
 *
 *   Karena Nano hanya mengikuti sinyal dari ESP32:
 *
 *     ESP32 LOW  → Nano output LOW  → device ON
 *     ESP32 HIGH → Nano output HIGH → device OFF
 *
 *   Dengan kata lain:
 *
 *     LOW  = ON
 *     HIGH = OFF
 *
 *   Logic utama tetap ditentukan oleh ESP32.
 *   Nano hanya bertugas menaikkan level tegangan dari 3.3V ke 5V.
 *
 * ─── MAPPING PIN ────────────────────────────────────────────────
 *
 *   ESP32 GPIO7 → Nano D2 → Nano D4 → Relay/Fan IN
 *   ESP32 GPIO8 → Nano D3 → Nano D5 → Buzzer IO
 *
 * ─── CATATAN INPUT ──────────────────────────────────────────────
 *
 *   Pin input Nano dari ESP32 menggunakan INPUT biasa.
 *   Jangan gunakan INPUT_PULLUP, karena pull-up internal Nano menuju
 *   5V dan berisiko memberi tegangan 5V balik ke pin ESP32.
 *
 * ================================================================
 */

const uint8_t IN_FAN_FROM_ESP  = 2;  // Nano D2 <- ESP32 GPIO7
const uint8_t IN_BUZZ_FROM_ESP = 3;  // Nano D3 <- ESP32 GPIO8

const uint8_t OUT_RELAY_FAN    = 4;  // Nano D4 -> Relay/Fan IN
const uint8_t OUT_BUZZER       = 5;  // Nano D5 -> Buzzer IO

void setup() {
  /*
   * Output disiapkan lebih dulu dan langsung dibuat OFF.
   *
   * Karena relay dan buzzer Active LOW:
   *
   *   HIGH = OFF
   *   LOW  = ON
   *
   * Maka kondisi aman saat startup adalah HIGH.
   */
  pinMode(OUT_RELAY_FAN, OUTPUT);
  pinMode(OUT_BUZZER, OUTPUT);

  digitalWrite(OUT_RELAY_FAN, HIGH);  // Fan OFF
  digitalWrite(OUT_BUZZER, HIGH);     // Buzzer OFF

  /*
   * Input dari ESP32.
   *
   * Tidak memakai INPUT_PULLUP agar tidak ada pull-up 5V dari Nano
   * menuju pin ESP32.
   */
  pinMode(IN_FAN_FROM_ESP, INPUT);
  pinMode(IN_BUZZ_FROM_ESP, INPUT);
}

void loop() {
  /*
   * Baca perintah dari ESP32.
   *
   * Karena ESP32 juga memakai logic Active LOW:
   *
   *   LOW  = ON
   *   HIGH = OFF
   */
  int fanCommand  = digitalRead(IN_FAN_FROM_ESP);
  int buzzCommand = digitalRead(IN_BUZZ_FROM_ESP);

  /*
   * Nano hanya mengikuti logic ESP32.
   *
   * Tidak ada pembalikan/invert di sini.
   *
   *   ESP32 LOW  -> Nano LOW  -> device ON
   *   ESP32 HIGH -> Nano HIGH -> device OFF
   */
  digitalWrite(OUT_RELAY_FAN, fanCommand);
  digitalWrite(OUT_BUZZER, buzzCommand);
}
```

\* Jika menggunakan MOSFET IRLZ44N sebagai level shifter, file ini tidak diperlukan. Koneksi langsung GPIO ESP32-S3 → MOSFET gate → Relay IN / Buzzer IO. Sesuaikan `RELAY_ON/OFF` dan `BUZZER_ON/OFF` di `FinalProjectMaster.ino` dengan logika Active HIGH MOSFET.

---

## Bagian 5 — Catatan Teknis

### 5.1 Manajemen Daya

Sistem menggunakan satu adaptor 12V 2A dengan dua jalur distribusi yang sepenuhnya terpisah:

```
Adaptor 12V 2A
├──► LM2596 (5.1V) ──► Rail 5V ──► ESP32-S3, ESP32-A, ESP32-B,
│                                  Nano, Relay, Buzzer, Servo,
│                              ──► Sensor MQ-7, Sensor MQ-135
│
└──► SIM808 VIN (langsung 12V) ──► SIM808 saja
```

SIM808 dipisah karena spike arus 2A saat GPRS transmisi dapat menyebabkan voltage drop pada Rail 5V dan memicu Brownout Reset (BOD) ESP32-S3.

**Kapasitor:**

| Posisi | Nilai | Fungsi |
|--------|-------|--------|
| Output LM2596 | 2 × 1000 µF paralel = 2000 µF | Buffer spike arus WiFi + aktuator |
| Pin 3.3V ESP32-S3 | 1 × 1000 µF | Stabilisasi 3.3V saat WiFi cold start |

**Penanganan Brownout Reset:**

Saat pertama kali boot, WiFi cold start calibration ESP32-S3 menarik ~300–400 mA dalam burst singkat. Tiga lapisan mitigasi diterapkan secara bersamaan:

1. Kapasitor 1000 µF di pin 3.3V — buffer langsung di titik yang dipantau BOD
2. SIM808 pada jalur 12V terpisah — menghilangkan kontribusi spike SIM808
3. Delay 10 detik di `Task_Network` antara SIM808 init dan WiFi init — memberi waktu Rail 5V stabil sebelum WiFi minta arus besar

---

### 5.2 Dua Mode Koneksi Internet

Pilihan mode ditentukan dengan satu konstanta di `FinalProjectMaster.ino`:

```cpp
#define USING WIFI   // ganti ke SIM untuk mode GPRS
```

**Mode WiFi** — development dan demonstrasi:
- `WiFiClientSecure` + `HTTPClient` POST `application/x-www-form-urlencoded`
- Koneksi lebih cepat dan stabil untuk testing iteratif
- Konfigurasi: ubah `WIFI_SSID` dan `WIFI_PASS`

**Mode SIM/GPRS** — deployment kendaraan:
- AT command SIM808: `AT+HTTPINIT` → `AT+HTTPSSL=1` → `AT+HTTPACTION=0`
- Tidak bergantung infrastruktur WiFi — cocok untuk kendaraan bergerak
- Konfigurasi: ubah `SIM_APN` sesuai operator
  - Telkomsel: `"internet"`, Indosat: `"indosatgprs"`, XL: `"internet"`

Keduanya menggunakan fungsi `urlEncode()` yang identik — karakter khusus dalam URL Google Maps (`/`, `?`, `=`, koordinat desimal) dikodekan dengan benar. GPS tersedia di kedua mode karena SIM808 selalu diinisialisasi.

---

### 5.3 Proses Kalibrasi Sensor

#### 5.3.1 Konsep R0 dan RS/R0

Sensor MQ menggunakan lapisan semikonduktor SnO₂ (tin dioxide) yang resistansinya berubah saat bereaksi dengan gas target. Resistansi sensor saat ini disebut **RS**. Karena setiap unit sensor memiliki karakteristik fisik yang berbeda (variasi manufaktur), RS absolut tidak dapat langsung digunakan untuk menghitung konsentrasi gas.

Solusinya adalah normalisasi: RS dibandingkan dengan **R0**, yaitu resistansi sensor yang sama diukur di udara bersih sebagai titik referensi. Rasio RS/R0 bersifat konsisten antar unit sensor dan menjadi dasar konversi ke ppm.

#### 5.3.2 Formula Kalibrasi R0

Datasheet mendefinisikan nilai RS/R0 di udara bersih sebagai konstanta `RSRO_CLEAN`:
- MQ-7: `RSRO_CLEAN = 9.6` → di udara bersih, RS = 9.6 × R0
- MQ-135: `RSRO_CLEAN = 3.6`

Dari definisi tersebut:

```
R0 = RS_clean / RSRO_CLEAN
```

Implementasi dalam kode (60 detik averaging):

```
Ukur RS setiap detik selama 60 detik di udara bersih
RS_rata_rata = Σ RS(t) / 60
R0 = RS_rata_rata / RSRO_CLEAN
```

#### 5.3.3 Konversi RS ke Tegangan ke R0

Pipeline lengkap dari pembacaan ADC hingga R0:

```
Langkah 1: ADC (12-bit, 0–4095) → Tegangan
  V_out = (ADC_avg / 4095) × 3.3 V
  ADC_avg = rata-rata 5 sample (mengurangi noise ADC ESP32)

Langkah 2: Tegangan → RS  (dari rangkaian voltage divider)
  RS = ((VCC − V_out) / V_out) × RL
  VCC = 3.3 V, RL = 0.95 kΩ (terukur dengan multimeter)

Langkah 3: RS di udara bersih → R0
  R0 = RS_clean_average / RSRO_CLEAN
```

#### 5.3.4 Proses Kalibrasi Development (Saat Ini)

1. Sensor dinyalakan, warmup 3 menit (LED Kuning menyala selama warmup)
2. Sensor ditempatkan di udara yang dianggap bersih di sekitar ruangan
3. Pengukuran RS dilakukan otomatis selama 60 detik
4. R0 dihitung dan disimpan di RAM (`g_R0_co`, `g_R0_nox`)
5. Sistem siap beroperasi (LED berkedip 5×)

#### 5.3.5 Keterbatasan Kalibrasi Development

| Aspek | Development (saat ini) | Production yang proper |
|-------|----------------------|----------------------|
| Durasi | 60 detik | 30–60 menit |
| Kondisi udara | Ruangan biasa | Certified clean air / N₂ murni |
| Burn-in sensor baru | Tidak dilakukan | 24–48 jam |
| Kompensasi suhu/RH | Tidak ada | Sensor DHT22 + koreksi formula |
| Verifikasi | Tidak ada | Reference gas bersertifikat |
| Rekalibrasi berkala | Setiap boot | Terjadwal 3–6 bulan |

---

### 5.4 Proses Preprocessing Sinyal Sensor

Pembacaan ADC mentah memiliki dua jenis gangguan yang perlu ditangani sebelum konversi ke ppm dan klasifikasi status:

- **Noise** — fluktuasi kecil berkelanjutan (±3–10 ppm) akibat noise elektrik ADC dan variasi minor sensor
- **Spike** — lonjakan tiba-tiba besar sesaat akibat gangguan elektromagnetik atau cross-sensitivity

#### 5.4.1 Pipeline Lengkap

```
ADC (0–4095)
    │
    │  V_out = (ADC / 4095) × 3.3V
    ▼
Tegangan V_out (0–3.3V)
    │
    │  RS = ((3.3 − V_out) / V_out) × RL
    ▼
Resistansi RS (kΩ)
    │
    │  ratio = RS / R0
    ▼
Rasio RS/R0
    │
    │  ppm = A × (RS/R0)^B
    ▼
PPM raw
    │
    ├── MQ-7  : Outlier Rejection → EMA (α=0.10)
    └── MQ-135: Median Filter (window=5) → EMA (α=0.15)
    │
    ▼
PPM filtered
    │
    │  bandingkan dengan threshold
    ▼
Status: aman / waspada / bahaya
```

#### 5.4.2 Konversi PPM — Power Law

Kurva sensitivitas datasheet dalam skala log-log membentuk garis lurus, yang dalam skala linear merupakan fungsi pangkat:

```
ppm = A × (RS/R0)^B
```

Koefisien dari Andhika et al., JTECE Vol.7 No.1, 2025:

| Sensor | A | B |
|--------|---|---|
| MQ-7 (CO) | 96.7924 | −1.5277 |
| MQ-135 (NOx) | 78.2986 | −3.2482 |

B negatif mencerminkan hubungan terbalik: semakin banyak gas, RS turun, RS/R0 turun, sehingga (RS/R0)^B naik → ppm naik.

#### 5.4.3 Preprocessing MQ-7 (CO): Outlier Rejection + EMA

**Outlier Rejection** — diterapkan sebelum EMA:

```
Jika |ppm_raw − EMA_sebelumnya| / EMA_sebelumnya > 0.50
  → tolak ppm_raw, gunakan EMA_sebelumnya
  → mencegah spike masuk ke filter
```

Threshold 50% dipilih karena lonjakan gas CO yang nyata (kebocoran sesungguhnya) umumnya naik secara bertahap, bukan tiba-tiba 2× lipat dalam satu detik.

**EMA (Exponential Moving Average)** dengan α = 0.10:

```
EMA_baru = 0.10 × ppm_filtered + 0.90 × EMA_sebelumnya
```

α = 0.10 (lambat): CO naik secara perlahan di dunia nyata — filter lambat dengan histori panjang lebih representatif. Filter agresif akan terlambat mendeteksi kenaikan bertahap.

#### 5.4.4 Preprocessing MQ-135 (NOx): Median Filter + EMA

**Alasan memilih Median Filter:**
MQ-135 lebih cross-sensitive dibanding MQ-7 — bereaksi terhadap CO₂, alkohol, NH₃, benzena, dll. Spike tajam terjadi lebih sering. EMA saja tidak cukup karena spike besar tetap mempengaruhi nilai rata-rata eksponensial.

**Median Filter** dengan window = 5:

```
Buffer sirkular 5 nilai terbaru:
  contoh: [4, 5, 80, 4, 5]
  diurutkan: [4, 4, 5, 5, 80]
  median (nilai tengah): 5  ← spike 80 dieliminasi sempurna
```

Median tidak terpengaruh spike selama spike < 50% dari window. EMA sesudah median (α = 0.15) untuk smoothing akhir. α sedikit lebih besar dari MQ-7 karena NOx dapat naik lebih cepat dari CO.

---

### 5.5 Pengembangan Lanjutan (Future Development)

#### 5.5.1 Penggantian Arduino Nano dengan MOSFET IRLZ44N

Penggunaan Arduino Nano sebagai level shifter adalah solusi darurat. Untuk implementasi yang lebih bersih dan efisien:

```
Sekarang (Nano):
  ESP32-S3 GPIO (3.3V) → Nano (50 mA) → Relay/Buzzer

Proposed (MOSFET per channel):
  ESP32-S3 GPIO (3.3V) → Gate IRLZ44N → Drain → Relay/Buzzer
  Konsumsi: ~1 mA per channel, tidak butuh kode tambahan
```

IRLZ44N adalah logic-level MOSFET dengan V_GS threshold 1–2V, fully ON di V_GS = 3.3V. Tidak memerlukan Arduino Nano, tidak ada latency forwarding, dan beban Rail 5V berkurang ~50 mA.

#### 5.5.2 Arsitektur Master Modular — Plug & Run Sensor Berbeda

Saat ini Master hardcoded untuk membaca sensor CO (sumber "A") dan NOx (sumber "B"). Pengembangan ke depan dapat membuat Master bersifat modular:

- JSON dari sensor menyertakan field `"type"` yang mendeskripsikan jenis gas dan koefisien
- Master tidak perlu tahu sensor apa yang terpasang — cukup baca field `ppm`, `status`, dan `type`
- Threshold dapat dikonfigurasi per tipe gas melalui NVS flash atau file konfigurasi
- Sensor baru cukup menyesuaikan firmware board sensornya saja, Master tidak perlu diubah

Ini memungkinkan sistem digunakan untuk monitoring gas lain (H₂S, LPG, NH₃, dll.) hanya dengan mengganti board sensor.

#### 5.5.3 Kalibrasi Production di Lingkungan Terkontrol

Kalibrasi 60 detik yang diimplementasikan saat ini adalah pendekatan development. Untuk deployment production yang akurat:

**Tahap 1 — Burn-in sensor baru:**
Sensor baru perlu dinyalakan 24–48 jam terus menerus sebelum dikalibrasi. Lapisan SnO₂ membutuhkan conditioning awal — tanpa burn-in, R0 tidak stabil dan akan bergeser selama minggu pertama penggunaan.

**Tahap 2 — Kalibrasi R0 multi-kondisi:**
Kalibrasi dilakukan dalam chamber terkontrol dengan:
- Certified clean air (atau N₂ murni)
- Minimal 3 kondisi suhu: 15°C, 25°C, 35°C
- Minimal 2 kondisi kelembapan: 40% RH, 70% RH
- Durasi minimum 30 menit per kondisi

Hasil: matriks R0(T, RH) yang digunakan untuk interpolasi saat runtime.

**Tahap 3 — Verifikasi dengan gas referensi bersertifikat:**
Setelah kalibrasi R0, sistem diuji dengan gas referensi berkonsentrasi terukur (certified calibration gas, misalnya CO 25 ppm ± 2%). Jika pembacaan sistem menyimpang, koefisien A disesuaikan:

```
A_terkoreksi = A_asli × (ppm_referensi / ppm_terbaca)
```

**Tahap 4 — Rekalibrasi berkala:**
Sensor MQ mengalami drift ~5–10% per bulan. Jadwal rekalibrasi:
- Rekalibrasi R0: setiap 3–6 bulan
- Verifikasi dengan reference gas: setiap 1–2 tahun

**Tahap 5 — Kompensasi suhu dan kelembapan real-time:**
Penambahan sensor DHT22 atau BME280 untuk mengukur T dan RH ambient, digunakan untuk mengkoreksi pembacaan RS secara real-time berdasarkan matriks kalibrasi yang tersimpan.
