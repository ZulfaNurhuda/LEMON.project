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