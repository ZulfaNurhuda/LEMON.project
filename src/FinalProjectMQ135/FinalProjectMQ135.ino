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