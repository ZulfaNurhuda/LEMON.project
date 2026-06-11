/*
 * ================================================================
 * ESP32-S3 MASTER CONTROLLER v9 — FreeRTOS Architecture
 * ================================================================
 * PERUBAHAN v9 — Refactor ke RTOS Multi-Task:
 *
 *  Tantangan utama: operasi network (GPS hingga 100 detik, Telegram
 *  hingga 20 detik) tidak boleh menghentikan pembacaan sensor dan
 *  respons aktuator. Solusi: pisahkan concern ke task terpisah:
 *    → Task_Network di Core 0 boleh blocking selama yang diperlukan
 *    → Task_SensorA/B + Task_Control di Core 1 tetap berjalan
 *    → Emergency handling seamless tanpa membekukan sistem
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