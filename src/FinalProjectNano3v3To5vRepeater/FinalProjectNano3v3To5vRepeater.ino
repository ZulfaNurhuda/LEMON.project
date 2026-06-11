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