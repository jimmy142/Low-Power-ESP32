// ESP32-S3 → BMP280 one-shot every 5 min; buffer 10 readings across deep sleep;
// on every 10th wake, transmit all 10 readings in a single ESP-NOW V3 packet.
// I2C: SDA=12, SCL=13; Battery divider on GPIO 8 (50/50). Channel 11.

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_idf_version.h>
#include "driver/adc.h"

#define SDA_PIN 12
#define SCL_PIN 13
#define BAT_PIN 8

#define WIFI_CH 11
#define SLEEP_SECONDS 300        // 5 minutes
#define TX_EVERY      10         // send batch every 10th wake

// Used only to lock the radio quickly to channel 11
const char* WIFI_SSID = "WiFI SSID";
const char* WIFI_PASS = "PASSWORD";

// Battery thresholds
#define BAT_CUTOFF_V  3.30f
#define BAT_RESUME_V  3.45f

Adafruit_BMP280 bmp;

// -------- Packet + RTC buffers --------
struct Reading { float t_c, p_hpa, b_v; uint32_t ts; };

struct SensorPktV3 {
  uint32_t node_uid;
  uint16_t count;     // <= 10
  uint16_t reserved;  // align
  uint32_t seq;       // batch sequence
  struct { float t_c, p_hpa, b_v; uint32_t ts; } s[10];
};

RTC_DATA_ATTR uint32_t rtc_seq        = 0;
RTC_DATA_ATTR uint32_t rtc_wake_count = 0;      // modulo TX_EVERY
RTC_DATA_ATTR bool     rtc_lowbat     = false;
RTC_DATA_ATTR Reading  rtc_buf[10];            // circular buffer
RTC_DATA_ATTR uint32_t rtc_idx        = 0;     // total readings stored so far

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
static void onSend(const wifi_tx_info_t*, esp_now_send_status_t) {}
#else
static void onSend(const uint8_t*, esp_now_send_status_t) {}
#endif

static uint32_t uid32() { return (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFFULL); }

// --- BMP280 forced (one-shot) read ---
static bool bmp_one_shot(float& tC, float& pHpa) {
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) return false;
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X8,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_125);
  bmp.takeForcedMeasurement();
  tC   = bmp.readTemperature();
  pHpa = bmp.readPressure() / 100.0f;
  return !(isnan(tC) || isnan(pHpa));
}

// --- Battery read on 50/50 divider ---
static float read_battery_v() {
  analogSetPinAttenuation(BAT_PIN, ADC_11db);
  const int N = 8;
  long mv_sum = 0;
  for (int i = 0; i < N; ++i) { mv_sum += analogReadMilliVolts(BAT_PIN); delayMicroseconds(300); }
  float pin_mv = mv_sum / (float)N;
  return (pin_mv * 2.0f) / 1000.0f; // volts
}

// --- Channel lock for STA / ESP-NOW on ch11 ---
static void lock_channel_sta_or_force() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);

  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CH, nullptr, true);
  const unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 1500) { delay(10); }

  if (WiFi.status() != WL_CONNECTED) {
    esp_wifi_disconnect();
    delay(10);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CH, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  }
}

// --- Build & send V3 batch with up to 10 readings ---
static void espnow_send_batch() {
  if (esp_now_init() != ESP_OK) return;
  esp_now_register_send_cb(onSend);

  static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_peer_info_t peer{}; memcpy(peer.peer_addr, BCAST, 6);
  peer.ifidx   = WIFI_IF_STA;
  peer.channel = 0;            // current primary
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  SensorPktV3 pkt{};
  pkt.node_uid = uid32();
  pkt.seq      = rtc_seq++;

  // Determine how many readings we have (<=10), in chronological order
  uint32_t n = (rtc_idx < 10) ? rtc_idx : 10;
  pkt.count   = (uint16_t)n;

  uint32_t start = (rtc_idx >= 10) ? (rtc_idx - 10) : 0;
  for (uint32_t i = 0; i < n; ++i) {
    const Reading &r = rtc_buf[(start + i) % 10];
    pkt.s[i].t_c   = r.t_c;
    pkt.s[i].p_hpa = r.p_hpa;
    pkt.s[i].b_v   = r.b_v;
    pkt.s[i].ts    = r.ts; // base will fallback to its own time if 0
  }

  // Compute actual payload size for n samples
  const size_t header = sizeof(pkt.node_uid) + sizeof(pkt.count) + sizeof(pkt.reserved) + sizeof(pkt.seq);
  const size_t per    = sizeof(pkt.s[0]);
  const size_t len    = header + n * per;

  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), len);
  delay(20);
}

static void radio_off_and_sleep(uint32_t seconds) {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  // advance wake counter
  rtc_wake_count = (rtc_wake_count + 1) % TX_EVERY;

  // battery first
  float vbat = read_battery_v();
  if (rtc_lowbat) {
    if (vbat < BAT_RESUME_V) { radio_off_and_sleep(SLEEP_SECONDS); return; }
    rtc_lowbat = false;
  }
  if (vbat < BAT_CUTOFF_V) { rtc_lowbat = true; radio_off_and_sleep(SLEEP_SECONDS); return; }

  // read sensor quickly
  float tC = NAN, pH = NAN;
  bmp_one_shot(tC, pH);

  // stash reading into RTC circular buffer
  Reading cur{ tC, pH, vbat, (uint32_t)(millis()/1000) }; // ts is relative; base can replace with unix if 0
  rtc_buf[rtc_idx % 10] = cur;
  rtc_idx++;

  // only transmit on every 10th wake
  if (rtc_wake_count == 0) {
    lock_channel_sta_or_force();
    espnow_send_batch();
  }

  radio_off_and_sleep(SLEEP_SECONDS);
}

void loop() {}
