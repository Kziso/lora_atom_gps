#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <cstring>
#include <cstdio>
#include "esp32_e220900t22s_jp_lib.h"

CLoRa lora;
LoRaConfigItem_t cfg;

static const char kTimeZoneSuffix[] = "+09:00";
static const uint32_t DATA_TIMEOUT_MS = 10000;
static const uint32_t ERROR_HOLD_MS = 3000;
static constexpr int LED_PIN = 35;
static Adafruit_NeoPixel statusLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

struct __attribute__((packed)) TelemetryPacket {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  int32_t latitude_e7;
  int32_t longitude_e7;
  int32_t altitude_cm;
  uint16_t hdop_x10;
  uint8_t satellites;
  uint8_t fix_flags;
};

enum class LedMode {
  Boot,
  WaitingData,
  DataTimeout,
  Active,
  Error
};

static LedMode g_currentLed = LedMode::Boot;
static uint32_t g_lastFrameMs = 0;
static uint32_t g_lastErrorMs = 0;

static void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

static void applyLedMode(LedMode mode) {
  if (mode == g_currentLed) {
    return;
  }
  g_currentLed = mode;
  switch (mode) {
    case LedMode::Boot:
      setLedColor(64, 0, 64);  // purple
      break;
    case LedMode::WaitingData:
      setLedColor(80, 40, 0);  // amber
      break;
    case LedMode::DataTimeout:
      setLedColor(0, 0, 80);   // blue
      break;
    case LedMode::Active:
      setLedColor(0, 80, 0);   // green
      break;
    case LedMode::Error:
      setLedColor(80, 0, 0);   // red
      break;
  }
}

static void updateLedState() {
  const uint32_t now = millis();
  if (now - g_lastErrorMs <= ERROR_HOLD_MS) {
    applyLedMode(LedMode::Error);
    return;
  }

  if (g_lastFrameMs == 0) {
    applyLedMode(LedMode::WaitingData);
    return;
  }

  if (now - g_lastFrameMs > DATA_TIMEOUT_MS) {
    applyLedMode(LedMode::DataTimeout);
  } else {
    applyLedMode(LedMode::Active);
  }
}

static void printCsvHeader() {
  Serial.println(F("TimestampJST,Latitude,Longitude,Altitude_m,HDOP,Sats,FixFlags,RSSI_dBm"));
}

static void emitCsvLine(const TelemetryPacket &pkt, int rssi) {
  char timestamp[32];
  if (pkt.year != 0) {
    snprintf(timestamp, sizeof(timestamp), "%04u-%02u-%02u %02u:%02u:%02u%s",
             pkt.year, pkt.month, pkt.day, pkt.hour, pkt.minute, pkt.second, kTimeZoneSuffix);
  } else {
    timestamp[0] = '\0';
  }

  const double lat = static_cast<double>(pkt.latitude_e7) / 1e7;
  const double lon = static_cast<double>(pkt.longitude_e7) / 1e7;
  const double alt_m = static_cast<double>(pkt.altitude_cm) / 100.0;
  const double hdop = static_cast<double>(pkt.hdop_x10) / 10.0;

  Serial.printf("%s,%.7f,%.7f,%.2f,%.1f,%u,0x%02X,%d\n",
                timestamp,
                lat,
                lon,
                alt_m,
                hdop,
                pkt.satellites,
                pkt.fix_flags,
                rssi);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  statusLed.begin();
  statusLed.setBrightness(64);
  statusLed.clear();
  statusLed.show();
  applyLedMode(LedMode::Boot);

  auto ret = lora.LoadConfigSetting(CONFIG_FILENAME, cfg);
  Serial.printf("LoadConfig=%d, own=%u, target=%u\n", ret, cfg.own_address, cfg.target_address);
  Serial.println(CONFIG_FILENAME);
  if (lora.InitLoRaModule(cfg) != 0) {
    Serial.println("Init NG");
  }
  lora.SwitchToNormalMode();

  printCsvHeader();
  applyLedMode(LedMode::WaitingData);
}

void loop() {
  RecvFrameE220900T22SJP_t rf;
  int ret = lora.receiveFrame(&rf);
  if (ret != 0) {
    g_lastErrorMs = millis();
    updateLedState();
    delay(5);
    return;
  }

  const int rawLength = rf.recv_data_len;
  const int expectedSize = static_cast<int>(sizeof(TelemetryPacket));

  if (rawLength != expectedSize && rawLength != expectedSize + 3) {
    Serial.printf("# Unexpected payload size=%d (expected %d or %d)\n",
                  rawLength, expectedSize, expectedSize + 3);
    g_lastErrorMs = millis();
    updateLedState();
    return;
  }

  const uint8_t *payload = rf.recv_data;
  if (rawLength == expectedSize + 3) {
    payload += 3;
  }

  TelemetryPacket pkt;
  memcpy(&pkt, payload, sizeof(pkt));
  g_lastFrameMs = millis();
  emitCsvLine(pkt, rf.rssi);

  updateLedState();
}
