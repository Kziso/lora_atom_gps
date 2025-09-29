#include <Adafruit_NeoPixel.h>
#include <TinyGPSPlus.h>
#include <cstring>
#include "esp32_e220900t22s_jp_lib.h"

CLoRa lora;
LoRaConfigItem_t cfg;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);  // Serial1はLoRaモジュールが使用

struct TelemetryPacket;
struct DateTimeFields;

static constexpr int LED_PIN = 35;
static Adafruit_NeoPixel statusLed(1, LED_PIN, NEO_GRB + NEO_KHZ800);

static const int GPS_RX_PIN = 1;    // Grove 白線 -> GPS TX
static const int GPS_TX_PIN = 2;    // Grove 黄線 -> GPS RX (未使用可)
static const uint32_t GPS_BAUD = 115200;
static const uint32_t SEND_INTERVAL_MS = 30000;
static const int TIMEZONE_OFFSET_SECONDS = 9 * 3600;  // JST (UTC+9)
static const uint32_t GPS_DATA_TIMEOUT_MS = 10000;
static const uint32_t ERROR_HOLD_MS = 3000;

enum class LedMode {
  Boot,
  WaitingGps,
  GpsTimeout,
  GpsFix,
  SendError
};

static LedMode g_currentLed = LedMode::Boot;
static uint32_t g_lastGpsDataMs = 0;
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
    case LedMode::WaitingGps:
      setLedColor(80, 40, 0);  // amber
      break;
    case LedMode::GpsTimeout:
      setLedColor(0, 0, 80);   // blue
      break;
    case LedMode::GpsFix:
      setLedColor(0, 80, 0);   // green
      break;
    case LedMode::SendError:
      setLedColor(80, 0, 0);   // red
      break;
  }
}

static void updateLedState(bool hasFix) {
  const uint32_t now = millis();
  if (now - g_lastErrorMs <= ERROR_HOLD_MS) {
    applyLedMode(LedMode::SendError);
    return;
  }

  if (g_lastGpsDataMs == 0 || now - g_lastGpsDataMs > GPS_DATA_TIMEOUT_MS) {
    applyLedMode(LedMode::GpsTimeout);
    return;
  }

  if (hasFix) {
    applyLedMode(LedMode::GpsFix);
  } else {
    applyLedMode(LedMode::WaitingGps);
  }
}

struct __attribute__((packed)) TelemetryPacket {
  uint16_t year;       // JST 年 (無効時は0)
  uint8_t month;       // 1-12
  uint8_t day;         // 1-31
  uint8_t hour;        // 0-23 (JST)
  uint8_t minute;      // 0-59 (JST)
  uint8_t second;      // 0-59 (JST)
  int32_t latitude_e7; // 1e-7 度単位の緯度
  int32_t longitude_e7;// 1e-7 度単位の経度
  int32_t altitude_cm; // cm 単位の高度
  uint16_t hdop_x10;   // HDOP ×10
  uint8_t satellites;  // 衛星数 (無効時は0)
  uint8_t fix_flags;   // 各値の有効フラグ
};

enum FixFlags : uint8_t {
  FIX_LOCATION = 0x01,
  FIX_ALTITUDE = 0x02,
  FIX_HDOP     = 0x04,
  FIX_TIME     = 0x08
};

struct DateTimeFields {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

static bool isLeapYear(int year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

static int daysInMonth(int year, int month) {
  static const int days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  if (month == 2) {
    return days[1] + (isLeapYear(year) ? 1 : 0);
  }
  return days[month - 1];
}

static void addDays(DateTimeFields &dt, int deltaDays) {
  while (deltaDays > 0) {
    const int dim = daysInMonth(dt.year, dt.month);
    if (dt.day < dim) {
      ++dt.day;
    } else {
      dt.day = 1;
      if (dt.month < 12) {
        ++dt.month;
      } else {
        dt.month = 1;
        ++dt.year;
      }
    }
    --deltaDays;
  }

  while (deltaDays < 0) {
    if (dt.day > 1) {
      --dt.day;
    } else {
      if (dt.month > 1) {
        --dt.month;
      } else {
        dt.month = 12;
        --dt.year;
      }
      dt.day = daysInMonth(dt.year, dt.month);
    }
    ++deltaDays;
  }
}

static void applyTimezoneOffset(DateTimeFields &dt, int offsetSeconds) {
  int totalSeconds = dt.hour * 3600 + dt.minute * 60 + dt.second + offsetSeconds;
  int dayDelta = 0;

  while (totalSeconds >= 86400) {
    totalSeconds -= 86400;
    ++dayDelta;
  }

  while (totalSeconds < 0) {
    totalSeconds += 86400;
    --dayDelta;
  }

  dt.hour = totalSeconds / 3600;
  dt.minute = (totalSeconds % 3600) / 60;
  dt.second = totalSeconds % 60;

  if (dayDelta != 0) {
    addDays(dt, dayDelta);
  }
}

static bool buildTelemetryPacket(TelemetryPacket &pkt) {
  memset(&pkt, 0, sizeof(pkt));

  if (gps.location.isValid()) {
    pkt.latitude_e7 = static_cast<int32_t>(gps.location.lat() * 1e7);
    pkt.longitude_e7 = static_cast<int32_t>(gps.location.lng() * 1e7);
    pkt.fix_flags |= FIX_LOCATION;
  }

  if (gps.altitude.isValid()) {
    pkt.altitude_cm = static_cast<int32_t>(gps.altitude.meters() * 100.0);
    pkt.fix_flags |= FIX_ALTITUDE;
  }

  if (gps.hdop.isValid()) {
    pkt.hdop_x10 = static_cast<uint16_t>(gps.hdop.hdop() * 10.0);
    pkt.fix_flags |= FIX_HDOP;
  }

  if (gps.time.isValid() && gps.date.isValid()) {
    DateTimeFields dt = {
      gps.date.year(),
      static_cast<int>(gps.date.month()),
      static_cast<int>(gps.date.day()),
      static_cast<int>(gps.time.hour()),
      static_cast<int>(gps.time.minute()),
      static_cast<int>(gps.time.second())
    };
    applyTimezoneOffset(dt, TIMEZONE_OFFSET_SECONDS);

    pkt.year = static_cast<uint16_t>(dt.year);
    pkt.month = static_cast<uint8_t>(dt.month);
    pkt.day = static_cast<uint8_t>(dt.day);
    pkt.hour = static_cast<uint8_t>(dt.hour);
    pkt.minute = static_cast<uint8_t>(dt.minute);
    pkt.second = static_cast<uint8_t>(dt.second);
    pkt.fix_flags |= FIX_TIME;
  }

  if (gps.satellites.isValid()) {
    pkt.satellites = gps.satellites.value();
  }

  return (pkt.fix_flags & FIX_LOCATION) != 0;
}

static void printGpsDebug() {
  if (gps.location.isValid()) {
    Serial.printf("Lat: %.6f, Lon: %.6f\n", gps.location.lat(), gps.location.lng());
  } else {
    Serial.println("Lat/Lon: ---");
  }

  if (gps.altitude.isValid()) {
    Serial.printf("Alt: %.1f m\n", gps.altitude.meters());
  } else {
    Serial.println("Alt: ---");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("UTC: %04u-%02u-%02u %02u:%02u:%02u\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.println("UTC: ---");
  }

  if (gps.hdop.isValid()) {
    Serial.printf("HDOP: %.1f\n", gps.hdop.hdop());
  } else {
    Serial.println("HDOP: ---");
  }

  Serial.printf("Satellites: %u\n", gps.satellites.isValid() ? gps.satellites.value() : 0);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  statusLed.begin();
  statusLed.setBrightness(64);
  statusLed.clear();
  statusLed.show();
  applyLedMode(LedMode::Boot);

  auto ret = lora.LoadConfigSetting(CONFIG_FILENAME, cfg);   // 無ければデフォルト
  Serial.printf("LoadConfig=%d, own=%u, target=%u\n", ret, cfg.own_address, cfg.target_address);
  Serial.println(CONFIG_FILENAME);
  if (lora.InitLoRaModule(cfg) != 0) Serial.println("Init NG");
  lora.SwitchToNormalMode();

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS start");
  applyLedMode(LedMode::WaitingGps);
}

void loop() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      g_lastGpsDataMs = millis();
    }
  }

  static uint32_t lastDebugPrint = 0;
  const uint32_t now = millis();
  if (now - lastDebugPrint >= 1000) {
    lastDebugPrint = now;
    printGpsDebug();
  }

  static uint32_t lastSend = 0;
  if (now - lastSend < SEND_INTERVAL_MS) {
    updateLedState(gps.location.isValid());
    return;
  }
  lastSend = now;

  TelemetryPacket packet;
  const bool hasFix = buildTelemetryPacket(packet);

  uint8_t *payload = reinterpret_cast<uint8_t *>(&packet);
  const auto result = lora.SendFrame(cfg, payload, sizeof(packet));

  if (result == 0) {
    Serial.printf("LoRa sent (%u bytes), hasFix=%d, flags=0x%02X\n",
                  static_cast<unsigned>(sizeof(packet)), hasFix, packet.fix_flags);
  } else {
    Serial.printf("LoRa send error=%d\n", result);
    g_lastErrorMs = now;
  }

  updateLedState(hasFix);
}
