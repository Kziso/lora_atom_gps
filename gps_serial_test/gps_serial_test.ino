#include <TinyGPSPlus.h>

HardwareSerial SerialGPS(1);

static const int GPS_RX_PIN = 1;   // Grove white wire -> GPS TX
static const int GPS_TX_PIN = 2;   // Grove yellow wire -> GPS RX (optional)
static const uint32_t GPS_BAUD = 115200;

TinyGPSPlus gps;

void printGpsInfo() {
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
  Serial.printf("Stats: chars=%lu sentences_with_fix=%lu failed_checksum=%lu\n",
                static_cast<unsigned long>(gps.charsProcessed()),
                static_cast<unsigned long>(gps.sentencesWithFix()),
                static_cast<unsigned long>(gps.failedChecksum()));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    delay(10);
  }
  Serial.println("GPS serial test starting...");

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Waiting for NMEA sentences...\n");
}

void loop() {
  while (SerialGPS.available() > 0) {
    char c = static_cast<char>(SerialGPS.read());
    Serial.write(c);  // 生データをそのまま表示して NMEA/UBX を確認
    if (c == '\n') {
      Serial.flush();
    }
    gps.encode(c);
  }

  static uint32_t lastPrint = 0;
  const uint32_t now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    printGpsInfo();
  }
}
