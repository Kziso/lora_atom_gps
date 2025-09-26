#include "esp32_e220900t22s_jp_lib.h"
CLoRa lora;
LoRaConfigItem_t cfg;

void setup() {
  Serial.begin(115200);
  auto ret = lora.LoadConfigSetting(CONFIG_FILENAME, cfg);   // 無ければデフォルト
  Serial.printf("LoadConfig=%d, own=%u, target=%u\n", ret, cfg.own_address, cfg.target_address);
  Serial.println(CONFIG_FILENAME);
  if (lora.InitLoRaModule(cfg) != 0) Serial.println("Init NG");
  lora.SwitchToNormalMode();
}

void loop() {
  static uint32_t t=0;
  if (millis()-t>2000) {
    t=millis();
    const char* s = "PING\n";
    if (lora.SendFrame(cfg, (uint8_t*)s, strlen(s))==0) Serial.println("sent");
  }
}
