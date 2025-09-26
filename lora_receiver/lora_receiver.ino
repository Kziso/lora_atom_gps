#include <Arduino.h>
#include "esp32_e220900t22s_jp_lib.h"

CLoRa lora;
LoRaConfigItem_t cfg;

void print_hex(const uint8_t* p, int n) {
  for (int i = 0; i < n; i++) {
    if (p[i] < 16) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(i + 1 < n ? ' ' : '\n');
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  auto ret = lora.LoadConfigSetting(CONFIG_FILENAME, cfg);   // 無ければデフォルト
  Serial.printf("LoadConfig=%d, own=%u, target=%u\n", ret, cfg.own_address, cfg.target_address);
  Serial.println(CONFIG_FILENAME);
  if (lora.InitLoRaModule(cfg) != 0) Serial.println("Init NG");
  // 受信（通常）モードへ
  lora.SwitchToNormalMode();

  Serial.println("[INFO] E220 receiver ready.");
}

void loop() {
  RecvFrameE220900T22SJP_t rf;  // ライブラリ側の受信フレーム構造体


  int ret = lora.receiveFrame(&rf);
  if (ret == 0) {
    // 文字列として表示（終端保証のため一時的に'\0'を付加）
    // ライブラリがサイズ情報を持っている前提（例: rf.recv_data_len）
    int n = rf.recv_data_len;             // ※ライブラリの実装により名称が異なる場合は合わせてください
    if (n > 0) {
      // 安全に表示：最大長に注意（ライブラリ側の最大長に合わせてください）
      static char line[256];
      int m = min(n, (int)sizeof(line)-1);
      memcpy(line, rf.recv_data, m);
      line[m] = '\0';

      Serial.println(F("recv data:"));
      Serial.println(line);

      Serial.println(F("hex dump:"));
      print_hex(rf.recv_data, n);

      // RSSI（ライブラリで末尾RSSIバイトを有効にしている場合）
      Serial.print(F("RSSI: "));
      Serial.print(rf.rssi);
      Serial.println(F(" dBm"));

      // 送信元アドレスやチャネルが取れるなら参考表示（実装に合わせてコメント解除）
      // Serial.printf("FROM: 0x%02X%02X, CH:%d\n", rf.addr_h, rf.addr_l, rf.channel);

      Serial.println();
    }
  }

  // ポーリング間隔（必要に応じて調整）
  delay(5);
}
