#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
extern "C" {
  #include "esp_wifi.h"
}

// ===== Pines OLED =====
#define OLED_SDA 8
#define OLED_SCL 4

// ===== OLED =====
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===== ESP-NOW =====
uint8_t espMasterMac[6] = {0xC8,0x2E,0x18,0xAD,0x53,0xCC};
const bool USE_BROADCAST = false;
const int ESP_NOW_CHANNEL = 6;

// ===== Mensaje tipo Distancia =====
#pragma pack(push,1)
struct MsgDist {
  uint8_t kind;       // 3 según tu ESP principal
  float dist_cm;
  uint32_t seq;
};
#pragma pack(pop)

uint32_t seqCounter = 1;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

// ===== Datos del sensor =====
unsigned char data[4] = {};
float distance;

// ===== Intervalo de envío =====
const unsigned long SEND_INTERVAL_MS = 180000UL;
unsigned long lastSend = 0;

// ================= FUNCIONES =================
#if ESP_IDF_VERSION_MAJOR >= 5
void onSendCallback(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  lastSendStatus = status;
  if (info) {
    Serial.printf("Send callback: status=%d, tx_ctrl_present\n", (int)status);
  } else {
    Serial.printf("Send callback: status=%d\n", (int)status);
  }
}
#else
void onSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;
  char macbuf[32] = {0};
  if (mac_addr) snprintf(macbuf, sizeof(macbuf), "%02X:%02X:%02X:%02X:%02X:%02X",
    mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);
  else strcpy(macbuf, "??:??:??:??:??:??");
  Serial.printf("Send callback -> to=%s status=%d\n", macbuf, (int)status);
}
#endif

void sendDistance(float distance) {
  MsgDist msg;
  msg.kind = 3;
  msg.dist_cm = distance;
  msg.seq = seqCounter++;

  uint8_t destMac[6];
  if (USE_BROADCAST) {
    for (int i=0;i<6;i++) destMac[i] = 0xFF;
  } else {
    memcpy(destMac, espMasterMac, 6);
  }

  esp_err_t result = esp_now_send(destMac, (uint8_t *)&msg, sizeof(msg));
  if (result != ESP_OK) {
    Serial.printf("esp_now_send returned ERROR: 0x%08X\n", result);
  } else {
    Serial.printf("esp_now_send returned OK (tentative). Dist: %.2f cm seq=%u\n", distance, (unsigned)msg.seq);
  }
}

void drawOLED(float distance, bool sending) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[32];

  if (distance < 0) snprintf(buf,sizeof(buf),"Dist: --.- cm");
  else snprintf(buf,sizeof(buf),"Dist: %.1f cm", distance);
  u8g2.drawStr(2,20,buf);

  snprintf(buf,sizeof(buf),"ESP-NOW: %s", sending ? "OK" : "ERR");
  u8g2.drawStr(2,40,buf);

  u8g2.sendBuffer();
}

// ================= SETUP =================
void setup() {
  Serial0.begin(9600); // Sensor
  Serial.begin(115200);
  pinMode(8, OUTPUT);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(100);

  esp_err_t rc = esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (rc == ESP_OK) {
    Serial.printf("esp_wifi_set_channel -> OK (chan %d)\n", ESP_NOW_CHANNEL);
  } else {
    Serial.printf("esp_wifi_set_channel -> ERROR 0x%08X (chan %d)\n", rc, ESP_NOW_CHANNEL);
  }

  // Imprimir MAC local para verificar pares
  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
  } else {
    esp_now_register_send_cb(onSendCallback);

    if (!USE_BROADCAST) {
      esp_now_peer_info_t peer;
      memset(&peer, 0, sizeof(peer));
      memcpy(peer.peer_addr, espMasterMac, 6);
      peer.channel = ESP_NOW_CHANNEL; // importante: mismo canal
      peer.encrypt = false;
      if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add peer (esp_now_add_peer) -> intentar con broadcast si persiste");
      } else {
        Serial.println("Peer agregado OK");
      }
    } else {
      Serial.println("Usando BROADCAST (FF:FF:FF:FF:FF:FF) - no se agrega peer");
    }
  }
}

// ================= LOOP =================
void loop() {
  Serial0.println("");

  // ===== Lectura del sensor tal cual =====
  do {
    for (int i = 0; i < 4; i++) {
      data[i] = Serial0.read();
    }
  } while (Serial0.read() == 0xff);
  Serial0.flush();

  if (data[0] == 0xff) {
    int sum;
    sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3]) {
      distance = (data[1] << 8) + data[2];
      distance /= 10.0; // convertir a cm
      Serial.print("distance=");
      Serial.print(distance);
      Serial.println("cm");
    } else {
      Serial.println("ERROR");
      distance = -1;
    }
  } else {
    distance = -1;
  }

  // ===== Enviar cada 3s =====
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendDistance(distance);
  }

  // ===== Mostrar en OLED =====
  drawOLED(distance, lastSendStatus == ESP_NOW_SEND_SUCCESS);

  delay(2000); // pequeña pausa para estabilidad
}