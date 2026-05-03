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

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);



// ===== ESP-NOW =====
uint8_t espMasterMac[6] = {0xC8,0x2E,0x18,0xAD,0x53,0xCC};
uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const int ESP_NOW_CHANNEL = 1;

// ===== Mensaje =====
#pragma pack(push,1)
struct MsgDist {
  uint8_t kind;
  float dist_cm;
  uint32_t seq;
};
#pragma pack(pop)

uint32_t seqCounter = 1;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
bool sendSuccess = false;

// ===== Sensor =====
unsigned char data[4] = {};
float distance;
void sendDistance(float distance);

// ===== Timing =====
const unsigned long SEND_INTERVAL_MS = 180000UL;
unsigned long lastSend = 0;


// ===== CALLBACK =====
#if ESP_IDF_VERSION_MAJOR >= 5

void onSendCallback(const wifi_tx_info_t *info, esp_now_send_status_t status) {

  lastSendStatus = status;

  if (status == ESP_NOW_SEND_SUCCESS)
    sendSuccess = true;
  else
    sendSuccess = false;

  Serial.printf("Send status: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

#else

void onSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {

  lastSendStatus = status;

  if (status == ESP_NOW_SEND_SUCCESS)
    sendSuccess = true;
  else
    sendSuccess = false;

  Serial.printf("Send status: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

#endif

// ===== OLED =====
void drawOLED(float distance, String state) {

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  char buf[32];

  snprintf(buf, sizeof(buf), "Dist: %.1f cm", distance);
  u8g2.drawStr(2, 20, buf);

  u8g2.drawStr(2, 40, state.c_str());

  u8g2.sendBuffer();
}

// ===== ENVIO =====
void sendDistance(float distance) {

  MsgDist msg;
  msg.kind = 3;
  msg.dist_cm = distance;
  msg.seq = seqCounter++;

  esp_err_t result = esp_now_send(espMasterMac, (uint8_t *)&msg, sizeof(msg));

  if (result == ESP_OK) {

    Serial.println("ESP-NOW enviado a peer");

    lastSendStatus = ESP_NOW_SEND_SUCCESS;
  } 
  else {

    Serial.println("Fallo peer -> enviando broadcast");

    esp_err_t bc = esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));

    if (bc == ESP_OK) {

      Serial.println("Broadcast enviado");
      lastSendStatus = ESP_NOW_SEND_SUCCESS;
    }
    else {

      Serial.println("Broadcast fallo");
      lastSendStatus = ESP_NOW_SEND_FAIL;
    }
  }
}

// ===== SETUP =====
void setup() {
  Serial0.begin(9600);  // NO TOCAR
  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();

  // ===== WIFI =====

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(200);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  delay(50);

  Serial.println("WiFi iniciado (modo STA)");

  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // ===== ESP-NOW =====
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT ERROR");
    return;
  }

  esp_now_register_send_cb(onSendCallback);

  // ===== PEER =====
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, espMasterMac, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR ADD PEER");
  } else {
    Serial.println("PEER OK");
  }

  // peer broadcast
  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, broadcastAddress, 6);
  broadcastPeer.channel = ESP_NOW_CHANNEL;
  broadcastPeer.encrypt = false;
  esp_now_add_peer(&broadcastPeer);

  delay(1000);
}

// ===== LOOP =====
void loop() {

  Serial0.println("");

  // ===== SENSOR (NO TOCADO) =====
  do {
    for (int i = 0; i < 4; i++) {
      data[i] = Serial0.read();
    }
  } while (Serial0.read() == 0xff);
  Serial0.flush();

  if (data[0] == 0xff) {
    int sum = (data[0] + data[1] + data[2]) & 0xFF;
    if (sum == data[3]) {
      distance = (data[1] << 8) + data[2];
      distance /= 10.0;
      Serial.printf("distance=%.2f cm\n", distance);
    } else {
      Serial.println("ERROR");
      distance = -1;
    }
  } else {
    distance = -1;
  }

  // ===== ENVIO =====
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendDistance(distance);
  }

  String state = "ESP-NOW: WAIT";

  if (lastSendStatus == ESP_NOW_SEND_SUCCESS)
    state = "ESP-NOW: OK";

  drawOLED(distance, state);

  delay(2000);
}