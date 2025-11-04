#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
extern "C" {
  #include "esp_wifi.h"
}

// ===== Pines OLED (como en tu ejemplo) =====
#define OLED_SDA 21
#define OLED_SCL 22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===== ESP-NOW =====
uint8_t espMasterMac[6] = {0xC8,0x2E,0x18,0xAD,0x53,0xCC}; // Cambia si hace falta
const bool USE_BROADCAST = false;
const int ESP_NOW_CHANNEL = 6;

uint32_t seqCounter = 1;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

// ===== Mensaje de estados de nivel =====
#pragma pack(push,1)
struct MsgLevel {
  uint8_t kind;      // identifica el tipo (ej. 4)
  uint8_t states[4]; // 0/1 para cada sensor
  uint32_t seq;
};
#pragma pack(pop)

// ===== Pines de los sensores de nivel =====
const int sensorPins[4] = {15, 32, 25, 26}; // Pines ESP32
int sensorStates[4]; // Array para almacenar estados

// ===== Intervalo de envío =====
const unsigned long SEND_INTERVAL_MS = 190000UL;
unsigned long lastSend = 0;

// ================= CALLBACK ESP-NOW =================
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

// ================= FUNCIONES =================
void sendLevels() {
  MsgLevel msg;
  msg.kind = 4; // elige 4 para "nivel"
  for (int i = 0; i < 4; ++i) msg.states[i] = (sensorStates[i] == LOW) ? 0 : 1; // 0=bajo,1=alto
  msg.seq = seqCounter++;

  uint8_t destMac[6];
  if (USE_BROADCAST) {
    for (int i=0;i<6;i++) destMac[i] = 0xFF;
  } else {
    memcpy(destMac, espMasterMac, 6);
  }

  // Debug: imprimir payload y destino
  uint8_t *p = (uint8_t *)&msg;
  Serial.print("SEND payload hex: ");
  for (size_t i=0;i<sizeof(msg);i++) {
    if (p[i] < 0x10) Serial.print('0');
    Serial.print(p[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("Dest MAC: ");
  for (int i=0;i<6;i++) {
    if (i) Serial.print(':');
    if (destMac[i] < 0x10) Serial.print('0');
    Serial.print(destMac[i], HEX);
  }
  Serial.println();

  esp_err_t result = esp_now_send(destMac, (uint8_t *)&msg, sizeof(msg));
  if (result != ESP_OK) {
    Serial.printf("esp_now_send returned ERROR: 0x%08X\n", result);
  } else {
    Serial.printf("esp_now_send returned OK (tentative). seq=%u states=%u,%u,%u,%u\n",
                  (unsigned)msg.seq, msg.states[0], msg.states[1], msg.states[2], msg.states[3]);
  }
}

void drawOLED() {
  u8g2.clearBuffer();

  // Título pequeño
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2,10,"Niveles (Alto/Bajo):");

  // Mostrar estados en líneas separadas para legibilidad
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[32];
  const char* s0 = (sensorStates[0] == LOW) ? "Bajo" : "Alto";
  const char* s1 = (sensorStates[1] == LOW) ? "Bajo" : "Alto";
  const char* s2 = (sensorStates[2] == LOW) ? "Bajo" : "Alto";
  const char* s3 = (sensorStates[3] == LOW) ? "Bajo" : "Alto";

  // Columna izquierda S1 y S3, columna derecha S2 y S4
  snprintf(buf, sizeof(buf), "S1: %s", s0); u8g2.drawStr(2, 26, buf);
  snprintf(buf, sizeof(buf), "S2: %s", s1); u8g2.drawStr(74, 26, buf);
  snprintf(buf, sizeof(buf), "S3: %s", s2); u8g2.drawStr(2, 44, buf);
  snprintf(buf, sizeof(buf), "S4: %s", s3); u8g2.drawStr(74, 44, buf);

  // Estado ESP-NOW al pie
  const char* statusText = (lastSendStatus == ESP_NOW_SEND_SUCCESS) ? "OK" : "ERR";
  snprintf(buf, sizeof(buf), "ESP-NOW: %s", statusText);
  u8g2.drawStr(2, 62, buf);

  u8g2.sendBuffer();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Pines sensores
  for (int i = 0; i < 4; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
    sensorStates[i] = digitalRead(sensorPins[i]);
  }

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(100);

  esp_err_t rc = esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (rc == ESP_OK) {
    Serial.printf("esp_wifi_set_channel -> OK (chan %d)\n", ESP_NOW_CHANNEL);
  } else {
    Serial.printf("esp_wifi_set_channel -> ERROR 0x%08X (chan %d)\n", rc, ESP_NOW_CHANNEL);
  }

  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
  } else {
    esp_now_register_send_cb(onSendCallback);

    if (!USE_BROADCAST) {
      esp_now_peer_info_t peer;
      memset(&peer, 0, sizeof(peer));
      memcpy(peer.peer_addr, espMasterMac, 6);
      peer.channel = ESP_NOW_CHANNEL;
      peer.encrypt = false;
      if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Failed to add peer (esp_now_add_peer) -> intenta broadcast si persiste");
      } else {
        Serial.println("Peer agregado OK");
      }
    } else {
      Serial.println("Usando BROADCAST (FF:FF:FF:FF:FF:FF) - no se agrega peer");
    }
  }

  Serial.println("Lectura de sensores de nivel iniciada...");
  // Primer dibujado
  drawOLED();
  delay(5000);
}

// ================= LOOP =================
void loop() {
  // Leer los sensores
  for (int i = 0; i < 4; i++) {
    sensorStates[i] = digitalRead(sensorPins[i]);
  }

  // Mostrar en Serial
  Serial.print("Sensor 1 (pin 22): ");
  Serial.print(sensorStates[0] == LOW ? "Bajo" : "Alto");
  Serial.print(" | Sensor 2 (pin 32): ");
  Serial.print(sensorStates[1] == LOW ? "Bajo" : "Alto");
  Serial.print(" | Sensor 3 (pin 25): ");
  Serial.print(sensorStates[2] == LOW ? "Bajo" : "Alto");
  Serial.print(" | Sensor 4 (pin 26): ");
  Serial.println(sensorStates[3] == LOW ? "Bajo" : "Alto");

  // Enviar cada intervalo
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendLevels();
  }

  // Mostrar en OLED
  drawOLED();

  delay(1000); // pequeña pausa
}

