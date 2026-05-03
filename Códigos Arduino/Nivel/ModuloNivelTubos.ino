#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_now.h>
extern "C" {
  #include "esp_wifi.h"
}

// ===== Pines OLED =====
#define OLED_SDA 21
#define OLED_SCL 22
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===== ESP-NOW =====
uint8_t espMasterMac[6] = {0xC8,0x2E,0x18,0xAD,0x53,0xCC}; // Cambia si hace falta
uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Dirección Broadcast
const int ESP_NOW_CHANNEL = 1; // Ajustado al canal 1 para coincidir con el maestro

uint32_t seqCounter = 1;
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

// ===== Mensaje de estados de nivel =====
#pragma pack(push,1)
struct MsgLevel {
  uint8_t kind;      // identifica el tipo (ej. 4)
  uint8_t states[7]; // 0/1 para cada sensor
  uint32_t seq;
};
#pragma pack(pop)

// ===== Pines de los sensores de nivel =====
const int sensorPins[7] = {13, 14, 25, 26, 27, 32, 33}; 
int sensorStates[7]; 

// ===== Intervalos de Tiempo =====
const unsigned long SEND_INTERVAL_MS = 3000UL;   // Envío ESP-NOW cada 3 seg
const unsigned long SCREEN_INTERVAL_MS = 15000UL; // Cambio de pantalla cada 15 seg

unsigned long lastSend = 0;
unsigned long lastScreenChange = 0;

// Variable para controlar qué pantalla mostrar
bool pantalla1Activa = true;

// ================= CALLBACK ESP-NOW =================
#if ESP_IDF_VERSION_MAJOR >= 5
void onSendCallback(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  lastSendStatus = status;
}
#else
void onSendCallback(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;
}
#endif

// ================= FUNCIONES =================
void sendLevels() {
  MsgLevel msg;
  msg.kind = 4; 
  for (int i = 0; i < 7; ++i) msg.states[i] = (sensorStates[i] == LOW) ? 0 : 1; 
  msg.seq = seqCounter++;

  // Intento de envío directo al Master
  esp_err_t result = esp_now_send(espMasterMac, (uint8_t *)&msg, sizeof(msg));
  
  if (result == ESP_OK) {
    Serial.printf("Envio a Master OK. seq=%u\n", (unsigned)msg.seq);
  } else {
    // Fallback: Si falla el envío directo, usar Broadcast
    Serial.println("Fallo envio directo -> enviando broadcast");
    esp_err_t bc = esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
    
    if (bc == ESP_OK) {
      Serial.println("Broadcast enviado");
    } else {
      Serial.println("Broadcast fallo");
    }
  }
}

void drawOLED() {
  u8g2.clearBuffer();
  
  // Usamos una fuente clara y un poco más alta
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[32];
  
  // Guardamos las etiquetas en un arreglo
  const char* s[7];
  for (int i = 0; i < 7; i++) {
    s[i] = (sensorStates[i] == LOW) ? "Bajo" : "Alto";
  }

  // --- LÓGICA DE INTERCAMBIO DE PANTALLAS ---
  if (pantalla1Activa) {
    // PANTALLA 1: Título y Sensores 1 al 4
    u8g2.drawStr(2, 10, "--- NIVELES (1/2) ---");
    
    snprintf(buf, sizeof(buf), "Sensor 1 (P13): %s", s[0]); u8g2.drawStr(2, 26, buf);
    snprintf(buf, sizeof(buf), "Sensor 2 (P14): %s", s[1]); u8g2.drawStr(2, 38, buf);
    snprintf(buf, sizeof(buf), "Sensor 3 (P25): %s", s[2]); u8g2.drawStr(2, 50, buf);
    snprintf(buf, sizeof(buf), "Sensor 4 (P26): %s", s[3]); u8g2.drawStr(2, 62, buf);

  } else {
    // PANTALLA 2: Título, Sensores 5 al 7 y Estado de Red
    u8g2.drawStr(2, 10, "--- NIVELES (2/2) ---");
    
    snprintf(buf, sizeof(buf), "Sensor 5 (P27): %s", s[4]); u8g2.drawStr(2, 26, buf);
    snprintf(buf, sizeof(buf), "Sensor 6 (P32): %s", s[5]); u8g2.drawStr(2, 38, buf);
    snprintf(buf, sizeof(buf), "Sensor 7 (P33): %s", s[6]); u8g2.drawStr(2, 50, buf);
    
    // Mostramos el estado del ESP-NOW en la última línea
    const char* statusText = (lastSendStatus == ESP_NOW_SEND_SUCCESS) ? "Conectado" : "No Conectado";
    snprintf(buf, sizeof(buf), "Red: %s", statusText);
    u8g2.drawStr(2, 62, buf);
  }

  u8g2.sendBuffer();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(100);

  // Pines sensores configurados
  for (int i = 0; i < 7; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
    sensorStates[i] = digitalRead(sensorPins[i]);
  }

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Desconectar antes de iniciar modo promiscuo
  delay(100);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
  } else {
    esp_now_register_send_cb(onSendCallback);

    // Registro del Peer Maestro
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, espMasterMac, 6);
    peer.channel = ESP_NOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    
    // Registro del Peer Broadcast (Fallback)
    esp_now_peer_info_t broadcastPeer;
    memset(&broadcastPeer, 0, sizeof(broadcastPeer));
    memcpy(broadcastPeer.peer_addr, broadcastAddress, 6);
    broadcastPeer.channel = ESP_NOW_CHANNEL;
    broadcastPeer.encrypt = false;
    esp_now_add_peer(&broadcastPeer);
  }

  Serial.println("Sistema iniciado.");
  drawOLED();
}

// ================= LOOP =================
void loop() {
  // 1. Leer los 7 sensores
  for (int i = 0; i < 7; i++) {
    sensorStates[i] = digitalRead(sensorPins[i]);
  }

  // 2. Control de tiempo para envío ESP-NOW (cada 3s)
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendLevels();
  }

  // 3. Control de tiempo para cambio de pantalla OLED (cada 15s)
  if (millis() - lastScreenChange >= SCREEN_INTERVAL_MS) {
    lastScreenChange = millis();
    pantalla1Activa = !pantalla1Activa; // Alterna entre true (pantalla 1) y false (pantalla 2)
  }

  // 4. Actualizar pantalla
  drawOLED();

  delay(50); // Pequeña pausa para estabilidad
}
