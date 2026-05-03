////////// LIBRERIAS ////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <HTTPClient.h>

/////////// PINES ///////////////////////////////////////////////////////////////////////////////////////////////
#define OLED_SCL 22
#define OLED_SDA 21
#define DHT_PIN 26
#define DHTTYPE DHT22

/////////// WIFI y FLASK ///////////////////////////////////////////////////////////////////////////////////////
const char *WIFI_SSID = "SOLOCUYES";
const char *WIFI_PASS = "guardianycoronel2712solocuyes";
const char *SERVER_BASE = "http://34.31.243.173:5000/";
const char *INGEST_TOKEN = "SocorroDelCarmen.20";

//////////// TIEMPOS ////////////////////////////////////////////////////////////////////////////////////////////
const uint32_t SEND_MS = 400000;
const uint32_t STATUS_TIMEOUT_MS = 300000;
const uint32_t UI_CYCLE_MS = 10000;

////////// OLED y DHT /////////////////////////////////////////////////////////////////////////////////////////
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DHT dht(DHT_PIN, DHTTYPE);

////////// ESTRUCTURAS ESP-NOW ////////////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
struct MsgPH { uint8_t kind; float ph; float tempC; uint32_t seq; };
struct MsgEC { uint8_t kind; float ec; float tempC; uint32_t seq; };
struct MsgDist { uint8_t kind; float dist_cm; uint32_t seq; };
struct MsgNivel { uint8_t kind; uint8_t n[7]; uint32_t seq; }; // Ajustado a 7 sensores
#pragma pack(pop)

////////// ESTADO GLOBAL ////////////////////////////////////////////////////////////////////////////////////////
float g_ph = NAN, g_ec = NAN, g_tph = NAN, g_tec = NAN;
float g_distance = NAN;
uint8_t g_nivel[7] = {0,0,0,0,0,0,0}; // Arreglo actualizado a 7 niveles
float g_temp_air = NAN, g_hum_air = NAN;
uint32_t lastSeqSeen[5] = {0};
uint32_t lastSeenKind[5] = {0};
bool kindSeen[5] = {false};

////////// UI /////////////////////////////////////////////////////////////////////////////////////////////////
enum UiMode { MODE_DATA, MODE_STATUS, MODE_ALL };
UiMode uiMode = MODE_DATA;
uint32_t lastUiCycle = 0;

////////// COLAS Y SEMAFOROS /////////////////////////////////////////////////////////////////////////////////
QueueHandle_t sensorQueue = nullptr;
SemaphoreHandle_t dataMutex = nullptr;

////////// ESTRUCTURAS AUX ////////////////////////////////////////////////////////////////////////////////////
struct SensorEvent {
  uint8_t kind;
  uint8_t buf[64];
  uint8_t len;
  uint8_t mac[6];
  int8_t rssi;
};

////////// FUNCIONES AUXILIARES /////////////////////////////////////////////////////////////////////////////
static inline float computeTempAgua_locked() {
  if (!isnan(g_tph) && !isnan(g_tec)) return (g_tph + g_tec) * 0.5f;
  if (!isnan(g_tph)) return g_tph;
  if (!isnan(g_tec)) return g_tec;
  return NAN;
}

static unsigned long _last_wifi_attempt = 0;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 5000UL;

static const char *wifiStatusToStr(wl_status_t s) {
  switch (s) {
    case WL_NO_SHIELD: return "NO_SHIELD";
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

void wifiEnsureConnectedNonBlocking() {
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - _last_wifi_attempt < WIFI_RETRY_INTERVAL_MS) return;

  _last_wifi_attempt = now;

  Serial.printf("WiFi: estado %s\n", wifiStatusToStr(st));

  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (millis() - t0 < 8000) {
    if (WiFi.status() == WL_CONNECTED) {

      Serial.printf("WiFi conectado IP=%s\n", WiFi.localIP().toString().c_str());

      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);

      return;
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

////////// ESP-NOW RX /////////////////////////////////////////////////////////////////////////////////////////
void onNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (!data || len <= 0) return;

  SensorEvent ev;
  ev.kind = data[0];
  ev.len = min(len, (int)sizeof(ev.buf));
  memcpy(ev.buf, data, ev.len);

  if (recv_info && recv_info->src_addr)
    memcpy(ev.mac, recv_info->src_addr, 6);
  else
    memset(ev.mac, 0, 6);

  ev.rssi = (recv_info && recv_info->rx_ctrl) ? recv_info->rx_ctrl->rssi : 0;

  if (sensorQueue)
    xQueueSendFromISR(sensorQueue, &ev, nullptr);

  Serial.printf("ESP-NOW recibido kind=%d RSSI=%d\n", ev.kind, ev.rssi);
}

////////// SENSOR TASK ////////////////////////////////////////////////////////////////////////////////////////
void sensorTask(void *pvParameters) {
  SensorEvent ev;
  for (;;) {
    if (xQueueReceive(sensorQueue, &ev, portMAX_DELAY) == pdTRUE) {

      uint32_t now = millis();

      xSemaphoreTake(dataMutex, portMAX_DELAY);

      if (ev.kind < 5) {
        lastSeenKind[ev.kind] = now;
        kindSeen[ev.kind] = true;
      }

      switch (ev.kind) {

        case 1: {
          MsgPH m;
          memcpy(&m, ev.buf, sizeof(MsgPH));
          if (m.seq > lastSeqSeen[1]) {
            g_ph = m.ph;
            g_tph = m.tempC;
            lastSeqSeen[1] = m.seq;
            Serial.printf("ESP1 -> pH: %.2f Temp: %.2f\n", g_ph, g_tph);
          }
        } break;

        case 2: {
          MsgEC m;
          memcpy(&m, ev.buf, sizeof(MsgEC));
          if (m.seq > lastSeqSeen[2]) {
            g_ec = m.ec;
            g_tec = m.tempC;
            lastSeqSeen[2] = m.seq;
            Serial.printf("ESP2 -> EC: %.2f Temp: %.2f\n", g_ec, g_tec);
          }
        } break;

        case 3: {
          MsgDist m;
          memcpy(&m, ev.buf, sizeof(MsgDist));
          if (m.seq > lastSeqSeen[3]) {
            g_distance = m.dist_cm;
            lastSeqSeen[3] = m.seq;
            Serial.printf("ESP3 -> Distancia: %.2f cm\n", g_distance);
          }
        } break;

        case 4: {
          MsgNivel m;
          memcpy(&m, ev.buf, sizeof(MsgNivel));
          if (m.seq > lastSeqSeen[4]) {
            for (int i=0;i<7;i++) g_nivel[i] = m.n[i]; // Lee los 7 sensores
            lastSeqSeen[4] = m.seq;
            Serial.printf("ESP4 -> Niveles: %d %d %d %d %d %d %d\n",
                          g_nivel[0], g_nivel[1], g_nivel[2], g_nivel[3],
                          g_nivel[4], g_nivel[5], g_nivel[6]);
          }
        } break;
      }

      xSemaphoreGive(dataMutex);
    }
  }
}

////////// UI DRAW ////////////////////////////////////////////////////////////////////////////////////////////
void drawDataScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  char buf[32];

  if (!isnan(g_temp_air))
    snprintf(buf,sizeof(buf),"Temp: %.1f C",g_temp_air);
  else
    snprintf(buf,sizeof(buf),"Temp: -- C");

  u8g2.drawStr(2,20,buf);

  if (!isnan(g_hum_air))
    snprintf(buf,sizeof(buf),"Hum: %.1f %%",g_hum_air);
  else
    snprintf(buf,sizeof(buf),"Hum: -- %%");

  u8g2.drawStr(2,40,buf);

  u8g2.sendBuffer();
}

void drawStatusScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  char buf[32];

  u8g2.drawStr(2, 10, "Estado ESP-NOW:");

  for (int k = 1; k <= 4; k++) {
    bool ok = kindSeen[k] && (millis() - lastSeenKind[k] <= STATUS_TIMEOUT_MS);
    snprintf(buf, sizeof(buf), "ESP%d: %s", k, ok ? "OK" : "OFF");
    u8g2.drawStr(2, 10 + k * 12, buf);
  }

  u8g2.sendBuffer();
}

void drawAllScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  char buf[48];

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  float ph = g_ph;
  float ec = g_ec;
  float twa = computeTempAgua_locked();
  float dist = g_distance;
  uint8_t n0 = g_nivel[0], n1 = g_nivel[1], n2 = g_nivel[2], n3 = g_nivel[3], n4 = g_nivel[4], n5 = g_nivel[5], n6 = g_nivel[6];

  xSemaphoreGive(dataMutex);

  snprintf(buf, sizeof(buf), "pH: %.2f", ph);
  u8g2.drawStr(2, 12, buf);

  snprintf(buf, sizeof(buf), "EC: %.2f", ec);
  u8g2.drawStr(70, 12, buf);

  snprintf(buf, sizeof(buf), "T Agua: %.1f", twa);
  u8g2.drawStr(2, 28, buf);

  snprintf(buf, sizeof(buf), "Dist: %.1f", dist);
  u8g2.drawStr(2, 44, buf);

  snprintf(buf, sizeof(buf), "Niv: %d%d%d%d%d%d%d", n0, n1, n2, n3, n4, n5, n6);
  u8g2.drawStr(2, 60, buf);

  u8g2.sendBuffer();
}

void uiTask(void *pvParameters) {
  for (;;) {
    if (uiMode == MODE_DATA) drawDataScreen();
    else if (uiMode == MODE_STATUS) drawStatusScreen();
    else drawAllScreen();

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

////////// INPUT TASK ////////////////////////////////////////////////////////////////////////////////////////
void inputTask(void *pvParameters) {

  uint32_t lastDht = 0;
  uint32_t lastSend = 0;

  for (;;) {
    uint32_t now = millis();

    // DHT
    if (now - lastDht >= 2000) {
      lastDht = now;

      float h = dht.readHumidity();
      float t = dht.readTemperature();

      xSemaphoreTake(dataMutex, portMAX_DELAY);

      if (!isnan(h) && !isnan(t)) {
        g_temp_air = t;
        g_hum_air = h;
      }

      xSemaphoreGive(dataMutex);
    }

    // ciclo pantallas
    if (now - lastUiCycle >= UI_CYCLE_MS) {
      lastUiCycle = now;
      uiMode = (UiMode)((uiMode + 1) % 3);
    }

    // envio datos
    if (now - lastSend >= SEND_MS) {
      lastSend = now;

      xSemaphoreTake(dataMutex, portMAX_DELAY);

      float ph = g_ph;
      float ec = g_ec;
      float tempAir = g_temp_air;
      float humAir = g_hum_air;
      float dist = g_distance;
      float twa = computeTempAgua_locked();
      uint8_t niv[7];
      for(int i=0; i<7; i++) niv[i] = g_nivel[i];

      xSemaphoreGive(dataMutex);

      char body[350];
      int len = 0;
      bool anyField = false;

      len += snprintf(body + len, sizeof(body) - len, "{");

      if (!isnan(tempAir)) {
        len += snprintf(body + len, sizeof(body) - len, "\"temp_air\":%.2f,", tempAir);
        anyField = true;
      }

      if (!isnan(humAir)) {
        len += snprintf(body + len, sizeof(body) - len, "\"hum_air\":%.2f,", humAir);
        anyField = true;
      }

      if (!isnan(ph)) {
        len += snprintf(body + len, sizeof(body) - len, "\"ph\":%.2f,", ph);
        anyField = true;
      }

      if (!isnan(ec)) {
        len += snprintf(body + len, sizeof(body) - len, "\"ec\":%.2f,", ec);
        anyField = true;
      }

      if (!isnan(twa)) {
        len += snprintf(body + len, sizeof(body) - len, "\"temp_water\":%.2f,", twa);
        anyField = true;
      }

      if (!isnan(dist)) {
        len += snprintf(body + len, sizeof(body) - len, "\"distance_cm\":%.2f,", dist);
        anyField = true;
      }
      
      // Se agregan los 7 sensores de nivel al JSON (como un arreglo para que sea fácil en tu backend)
      len += snprintf(body + len, sizeof(body) - len, "\"nivel\":[%d,%d,%d,%d,%d,%d,%d],", 
                niv[0], niv[1], niv[2], niv[3], niv[4], niv[5], niv[6]);
      anyField = true;

      // quitar última coma
      if (len > 1 && body[len - 1] == ',') {
        body[len - 1] = '\0';
        len--;
      }

      len += snprintf(body + len, sizeof(body) - len, "}");

      // enviar solo si hay datos
      if (anyField) {

        esp_now_deinit();

        wifiOn();

        if (WiFi.status() == WL_CONNECTED) {

          HTTPClient http;
          http.begin(String(SERVER_BASE) + "api/ingest");
          http.addHeader("Content-Type", "application/json");

          if (strlen(INGEST_TOKEN))
            http.addHeader("X-INGEST-TOKEN", INGEST_TOKEN);

          int code = http.POST(body);

          Serial.printf("POST code: %d\n", code);

          http.end();
        }

        wifiOff();

        int retries = 0;

        while (esp_now_init() != ESP_OK && retries < 3) {
          Serial.println("Reintentando ESP-NOW...");
          vTaskDelay(pdMS_TO_TICKS(500));
          retries++;
        }

        if (retries < 3) {
          esp_now_register_recv_cb(onNowRecv);

          esp_wifi_set_promiscuous(true);
          esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
          esp_wifi_set_promiscuous(false);

          Serial.println("ESP-NOW reiniciado");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

////////// WIFI TASK //////////////////////////////////////////////////////////////////////////////////////////
void wifiOn() {
  Serial.println("Encendiendo WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 5000) {
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi fallo");
  }
}

void wifiOff() {
  Serial.println("Apagando WiFi...");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  vTaskDelay(pdMS_TO_TICKS(200));
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

////////// SETUP ///////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  Serial.begin(115200);
  delay(50);

  Wire.begin(OLED_SDA,OLED_SCL);
  u8g2.begin();
  dht.begin();

  sensorQueue=xQueueCreate(32,sizeof(SensorEvent));
  dataMutex=xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if(esp_now_init()!=ESP_OK)
    Serial.println("ESP-NOW error");
  else{
    esp_now_register_recv_cb(onNowRecv);
    Serial.println("ESP-NOW iniciado");
  }

  xTaskCreatePinnedToCore(sensorTask,"SensorTask",6*1024,NULL,3,NULL,1);
  xTaskCreatePinnedToCore(uiTask,"UITask",6*1024,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(inputTask,"InputTask",6*1024,NULL,2,NULL,1);

  lastUiCycle=millis();

  Serial.println("Sistema iniciado correctamente");
}

////////// LOOP ///////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}