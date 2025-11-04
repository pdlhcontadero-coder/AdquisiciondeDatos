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
#define BTN_1 25
#define BTN_2 27
#define BTN_3 32

/////////// WIFI y FLASK ///////////////////////////////////////////////////////////////////////////////////////
const char *WIFI_SSID = "SOLOCUYES";
const char *WIFI_PASS = "guardianycoronel2712solocuyes";
const char *SERVER_BASE = "http://34.133.70.186:5000";
const char *INGEST_TOKEN = "SocorroDelCarmen.20";

//////////// TIEMPOS ////////////////////////////////////////////////////////////////////////////////////////////
const uint32_t SEND_MS = 400000;
const uint32_t STATUS_TIMEOUT_MS = 300000;
const uint32_t BTN_DEBOUNCE_MS = 50;
const uint32_t UI_CYCLE_MS = 5000;

////////// OLED y DHT /////////////////////////////////////////////////////////////////////////////////////////
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
DHT dht(DHT_PIN, DHTTYPE);

////////// ESTRUCTURAS ESP-NOW ////////////////////////////////////////////////////////////////////////////////
#pragma pack(push, 1)
struct MsgPH {
  uint8_t kind;
  float ph;
  float tempC;
  uint32_t seq;
};
struct MsgEC {
  uint8_t kind;
  float ec;
  float tempC;
  uint32_t seq;
};
struct MsgDist {
  uint8_t kind;
  float dist_cm;
  uint32_t seq;
};
struct MsgNivel {
  uint8_t kind;
  uint8_t n[4];
  uint32_t seq;
};
#pragma pack(pop)

////////// ESTADO GLOBAL ////////////////////////////////////////////////////////////////////////////////////////
float g_ph = NAN, g_ec = NAN, g_tph = NAN, g_tec = NAN;
float g_distance = NAN;
uint8_t g_nivel[4] = { 0, 0, 0, 0 };
float g_temp_air = NAN, g_hum_air = NAN;
uint32_t lastSeqSeen[5] = { 0 };
uint32_t lastSeenKind[5] = { 0 };
bool kindSeen[5] = { false };

////////// UI /////////////////////////////////////////////////////////////////////////////////////////////////
enum UiMode { MODE_DATA,
              MODE_STATUS,
              MODE_ALL };
UiMode uiMode = MODE_DATA;
bool autoCycle = false;
uint32_t lastUiCycle = 0;

////////// COLAS Y SEMAFOROS /////////////////////////////////////////////////////////////////////////////////
QueueHandle_t sensorQueue = nullptr;
QueueHandle_t httpQueue = nullptr;
SemaphoreHandle_t dataMutex = nullptr;
SemaphoreHandle_t oledMutex = nullptr;

////////// ESTRUCTURAS AUX ////////////////////////////////////////////////////////////////////////////////////
struct SensorEvent {
  uint8_t kind;
  uint8_t buf[64];
  uint8_t len;
  uint8_t mac[6];
  int8_t rssi;
};
struct OutboundMsg {
  char payload[384];
};
struct Btn {
  uint8_t pin;
  bool lastStable;
  bool lastRaw;
  uint32_t lastChangeMs;
};
Btn btns[3];

////////// FUNCIONES AUXILIARES /////////////////////////////////////////////////////////////////////////////
static inline float computeTempAgua_locked() {
  if (!isnan(g_tph) && !isnan(g_tec)) return (g_tph + g_tec) * 0.5f;
  if (!isnan(g_tph)) return g_tph;
  if (!isnan(g_tec)) return g_tec;
  return NAN;
}

// --- Control de reconexión WiFi con backoff (no spam de WiFi.begin) ---
static unsigned long _last_wifi_attempt = 0;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 5000UL;  // intentar cada 5s

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
  if (now - _last_wifi_attempt < WIFI_RETRY_INTERVAL_MS) {
    // no intentamos aún, esperamos el intervalo
    return;
  }
  _last_wifi_attempt = now;

  Serial.printf("WiFi: estado actual %d (%s). Intentando conectar a '%s'...\n",
                (int)st, wifiStatusToStr(st), WIFI_SSID);
  // Intento de reconexión no bloqueante: primero intenta reconectar automáticamente
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Espera corta y no bloqueante (polite) para comprobar si conecta rápido
  unsigned long t0 = millis();
  const unsigned long wait_ms = 8000UL;  // esperar hasta 8s por este intento
  while (millis() - t0 < wait_ms) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi: conectado! IP=%s\n", WiFi.localIP().toString().c_str());
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(250));  // cede a otras tareas
  }

  Serial.printf("WiFi: intento expirado (status=%s). Reintentaremos en %lu ms\n",
                wifiStatusToStr(WiFi.status()), WIFI_RETRY_INTERVAL_MS);
}


bool enqueueJsonToHttpQueue(const char *json) {
  if (!httpQueue) return false;
  OutboundMsg msg;
  strncpy(msg.payload, json, sizeof(msg.payload) - 1);
  return xQueueSend(httpQueue, &msg, 0) == pdTRUE;
}

////////// ESP-NOW RX /////////////////////////////////////////////////////////////////////////////////////////
void onNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (!data || len <= 0) return;

  SensorEvent ev;
  ev.kind = data[0];
  ev.len = min(len, (int)sizeof(ev.buf));
  memcpy(ev.buf, data, ev.len);

  if (recv_info && recv_info->src_addr) memcpy(ev.mac, recv_info->src_addr, 6);
  else memset(ev.mac, 0, 6);

  ev.rssi = (recv_info && recv_info->rx_ctrl) ? recv_info->rx_ctrl->rssi : 0;

  if (sensorQueue) xQueueSendFromISR(sensorQueue, &ev, nullptr);
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
        case 1:
          if (ev.len >= sizeof(MsgPH)) {
            MsgPH m;
            memcpy(&m, ev.buf, sizeof(MsgPH));
            if (m.seq > lastSeqSeen[1] && m.ph >= 0 && m.ph <= 14 && m.tempC > -20 && m.tempC < 80) {
              g_ph = m.ph;
              g_tph = m.tempC;
              lastSeqSeen[1] = m.seq;
            }
          }
          break;
        case 2:
          if (ev.len >= sizeof(MsgEC)) {
            MsgEC m;
            memcpy(&m, ev.buf, sizeof(MsgEC));
            if (m.seq > lastSeqSeen[2] && m.ec >= 0 && m.ec < 50000 && m.tempC > -20 && m.tempC < 80) {
              g_ec = m.ec;
              g_tec = m.tempC;
              lastSeqSeen[2] = m.seq;
            }
          }
          break;
        case 3:
          if (ev.len >= sizeof(MsgDist)) {
            MsgDist m;
            memcpy(&m, ev.buf, sizeof(MsgDist));
            if (m.seq > lastSeqSeen[3] && m.dist_cm >= 0 && m.dist_cm < 10000) g_distance = m.dist_cm;
            lastSeqSeen[3] = m.seq;
          }
          break;
        case 4:
          if (ev.len >= sizeof(MsgNivel)) {
            MsgNivel m;
            memcpy(&m, ev.buf, sizeof(MsgNivel));
            if (m.seq > lastSeqSeen[4]) {
              bool ok = true;
              for (int i = 0; i < 4; i++)
                if (m.n[i] > 1) ok = false;
              if (ok) {
                for (int i = 0; i < 4; i++) g_nivel[i] = m.n[i];
                lastSeqSeen[4] = m.seq;
              }
            }
          }
          break;
      }
      xSemaphoreGive(dataMutex);
    }
  }
}

////////// HTTP TASK //////////////////////////////////////////////////////////////////////////////////////////
void httpTask(void *pvParameters) {
  for (;;) {
    OutboundMsg msg;
    if (xQueueReceive(httpQueue, &msg, pdMS_TO_TICKS(2000)) == pdTRUE) {
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi no conectado, descarta msg");
        continue;
      }
      bool posted = false;
      int attempts = 0;
      while (!posted && attempts < 3) {
        attempts++;
        HTTPClient http;
        http.begin(String(SERVER_BASE) + "/api/ingest");
        http.addHeader("Content-Type", "application/json");
        if (strlen(INGEST_TOKEN)) http.addHeader("X-INGEST-TOKEN", INGEST_TOKEN);
        int code = http.POST(msg.payload);
        if (code >= 200 && code < 300) {
          posted = true;
          Serial.printf("POST ok %d\n", code);
        } else {
          Serial.printf("POST fallo %d attempt %d\n", code, attempts);
          vTaskDelay(pdMS_TO_TICKS(200 * attempts));
        }
        http.end();
      }
      if (!posted) Serial.println("No se pudo enviar JSON tras reintentos");
    } else vTaskDelay(pdMS_TO_TICKS(50));
  }
}

////////// UI DRAW ////////////////////////////////////////////////////////////////////////////////////////////
void drawDataScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[32];

  // Temp aire
  if (!isnan(g_temp_air)) snprintf(buf, sizeof(buf), "Temp: %.1f C", g_temp_air);
  else snprintf(buf, sizeof(buf), "Temp: -- C");
  u8g2.drawStr(2, 20, buf);

  // Humedad
  if (!isnan(g_hum_air)) snprintf(buf, sizeof(buf), "Hum: %.1f %%", g_hum_air);
  else snprintf(buf, sizeof(buf), "Hum: -- %%");
  u8g2.drawStr(2, 40, buf);

  u8g2.sendBuffer();
}

void drawAllScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  char buf[48];

  // lee datos protegidos por mutex
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  float ph = g_ph;
  float ec = g_ec;
  float twa = computeTempAgua_locked();
  float dist = g_distance;
  uint8_t n0 = g_nivel[0], n1 = g_nivel[1], n2 = g_nivel[2], n3 = g_nivel[3];
  float tempAir = g_temp_air;
  float humAir = g_hum_air;
  uint32_t now = millis();
  xSemaphoreGive(dataMutex);

  // aplicamos regla de 5 minutos
  if (!kindSeen[1] || now - lastSeenKind[1] > STATUS_TIMEOUT_MS) ph = NAN;
  if (!kindSeen[2] || now - lastSeenKind[2] > STATUS_TIMEOUT_MS) ec = NAN;
  if (!kindSeen[3] || now - lastSeenKind[3] > STATUS_TIMEOUT_MS) dist = NAN;
  if ((!kindSeen[1] || now - lastSeenKind[1] > STATUS_TIMEOUT_MS) &&
    (!kindSeen[2] || now - lastSeenKind[2] > STATUS_TIMEOUT_MS)) {
    twa = NAN;
  }
  if (!kindSeen[4] || now - lastSeenKind[4] > STATUS_TIMEOUT_MS) n0=n1=n2=n3=255; // 255 = sin dato
  if (!kindSeen[0] || now - lastSeenKind[0] > STATUS_TIMEOUT_MS) tempAir = NAN; 
  if (!kindSeen[0] || now - lastSeenKind[0] > STATUS_TIMEOUT_MS) humAir = NAN; 

  // fila 1: pH y EC
  if (!isnan(ph)) snprintf(buf, sizeof(buf), "pH: %.2f", ph);
  else snprintf(buf, sizeof(buf), "pH: --");
  u8g2.drawStr(2, 12, buf);

  if (!isnan(ec)) snprintf(buf, sizeof(buf), "EC: %.2f", ec);
  else snprintf(buf, sizeof(buf), "EC: --");
  u8g2.drawStr(74, 12, buf);

  // fila 2: Temp agua
  if (!isnan(twa)) snprintf(buf, sizeof(buf), "T Agua: %.1f C", twa);
  else snprintf(buf, sizeof(buf), "T Agua: -- C");
  u8g2.drawStr(2, 26, buf);

  // fila 3: Distancia
  if (!isnan(dist)) snprintf(buf, sizeof(buf), "Dist: %.1f cm", dist);
  else snprintf(buf, sizeof(buf), "Dist: -- cm");
  u8g2.drawStr(2, 40, buf);

  // fila 4: Niveles
  if (n0 != 255) snprintf(buf, sizeof(buf), "Niv: %d %d %d %d", n0, n1, n2, n3);
  else snprintf(buf, sizeof(buf), "Niv: -- -- -- --");
  u8g2.drawStr(2, 54, buf);

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


void uiTask(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(oledMutex, pdMS_TO_TICKS(200))) {
      if (uiMode == MODE_DATA) drawDataScreen();
      else if (uiMode == MODE_STATUS) drawStatusScreen();
      else drawAllScreen();
      xSemaphoreGive(oledMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

////////// INPUT TASK ////////////////////////////////////////////////////////////////////////////////////////
void setupButtons() {
  uint8_t pins[3] = { BTN_1, BTN_2, BTN_3 };
  for (int i = 0; i < 3; i++) {
    pinMode(pins[i], INPUT_PULLUP);
    bool st = digitalRead(pins[i]) == LOW;
    btns[i] = { pins[i], st, st, 0 };
  }
}

void handleButtonsTask(uint32_t now) {
  for (int i = 0; i < 3; i++) {
    bool raw = digitalRead(btns[i].pin) == LOW;
    if (raw != btns[i].lastRaw) {
      btns[i].lastRaw = raw;
      btns[i].lastChangeMs = now;
    } else if (now - btns[i].lastChangeMs >= BTN_DEBOUNCE_MS && raw != btns[i].lastStable) {
      btns[i].lastStable = raw;
      if (raw) {  // PRESIONADO
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        if (i == 0) {
          if (uiMode == MODE_DATA) uiMode = MODE_STATUS;
          else if (uiMode == MODE_STATUS) uiMode = MODE_ALL;
          autoCycle = false;
        } else if (i == 1) {
          if (uiMode == MODE_ALL) uiMode = MODE_STATUS;
          else if (uiMode == MODE_STATUS) uiMode = MODE_DATA;
          autoCycle = false;
        } else if (i == 2) {
          autoCycle = !autoCycle;
          lastUiCycle = now;
          Serial.printf("autoCycle %s\n", autoCycle ? "ON" : "OFF");
        }
        xSemaphoreGive(dataMutex);
      }
    }
  }
}

void inputTask(void *pvParameters) {
  setupButtons();
  uint32_t lastDht = 0, lastSend = 0;
  for (;;) {
    uint32_t now = millis();

    // Leer DHT cada 2s
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

    // Manejo de botones (con debounce)
    handleButtonsTask(now);

    // Auto cycle UI
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (autoCycle && now - lastUiCycle >= UI_CYCLE_MS) {
      lastUiCycle = now;
      uiMode = (UiMode)((uiMode + 1) % 3);
    }
    xSemaphoreGive(dataMutex);

    // Envío periódico de datos
    if (now - lastSend >= SEND_MS) {
      lastSend = now;

      // Leer valores compartidos UNA sola vez
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      float ph = g_ph;
      float ec = g_ec;
      float tempAir = g_temp_air;
      float humAir = g_hum_air;
      float dist = g_distance;
      float twa = computeTempAgua_locked();
      uint8_t n0 = g_nivel[0], n1 = g_nivel[1], n2 = g_nivel[2], n3 = g_nivel[3];
      xSemaphoreGive(dataMutex);

      // decidir si cada dato es "válido/reciente"
      bool have_tempAir = !isnan(tempAir);
      bool have_humAir = !isnan(humAir);

      bool have_ph = kindSeen[1] && (now - lastSeenKind[1] <= STATUS_TIMEOUT_MS) && !isnan(ph);
      bool have_ec = kindSeen[2] && (now - lastSeenKind[2] <= STATUS_TIMEOUT_MS) && !isnan(ec);
      bool have_dist = kindSeen[3] && (now - lastSeenKind[3] <= STATUS_TIMEOUT_MS) && !isnan(dist);

      // temp_water válida si hay temp de pH o EC reciente
      bool have_twa = (!isnan(twa)) && ((kindSeen[1] && (now - lastSeenKind[1] <= STATUS_TIMEOUT_MS)) || (kindSeen[2] && (now - lastSeenKind[2] <= STATUS_TIMEOUT_MS)));

      bool have_nivel = kindSeen[4] && (now - lastSeenKind[4] <= STATUS_TIMEOUT_MS);

      // --- Construcción dinámica del JSON: sólo incluimos campos válidos ---
      // Usamos un buffer y snprintf con offset para ir concatenando de forma segura.
      char body[384];
      int len = 0;
      bool anyField = false;

      len += snprintf(body + len, sizeof(body) - len, "{");

      if (have_tempAir) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"temp_air\":%.2f", tempAir);
        anyField = true;
      }
      if (have_humAir) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"hum_air\":%.2f", humAir);
        anyField = true;
      }
      if (have_ph) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"ph\":%.2f", ph);
        anyField = true;
      }
      if (have_ec) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"ec\":%.2f", ec);
        anyField = true;
      }
      if (have_twa) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"temp_water\":%.2f", twa);
        anyField = true;
      }
      if (have_dist) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"distance_cm\":%.2f", dist);
        anyField = true;
      }

      // Niveles: si hay datos recientes incluimos array, si no, lo omitimos
      if (have_nivel) {
        if (anyField) len += snprintf(body + len, sizeof(body) - len, ",");
        len += snprintf(body + len, sizeof(body) - len, "\"nivel\":[%d,%d,%d,%d]", n0, n1, n2, n3);
        anyField = true;
      }

      // Cerramos objeto JSON
      len += snprintf(body + len, sizeof(body) - len, "}");

      // Si no hay ningún campo válido, no enviamos nada
      if (!anyField) {
        Serial.println("No hay datos válidos para enviar — salto envío");
      } else {
        // DEBUG: descomenta la siguiente línea si quieres ver exactamente el JSON en Serial
        // Serial.println(body);
        if (!enqueueJsonToHttpQueue(body)) Serial.println("httpQueue llena, JSON descartado");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


////////// WIFI TASK //////////////////////////////////////////////////////////////////////////////////////////
void wifiTask(void *pvParameters) {
  for (;;) {
    wifiEnsureConnectedNonBlocking();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

////////// SETUP ///////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  delay(50);
  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  dht.begin();

  sensorQueue = xQueueCreate(32, sizeof(SensorEvent));
  httpQueue = xQueueCreate(12, sizeof(OutboundMsg));
  dataMutex = xSemaphoreCreateMutex();
  oledMutex = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  wifiEnsureConnectedNonBlocking();
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) Serial.println("ESP-NOW init error");
  else esp_now_register_recv_cb(onNowRecv);

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 6 * 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(httpTask, "HTTPTask", 8 * 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(uiTask, "UITask", 6 * 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(inputTask, "InputTask", 6 * 1024, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 4 * 1024, NULL, 2, NULL, 1);

  lastUiCycle = millis();
}

////////// LOOP VACIO /////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // loop vacío: todas las tareas corren en FreeRTOS
  vTaskDelay(pdMS_TO_TICKS(1000));
}
