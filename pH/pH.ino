#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_ADS1X15.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"

#include <Preferences.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

// ----------------- Ajustes hardware -----------------
#define I2C_SDA 8
#define I2C_SCL 9
#define ONEWIRE_PIN 5
#define OLED_SDA I2C_SDA
#define OLED_SCL I2C_SCL

#define BTN_ENTER_CAL 7
#define BTN_EXIT_CAL 10
#define BTN_SAVE_CAL 20

#define ADS_CHANNEL 0  // A0 corresponde al canal 0

const unsigned long SEND_INTERVAL_MS = 140000UL;
const unsigned long ADS_READ_MS = 1000UL;
const unsigned long DEBOUNCE_MS = 200UL;  // antirrebote botones

uint8_t peerMAC[6] = { 0xC8, 0x2E, 0x18, 0xAD, 0x53, 0xCC };  // Receptor

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds(&oneWire);

Adafruit_ADS1115 ads;
DFRobot_ESP_PH_WITH_ADC phSensor;

Preferences prefs;

float calib_a = 3.5f;
float calib_b = 0.0f;
bool calib_valid = false;

bool cal_waiting_for_second = false;
float cal_v_point1 = NAN;
float cal_ph_point1 = NAN;

unsigned long lastAdsRead = 0;
float lastVoltage = NAN;
float lastPH = NAN;
float lastTemp = NAN;

uint32_t seqSend = 1;
bool inCalibration = false;

bool espNowOk = false;

// Estado de último envío ESP-NOW
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

// Estructura mensaje
#pragma pack(push, 1)
struct MsgPH {
  uint8_t kind;
  float ph;
  float tempC;
  uint32_t seq;
};
#pragma pack(pop)

// ---------- Forward decls ----------
void drawMainScreen(bool ph_ok, bool temp_ok);
void drawCalibrationScreen(const char* line1, const char* line2, const char* line3);
float readADSVoltage();
float applyCalibrationAndTempCompensation(float volt, float tempC);
bool computeTwoPointCalibration(float v1, float p1, float v2, float p2, float& out_a, float& out_b);
void saveCalibrationToPrefs();
void loadCalibrationFromPrefs();

// ----------------- esp-now send callback -----------------
#if ESP_IDF_VERSION_MAJOR >= 5
void onSendCallback(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  lastSendStatus = status;  // guardar estado
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send: SUCCESS");
  else Serial.printf("ESP-NOW send: FAIL status=%d\n", (int)status);
}
#else
void onSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;  // guardar estado
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send: SUCCESS");
  else Serial.printf("ESP-NOW send: FAIL status=%d\n", (int)status);
}
#endif

// ----------------- Helpers -----------------
void saveCalibrationToPrefs() {
  prefs.putFloat("cal_a", calib_a);
  prefs.putFloat("cal_b", calib_b);
  prefs.putBool("cal_ok", true);
  Serial.printf("Saved calibration a=%.6f b=%.6f\n", calib_a, calib_b);
}
void loadCalibrationFromPrefs() {
  if (prefs.getBool("cal_ok", false)) {
    calib_a = prefs.getFloat("cal_a", 3.5f);
    calib_b = prefs.getFloat("cal_b", 0.0f);
    calib_valid = true;
    Serial.printf("Loaded calibration a=%.6f b=%.6f\n", calib_a, calib_b);
  } else {
    calib_valid = false;
    Serial.println("No calibration saved - using defaults");
  }
}

float readADSVoltage() {
  int16_t raw = ads.readADC_SingleEnded(ADS_CHANNEL);
  float lsb_mV = 0.125F;  // mV por bit para GAIN_ONE
  float mv = raw * lsb_mV;
  return mv / 1000.0F;  // resultado en voltios
}

float applyCalibrationAndTempCompensation(float volt, float tempC) {
  float ph_lin = calib_a * volt + calib_b;
  float ph_corr = ph_lin + (25.0f - tempC) * 0.03f;
  return ph_corr;
}

// ----- Leer y promediar varias muestras del ADS1115 (devuelve V) -----
float readADSVoltageAverage(int samples = 8, unsigned int delayMsBetween = 5) {
  long acc = 0;
  int valid = 0;
  for (int i = 0; i < samples; ++i) {
    int16_t raw = ads.readADC_SingleEnded(ADS_CHANNEL);
    acc += raw;
    valid++;
    delay(delayMsBetween);
  }
  if (valid == 0) return NAN;
  float avgRaw = (float)acc / (float)valid;
  float lsb_mV = 0.125F;  // mV por bit para GAIN_ONE
  float mv = avgRaw * lsb_mV;
  return mv / 1000.0F;  // voltios
}

// ----- Leer temperatura actual (retorna 25.0 si desconectado) -----
float readTempC() {
  ds.requestTemperatures();
  float t = ds.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C || isnan(t)) return 25.0f;
  return t;
}


bool computeTwoPointCalibration(float v1, float p1, float v2, float p2, float& out_a, float& out_b) {
  if (isnan(v1) || isnan(v2) || fabsf(v1 - v2) < 1e-9) return false;
  out_a = (p2 - p1) / (v2 - v1);
  out_b = p1 - out_a * v1;
  return true;
}

// ----------------- OLED Screens -----------------
void drawMainScreen(bool ph_ok, bool temp_ok) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  char buf[64];

  // 1. Estado ESP-NOW (según callback)
  const char* statusMsg;
  if (lastSendStatus == ESP_NOW_SEND_SUCCESS) statusMsg = "ok";
  else statusMsg = "error";
  u8g2.drawStr(2, 12, "ESP-NOW:");
  u8g2.drawStr(70, 12, statusMsg);

  // 2. pH
  if (ph_ok && !isnan(lastPH)) snprintf(buf, sizeof(buf), "pH: %.3f", lastPH);
  else snprintf(buf, sizeof(buf), "pH: --.- (nan)");
  u8g2.drawStr(2, 28, buf);

  // 3. Temperatura
  if (temp_ok && !isnan(lastTemp)) snprintf(buf, sizeof(buf), "T: %.2f C", lastTemp);
  else snprintf(buf, sizeof(buf), "T: 25.00 C (def)");
  u8g2.drawStr(2, 44, buf);

  // 4. Ayuda botones
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(2, 62, "BTN: ENTER / SAVE / EXIT");

  u8g2.sendBuffer();
}

void drawCalibrationScreen(const char* line1, const char* line2, const char* line3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 12, "=== CALIBRACION ===");
  u8g2.drawStr(2, 28, line1);
  u8g2.drawStr(2, 44, line2);
  u8g2.drawStr(2, 60, line3);
  u8g2.sendBuffer();
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();

  pinMode(BTN_ENTER_CAL, INPUT_PULLUP);
  pinMode(BTN_EXIT_CAL, INPUT_PULLUP);
  pinMode(BTN_SAVE_CAL, INPUT_PULLUP);

  ds.begin();

  ads.setGain(GAIN_ONE);
  if (!ads.begin()) Serial.println("Failed to initialize ADS1115");
  else Serial.println("ADS1115 OK");

  phSensor.begin();

  prefs.begin("ph_cal", false);
  loadCalibrationFromPrefs();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    espNowOk = false;
  } else {
    Serial.println("ESP-NOW init OK");
    espNowOk = true;
    esp_now_register_send_cb(onSendCallback);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, peerMAC, 6);
    peerInfo.channel = 6;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) Serial.println("Failed to add peer");
    else Serial.println("Peer added");
  }

  drawMainScreen(false, false);
  delay(10000);
}

// ----------------- Loop -----------------
void loop() {
  static unsigned long lastSend = 0;
  static unsigned long lastAdsReadLocal = 0;
  static unsigned long lastBtnEnter = 0, lastBtnSave = 0, lastBtnExit = 0;

  unsigned long now = millis();

  // ===== Lectura sensores =====
  if (now - lastAdsReadLocal >= ADS_READ_MS) {
    lastAdsReadLocal = now;

    ds.requestTemperatures();
    float t = ds.getTempCByIndex(0);
    lastTemp = (t == DEVICE_DISCONNECTED_C) ? 25.0f : t;

    float volt = readADSVoltage();
    lastVoltage = volt;

    bool ph_ok = false;
    if (!isnan(volt)) {
      if (calib_valid) {
        lastPH = applyCalibrationAndTempCompensation(volt, lastTemp);
        ph_ok = true;
      } else {
        float volt_mV = volt * 1000.0f;
        float fromLib = phSensor.readPH(volt_mV, lastTemp);
        if (!isnan(fromLib) && isfinite(fromLib)) {
          lastPH = fromLib;
          ph_ok = true;
        }
      }
    }

    if (inCalibration) {
      if (!cal_waiting_for_second)
        drawCalibrationScreen("Probe in pH 7.0", "Press SAVE to capture", "");
      else
        drawCalibrationScreen("Probe in pH 4.0", "Press SAVE to capture", "Then compute & save");
    } else {
      drawMainScreen(ph_ok, !isnan(lastTemp));
    }
  }

  // ===== Botones =====
  if (digitalRead(BTN_ENTER_CAL) == LOW && now - lastBtnEnter > DEBOUNCE_MS) {
    lastBtnEnter = now;
    if (!inCalibration) {
      inCalibration = true;
      cal_waiting_for_second = false;
      // Forzar lectura inicial para mostrar dato estable en pantalla
      lastVoltage = readADSVoltageAverage(12, 6);
      lastTemp = readTempC();
      drawCalibrationScreen("Probe in pH 7.0", "Press SAVE to capture", "");
      Serial.println("Calibration started: put probe in pH 7.0");
    }
  }


  if (digitalRead(BTN_SAVE_CAL) == LOW && now - lastBtnSave > DEBOUNCE_MS) {
    lastBtnSave = now;
    if (inCalibration) {
      // Leer promediado en el momento de pulsar SAVE
      float currentVolt = readADSVoltageAverage(12, 6);  // 12 muestras, 6ms entre ellas
      float currentTemp = readTempC();

      if (isnan(currentVolt)) {
        Serial.println("SAVE: Voltaje NAN, intenta de nuevo");
        drawCalibrationScreen("Lectura invalida", "Voltaje NAN, reintenta", "");
        delay(700);
        return;
      }

      if (!cal_waiting_for_second) {
        // Captura punto 1 (pH 7)
        cal_v_point1 = currentVolt;
        cal_ph_point1 = 7.0f;
        cal_waiting_for_second = true;
        Serial.printf("Captured point1: V=%.6f V  T=%.2f C  pH=7.0\n", cal_v_point1, currentTemp);
        char l1[32];
        snprintf(l1, sizeof(l1), "PUNTO1 V: %.4f V", cal_v_point1);
        char l2[32];
        snprintf(l2, sizeof(l2), "Temp: %.2f C", currentTemp);
        drawCalibrationScreen(l1, l2, "Ahora pH 4.0 y SAVE");
      } else {
        // Captura punto 2 (pH 4) y calcular
        float v2 = currentVolt;
        float p2 = 4.0f;
        Serial.printf("Captured point2: V=%.6f V  T=%.2f C  pH=4.0\n", v2, currentTemp);

        // Comprobaciones: no NAN y voltajes distintos
        if (isnan(cal_v_point1) || isnan(v2)) {
          Serial.println("Calibration failed: NAN values");
          drawCalibrationScreen("Error:", "Lectura NAN", "Abortando");
          inCalibration = false;
          cal_waiting_for_second = false;
          delay(800);
        } else if (fabsf(cal_v_point1 - v2) < 1e-4f) {
          // Si las lecturas son prácticamente iguales, fallamos - muy ruidoso o sonda mal
          Serial.println("Calibration failed: voltages too similar");
          drawCalibrationScreen("Error:", "Voltajes muy similares", "Reinicia y reintenta");
          inCalibration = false;
          cal_waiting_for_second = false;
          delay(1200);
        } else {
          float a, b;
          if (computeTwoPointCalibration(cal_v_point1, cal_ph_point1, v2, p2, a, b)) {
            calib_a = a;
            calib_b = b;
            calib_valid = true;
            saveCalibrationToPrefs();
            Serial.printf("Calibration done: a=%.6f b=%.6f\n", calib_a, calib_b);

            char l1[32];
            snprintf(l1, sizeof(l1), "CAL OK a: %.4f", calib_a);
            char l2[32];
            snprintf(l2, sizeof(l2), "b: %.4f", calib_b);
            drawCalibrationScreen("Calibracion exitosa", l1, l2);
            delay(1200);
          } else {
            Serial.println("Calibration computeTwoPointCalibration returned false");
            drawCalibrationScreen("Calculo fallido", "verifica lecturas", "");
            delay(1200);
          }
          inCalibration = false;
          cal_waiting_for_second = false;
        }
      }
    }
  }


  if (digitalRead(BTN_EXIT_CAL) == LOW && now - lastBtnExit > DEBOUNCE_MS) {
    lastBtnExit = now;
    if (inCalibration) {
      inCalibration = false;
      Serial.println("Calibration aborted");
    }
  }

  // ===== Envío ESP-NOW =====
  if (!inCalibration && (millis() - lastSend >= SEND_INTERVAL_MS)) {
    lastSend = millis();
    MsgPH msg;
    msg.kind = 1;
    msg.ph = lastPH;
    msg.tempC = lastTemp;
    msg.seq = seqSend++;

    esp_err_t res = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    if (res != ESP_OK) Serial.printf("esp_now_send failed: %d\n", (int)res);
    else Serial.printf("Sent ph=%.3f T=%.2f seq=%u\n", msg.ph, msg.tempC, msg.seq);
  }

  delay(10);
}
