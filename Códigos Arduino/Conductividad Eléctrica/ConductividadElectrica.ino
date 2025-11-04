#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_ADS1X15.h>
#include "DFRobot_ESP_EC.h"
#include <Preferences.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ----------------- Ajustes hardware -----------------
#define I2C_SDA 8
#define I2C_SCL 9

#define ONEWIRE_PIN 5
#define OLED_SDA I2C_SDA
#define OLED_SCL I2C_SCL

// Botones (INPUT_PULLUP)
#define BTN_ENTER_CAL 7
#define BTN_SAVE_CAL 10
#define BTN_EXIT_CAL 20

#define ADS_CHANNEL 0

// Timings
const unsigned long DEBOUNCE_MS = 200UL;

// Peer MAC (receptor) -- ajusta si cambia
uint8_t peerMAC[6] = { 0xC8, 0x2E, 0x18, 0xAD, 0x53, 0xCC };

// Mensajes kind IDs
const uint8_t KIND_EC = 2;  // EC + temp

// ---------- Periféricos ----------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds(&oneWire);

Adafruit_ADS1115 ads;
DFRobot_ESP_EC ecSensor;

Preferences prefs;

// Estado / variables
volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;
bool peerAdded = false;
bool useBroadcastFallback = false;

float lastTemp = NAN;
bool haveECreading = false;
// lastEC en µS/cm
float lastEC = NAN;
float lastVoltage = NAN;

uint32_t seqSend = 1;

// Calibration state
bool inCalibration = false;
bool cal_captured = false;
float cal_voltage_point = NAN;
// 1.413 mS/cm == 1413 µS/cm
float cal_ec_value = 1.413f;
bool cal_saved = false;

// Preferences keys
const char* PREF_NAMESPACE = "ec_cal";
const char* PREF_CAL_OK = "cal_ok";
const char* PREF_LAST_CAL_V = "cal_v";
const char* PREF_LAST_CAL_EC = "cal_ec";

// Mensaje (emisor -> receptor)
#pragma pack(push, 1)
struct MsgEC {
  uint8_t kind;  // 2 = EC+temp
  float ec;      // µS/cm (or -1 if none)
  float tempC;
  uint32_t seq;
};
#pragma pack(pop)

// ---------- Detección unidad librería EC ----------
bool ecLibExpects_mV = false;   // si true, pasar mV (volts*1000) a readEC/calibration
bool ecUnitDetected = false;    // detector ejecutado

// ---------- Forward decls ----------
void drawMainScreen();
void drawCalScreen(const char* l1, const char* l2, const char* l3);
float readADSVoltage_mv_like_example();
float readADSVoltageAvg(int samples = 8, unsigned long delayMs = 20);
void loadCalibrationPrefs();
void saveCalibrationPrefs();
bool ensurePeerAdded();
bool sendWithRetries(const uint8_t* dest, const uint8_t* data, size_t len);
void sendECMessage(float ec, float temp, uint8_t kind);
bool detectEcLibUnit(float volts, float temp);

// ---------- ESP-NOW callback ----------
#if ESP_IDF_VERSION_MAJOR >= 5
void onSendCallback(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  lastSendStatus = status;
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send callback: SUCCESS");
  else Serial.printf("ESP-NOW send callback: FAIL status=%d\n", (int)status);
}
#else
void onSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status) {
  lastSendStatus = status;
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send callback: SUCCESS");
  else Serial.printf("ESP-NOW send callback: FAIL status=%d\n", (int)status);
}
#endif

// ---------- Helpers ----------
float readADSVoltage_mv_like_example() {
  int16_t raw = ads.readADC_SingleEnded(ADS_CHANNEL);
  float mv = raw * 0.125f;  // LSB mV for GAIN_ONE
  return mv / 1000.0f;      // devuelve V
}

// PROMEDIO ADS (devuelve V)
float readADSVoltageAvg(int samples, unsigned long delayMs) {
  double sum = 0.0;
  for (int i = 0; i < samples; ++i) {
    int16_t raw = ads.readADC_SingleEnded(ADS_CHANNEL);
    float mv = raw * 0.125f;  // LSB mV for GAIN_ONE
    sum += (mv / 1000.0f);    // pasar a V
    delay(delayMs);
  }
  return (float)(sum / samples);
}

// Detección heurística de si la librería espera V o mV.
// Devuelve true si parece esperar mV.
bool detectEcLibUnit(float volts, float temp) {
  // Llamada normal (V)
  float ec_mS_normal = ecSensor.readEC(volts, temp);
  // Llamada con mV
  float ec_mS_mV = ecSensor.readEC(volts * 1000.0f, temp);

  Serial.printf("detectEcLibUnit: normal-> %.6f mS/cm, with mV-> %.6f mS/cm\n", ec_mS_normal, ec_mS_mV);

  bool normal_ok = isfinite(ec_mS_normal) && (ec_mS_normal > 0.05f);
  bool mv_ok     = isfinite(ec_mS_mV)     && (ec_mS_mV > 0.05f);

  if (!normal_ok && mv_ok) {
    Serial.println("detectEcLibUnit: decided -> library expects mV");
    return true;
  } else if (normal_ok && !mv_ok) {
    Serial.println("detectEcLibUnit: decided -> library expects V");
    return false;
  } else {
    // Ambiguo: preferir V (comportamiento por defecto) y avisar
    Serial.println("detectEcLibUnit: ambiguous result; defaulting to V (please check Serial outputs)");
    return false;
  }
}

void loadCalibrationPrefs() {
  prefs.begin(PREF_NAMESPACE, true);
  bool ok = prefs.getBool(PREF_CAL_OK, false);
  if (ok) {
    cal_captured = true;
    cal_voltage_point = prefs.getFloat(PREF_LAST_CAL_V, NAN);
    cal_ec_value = prefs.getFloat(PREF_LAST_CAL_EC, 1.413f);
    cal_saved = true;
    Serial.printf("Loaded calibration prefs: V=%.6f V, EC=%.4f mS/cm (%.0f µS/cm)\n",
                  cal_voltage_point, cal_ec_value, cal_ec_value * 1000.0f);
  } else {
    cal_saved = false;
    cal_captured = false;
    Serial.println("No calibration in prefs");
  }
  prefs.end();
}

void saveCalibrationPrefs() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putBool(PREF_CAL_OK, true);
  prefs.putFloat(PREF_LAST_CAL_V, cal_voltage_point);
  prefs.putFloat(PREF_LAST_CAL_EC, cal_ec_value);
  prefs.end();
  Serial.printf("Saved calibration prefs: V=%.6f V, EC=%.4f mS/cm (%.0f µS/cm)\n",
                cal_voltage_point, cal_ec_value, cal_ec_value * 1000.0f);
}

// Ensures peer added; if fails, sets useBroadcastFallback true
bool ensurePeerAdded() {
  if (peerAdded) return true;

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 6;
  peer.encrypt = false;

  esp_err_t rc = esp_now_add_peer(&peer);
  if (rc == ESP_OK) {
    Serial.println("ensurePeerAdded: peer agregado OK");
    peerAdded = true;
    useBroadcastFallback = false;
    return true;
  } else {
    Serial.printf("ensurePeerAdded: fallo esp_now_add_peer 0x%08X -> fallback broadcast\n", (unsigned)rc);
    peerAdded = false;
    useBroadcastFallback = true;
    return false;
  }
}

// Send with retries: queue send, wait small time for callback, retry up to attempts
bool sendWithRetries(const uint8_t* dest, const uint8_t* data, size_t len) {
  const int maxAttempts = 3;
  const unsigned long waitMsAfterSend = 250;  // small window to see callback result
  for (int attempts = 0; attempts < maxAttempts; ++attempts) {
    lastSendStatus = ESP_NOW_SEND_FAIL;  // reset
    esp_err_t rc = esp_now_send(dest, data, len);
    if (rc != ESP_OK) {
      Serial.printf("esp_now_send returned ERROR 0x%08X (attempt %d)\n", (unsigned)rc, attempts + 1);
      delay(100 + attempts * 100);
      continue;
    }
    unsigned long t0 = millis();
    while (millis() - t0 < waitMsAfterSend) {
      delay(10);
    }
    if (lastSendStatus == ESP_NOW_SEND_SUCCESS) {
      Serial.printf("sendWithRetries: success on attempt %d\n", attempts + 1);
      return true;
    } else {
      Serial.printf("sendWithRetries: callback NOT success (status=%d) attempt %d\n", (int)lastSendStatus, attempts + 1);
      delay(100 + attempts * 100);
    }
  }
  return false;
}

// Send message via ESP-NOW with peer-management & retries (uses broadcast fallback if needed)
void sendECMessage(float ec, float temp, uint8_t kind) {
  MsgEC msg;
  msg.kind = kind;
  msg.ec = ec;
  msg.tempC = temp;
  msg.seq = seqSend++;

  uint8_t dest[6];
  if (!peerAdded) ensurePeerAdded();

  if (peerAdded) {
    memcpy(dest, peerMAC, 6);
    Serial.printf("sendECMessage: attempt send to peer %02X:%02X:%02X:%02X:%02X:%02X kind=%u seq=%u\n",
                  dest[0], dest[1], dest[2], dest[3], dest[4], dest[5], kind, msg.seq);
    bool ok = sendWithRetries(dest, (uint8_t*)&msg, sizeof(msg));
    if (!ok) {
      Serial.println("sendECMessage: failed to send to peer after retries");
      // fallback: try broadcast once
      uint8_t bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      Serial.println("sendECMessage: trying broadcast fallback");
      esp_err_t rc = esp_now_send(bcast, (uint8_t*)&msg, sizeof(msg));
      Serial.printf("sendECMessage: broadcast esp_now_send rc=0x%08X\n", (unsigned)rc);
    }
  } else {
    // peer not added -> use broadcast
    uint8_t bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    Serial.println("sendECMessage: peer not added, sending broadcast");
    esp_err_t rc = esp_now_send(bcast, (uint8_t*)&msg, sizeof(msg));
    Serial.printf("sendECMessage: broadcast esp_now_send rc=0x%08X\n", (unsigned)rc);
  }
}

// Draw main screen: shows ESP-NOW OK/ERR based on lastSendStatus (same behavior as distance sketch)
void drawMainScreen() {
  u8g2.clearBuffer();

  // 1) Estado ESP-NOW (línea superior, centrada)
  u8g2.setFont(u8g2_font_ncenB08_tr); // fuente más grande
  const char* status = (lastSendStatus == ESP_NOW_SEND_SUCCESS) ? "ESP-NOW: OK" : "ESP-NOW: ERR";
  int status_w = u8g2.getStrWidth(status);
  u8g2.drawStr((128 - status_w) / 2, 14, status);

  // 2) EC (segunda línea) -- mostrar en µS/cm usando lastEC
  u8g2.setFont(u8g2_font_6x10_tf);
  char buf[40];
  if (haveECreading && !isnan(lastEC)) {
    // lastEC está en µS/cm; mostramos sin decimales si es >1000, con 1 decimal si <1000
    if (lastEC >= 1000.0f) {
      // valores grandes: mostrar sin decimales
      snprintf(buf, sizeof(buf), "EC: %.0f uS/cm", lastEC);
    } else {
      // valores pequeños: 1 decimal
      snprintf(buf, sizeof(buf), "EC: %.1f uS/cm", lastEC);
    }
  } else {
    snprintf(buf, sizeof(buf), "EC: --");
  }
  int ec_w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - ec_w) / 2, 36, buf);

  // 3) Temp agua (tercera línea)
  if (!isnan(lastTemp)) {
    snprintf(buf, sizeof(buf), "Temp agua: %.2f C", lastTemp);
  } else {
    snprintf(buf, sizeof(buf), "Temp agua: --");
  }
  int t_w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - t_w) / 2, 56, buf);

  u8g2.sendBuffer();
}

// Draw calibration screen
void drawCalScreen(const char* l1, const char* l2, const char* l3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 12, "=== CALIBRACION EC ===");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(2, 28, l1);
  u8g2.drawStr(2, 44, l2);
  u8g2.drawStr(2, 60, l3);
  u8g2.sendBuffer();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(50);

  // I2C & OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();

  // Buttons
  pinMode(BTN_ENTER_CAL, INPUT_PULLUP);
  pinMode(BTN_SAVE_CAL, INPUT_PULLUP);
  pinMode(BTN_EXIT_CAL, INPUT_PULLUP);

  // Sensors
  ds.begin();
  ads.setGain(GAIN_ONE);
  if (!ads.begin()) Serial.println("ADS1115 init FAILED");
  ecSensor.begin();

  // Load calibration
  loadCalibrationPrefs();

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  delay(50);
  esp_err_t rc = esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (rc == ESP_OK) Serial.println("esp_wifi_set_channel -> OK (chan 6)");
  else Serial.printf("esp_wifi_set_channel -> ERROR 0x%08X\n", (unsigned)rc);

  Serial.print("Local MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error init ESP-NOW");
  } else {
    esp_now_register_send_cb(onSendCallback);
    ensurePeerAdded();
  }

  drawMainScreen();
}

// ---------- Loop ----------
void loop() {
  static unsigned long lastTempTick = 0;
  static unsigned long lastBtnEnter_ms = 0, lastBtnSave_ms = 0, lastBtnExit_ms = 0;
  unsigned long now = millis();

  // 1) Leer temperatura cada 1s
  if (now - lastTempTick >= 1000UL) {
    lastTempTick = now;
    ds.requestTemperatures();
    float t = ds.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C || isnan(t)) {
      if (isnan(lastTemp)) lastTemp = 25.0f;
    } else {
      lastTemp = t;
    }
  }

  // 2) Botones (active LOW, con debounce)
  // ENTER_CAL: entrar en modo calibración
  if (digitalRead(BTN_ENTER_CAL) == LOW && now - lastBtnEnter_ms > DEBOUNCE_MS) {
    lastBtnEnter_ms = now;
    if (!inCalibration) {
      inCalibration = true;
      cal_captured = false;
      drawCalScreen("Place probe in 1413 uS/cm", "Press SAVE to capture", "EXIT to abort");
      Serial.println("Entered calibration mode: place probe in 1413 uS/cm, press SAVE");
    }
  }

  // SAVE_CAL
  if (digitalRead(BTN_SAVE_CAL) == LOW && now - lastBtnSave_ms > DEBOUNCE_MS) {
    lastBtnSave_ms = now;

    if (inCalibration) {
      // --- Calibración ---
      float volts = readADSVoltageAvg(10, 12); // V promedio
      float t = isnan(lastTemp) ? 25.0f : lastTemp;
      Serial.printf("Calibration capture (avg): V=%.6f V, T=%.2f\n", volts, t);

      cal_voltage_point = volts;
      cal_captured = true;

      // Antes de decidir/refinar, detectamos unidad de la librería si aún no lo hicimos
      if (!ecUnitDetected) {
        ecLibExpects_mV = detectEcLibUnit(volts, t);
        ecUnitDetected = true;
      }

      // Estimación preliminar de EC (mS/cm) usando la convención detectada
      float argVoltsForEst = ecLibExpects_mV ? (volts * 1000.0f) : volts;
      float ecEst_mS = ecSensor.readEC(argVoltsForEst, t);
      Serial.printf("Estimated EC (pre-cal, using %s): %.4f mS/cm (%.0f uS/cm)\n",
                    ecLibExpects_mV ? "mV" : "V", ecEst_mS, ecEst_mS * 1000.0f);

      // Referencias y tolerancia
      const float REF1 = 1.413f;   // 1413 µS/cm -> 1.413 mS/cm
      const float REF2 = 12.88f;   // 12.88 mS/cm
      const float TOLERANCE = 0.20f; // 20% (un poco más permisivo)

      bool detected = false;
      float detectedRef = 0.0f;
      if (isfinite(ecEst_mS)) {
        if (fabs(ecEst_mS - REF1) <= REF1 * TOLERANCE) {
          detected = true;
          detectedRef = REF1;
          Serial.println("Detected buffer: 1413 uS/cm (1.413 mS/cm)");
        } else if (fabs(ecEst_mS - REF2) <= REF2 * TOLERANCE) {
          detected = true;
          detectedRef = REF2;
          Serial.println("Detected buffer: 12.88 mS/cm");
        } else {
          detected = false;
          Serial.println("Buffer detection AMBIGUO (no close match)");
        }
      } else {
        Serial.println("ecSensor.readEC returned NaN during detection.");
      }

      // Si detectó, actualizamos cal_ec_value; si no, conservamos el valor anterior
      if (detected) {
        cal_ec_value = detectedRef; // mS/cm
      } else {
        Serial.println("No detection confident: keeping previous cal_ec_value.");
      }

      // Mostrar en OLED
      char line1[32];
      if (detected) {
        snprintf(line1, sizeof(line1), "Detected: %.0f uS/cm", cal_ec_value * 1000.0f);
      } else {
        snprintf(line1, sizeof(line1), "Detect ambiguous!");
      }
      drawCalScreen(line1, "Applying calibration...", "");

      // Secuencia de calibración: usar argVolts consistente con la librería
      float argVoltsCal = ecLibExpects_mV ? (cal_voltage_point * 1000.0f) : cal_voltage_point;
      Serial.printf("Calling calibration sequence (ref=%.4f mS/cm) using %s\n",
                    cal_ec_value, ecLibExpects_mV ? "mV" : "V");

      ecSensor.calibration(0.0f, 0.0f, (char*)"enterec");
      delay(200);
      ecSensor.calibration(argVoltsCal, t, (char*)"calec");
      delay(300);
      ecSensor.calibration(0.0f, 0.0f, (char*)"exitec");
      delay(200);

      // Guardar en NVS
      saveCalibrationPrefs();
      cal_saved = true;

      if (detected) {
        Serial.printf("Calibration finished. Ref = %.4f mS/cm (%.0f uS/cm)\n", cal_ec_value, cal_ec_value * 1000.0f);
        drawCalScreen(line1, "Calibration saved to NVS", "");
      } else {
        Serial.println("Calibration finished but ambiguous; check Serial output.");
        drawCalScreen("Calibration ambiguous", "Check Serial output", "");
      }

    } else {
      // --- Lectura manual EC (SAVE fuera de calibración) ---
      float volts = readADSVoltageAvg(6, 12); // V promedio
      float t = isnan(lastTemp) ? 25.0f : lastTemp;
      Serial.printf("Manual EC read (avg): V=%.6f V, T=%.2f\n", volts, t);
      lastVoltage = volts;

      // Detectar unidad la primera vez si no lo hemos hecho
      if (!ecUnitDetected) {
        ecLibExpects_mV = detectEcLibUnit(volts, t);
        ecUnitDetected = true;
      }

      float argVolts = ecLibExpects_mV ? (volts * 1000.0f) : volts;
      float ecVal_mS = ecSensor.readEC(argVolts, t);

      Serial.printf("DEBUG: used %s for readEC. ecVal_mS=%.6f mS/cm -> %.1f uS/cm\n",
                    ecLibExpects_mV ? "mV" : "V", ecVal_mS, ecVal_mS * 1000.0f);

      if (!isnan(ecVal_mS) && isfinite(ecVal_mS)) {
        lastEC = ecVal_mS * 1000.0f; // µS/cm
        haveECreading = true;
        Serial.printf("EC read: %.6f mS/cm == %.1f uS/cm (used %s)\n", ecVal_mS, lastEC, ecLibExpects_mV ? "mV" : "V");
      } else {
        Serial.println("EC read returned NaN or invalid");
        haveECreading = false;
      }
      drawMainScreen();
    }
  }

  // EXIT_CAL: salir de calibración o (en modo normal) enviar EC+temp si existe lectura
  if (digitalRead(BTN_EXIT_CAL) == LOW && now - lastBtnExit_ms > DEBOUNCE_MS) {
    lastBtnExit_ms = now;
    if (inCalibration) {
      // Salir/abortar calibración
      inCalibration = false;
      drawMainScreen();
      Serial.println("Calibration aborted / exited");
    } else {
      // En modo normal: enviar la última lectura de EC por ESP-NOW
      if (haveECreading && !isnan(lastEC)) {
        float t = isnan(lastTemp) ? 25.0f : lastTemp;
        // enviamos EC+temp con kind==2 (convención)
        sendECMessage(lastEC, t, KIND_EC);
      } else {
        Serial.println("No EC reading available to send (press SAVE to read first)");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.drawStr(2, 20, "No EC read. Press SAVE first.");
        u8g2.sendBuffer();
        delay(800);
        drawMainScreen();
      }
    }
  }

  // 3) UI update
  static unsigned long lastUi = 0;
  if (now - lastUi >= 500) {
    lastUi = now;
    if (!inCalibration) drawMainScreen();
    else {
      if (!cal_captured) drawCalScreen("Place probe in 1413 uS/cm", "Press SAVE to capture", "EXIT to abort");
      else drawCalScreen("Captured point", "Saved to NVS (if OK)", "EXIT to continue");
    }
  }

  delay(10);
}

