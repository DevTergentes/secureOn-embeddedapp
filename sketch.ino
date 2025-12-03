/********************************************************************
   Fusion-sensor v10 – MQ-2 + HR + GPS + NTC + LEDs (+ HTTP REST API)
   Versión optimizada para Wokwi (no bloquear, menos latencia)
********************************************************************/

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <MQUnifiedsensor.h>
#include "DisplayActuator.h"

/* ======== FLAGS ========== */
#define ENABLE_WIFI   true
#define ENABLE_GPS    false
#define USE_HTTPS     true

/* -------- Wi-Fi & HTTP ----------------------------------------- */
#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASSWORD   ""
#define BACKEND_URL     "https://secureon-backend-production.up.railway.app"
#define API_ENDPOINT    "/api/secureon/v1/records"

/* -------- Device Configuration -------------------------------- */
#define SENSOR_ID       1L
#define DELIVERY_ID     1L

/* -------- GPIO -------------------------------------------------- */
#define HR_PIN          33
#define GAS_PIN         34
#define NTC_PIN         35
#define GPS_RX          16
#define GPS_TX          17
#define LED_RED_PIN      4
#define LED_GREEN_PIN   12

/* -------- Objetos ---------------------------------------------- */
TinyGPSPlus gps;
DisplayActuator disp;
MQUnifiedsensor MQ2("ESP32", 3.3, 12, GAS_PIN, "MQ-2");

/* -------- Variables -------------------------------------------- */
int gasPct = 0, lastGas = -1;
int bpm = 0, lastBpm = -1;
float tempC = 25, lastTempSent = -999;
bool safe = true, lastSafeSent = true;

/* Lat/Lon fijos cuando GPS está deshabilitado */
const double WOKWI_LAT = -12.0463;
const double WOKWI_LON = -77.0428;

/* Cache UI */
String line0, line1, line2, line3;
String lastLine0, lastLine1, lastLine2, lastLine3;

/* -------- Timers (ms) ------------------------------------------ */
unsigned long t100 = 0, t500 = 0, t2000 = 0, tPost = 0;
unsigned long lastWifiAttempt = 0;

/* -------- Intervalos ------------------------------------------- */
#define MIN_POST_INTERVAL_MS   8000UL
#define WIFI_RETRY_MS          15000UL

/* -------- Sensor Validation Ranges ------------------------------ */
#define GAS_MIN           10.0
#define GAS_MAX           40.0
#define HEART_RATE_MIN    40.0
#define HEART_RATE_MAX    160.0
#define TEMP_MIN          -50.0
#define TEMP_MAX          100.0

/* =============================================================== */
bool validateSensor(float gasValue, float heartRate, float temperature) {
  bool gasOk  = (gasValue >= GAS_MIN && gasValue <= GAS_MAX);
  bool hrOk   = (heartRate >= HEART_RATE_MIN && heartRate <= HEART_RATE_MAX);
  bool tempOk = (temperature >= TEMP_MIN && temperature <= TEMP_MAX);
  return gasOk && hrOk && tempOk;
}

/* =============================================================== */
void connectWiFi() {
#if ENABLE_WIFI
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(300);
    Serial.print(".");
    attempts++;
    yield();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }
#endif
}

/* =============================================================== */
bool sendDataToBackend(float heartRate, float temperature, float gasValue,
                       bool safeFlag, double latitude, double longitude) {
#if !ENABLE_WIFI
  return true;
#else
  if (WiFi.status() != WL_CONNECTED) return false;

  String url = String(BACKEND_URL) + String(API_ENDPOINT);

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(4000);

  HTTPClient http;
  if (!http.begin(client, url)) return false;

  http.setTimeout(5000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.setReuse(false);

  StaticJsonDocument<384> doc;
  doc["sensorId"]         = SENSOR_ID;
  doc["deliveryId"]       = DELIVERY_ID;
  doc["gasValue"]         = gasValue;
  doc["heartRateValue"]   = heartRate;
  doc["temperatureValue"] = temperature;
  doc["latitude"]         = latitude;
  doc["longitude"]        = longitude;

  String payload;
  serializeJson(doc, payload);

  int code = http.POST((uint8_t*)payload.c_str(), payload.length());
  bool ok = false;

  if (code > 0) {
    if (code == 201 || (code >= 200 && code < 300)) ok = true;
    else Serial.println(http.getString());
  } else {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
  }

  http.end();
  return ok;
#endif
}

/* =============================================================== */
void drawIfChanged(uint8_t row, const String& text, String& cache) {
  if (text == cache) return;
  cache = text;

  disp.setCursor(0, row);
  String padding = text;
  while (padding.length() < 20) padding += ' ';
  disp.printText(padding);

  disp.setCursor(0, row);
  disp.printText(text);
}

/* =============================================================== */
void setup() {
  Wire.begin(21, 22);
  disp.init();
  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  Serial.begin(115200);

#if ENABLE_GPS
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
#endif

  Serial.println("\n=== Fusion Sensor v10 ===");
  connectWiFi();
  MQ2.init();

  disp.clear();
  drawIfChanged(0, "Fusion Ready!", lastLine0);
}

/* =============================================================== */
void loop() {

#if ENABLE_GPS
  while (Serial2.available()) gps.encode(Serial2.read());
#endif

  if (millis() - t100 >= 100) {
    t100 = millis();
    gasPct = map(analogRead(GAS_PIN), 0, 4095, 0, 100);
    bpm    = map(analogRead(HR_PIN ), 0, 4095, 0, 200);
  }

  if (millis() - t2000 >= 2000) {
    t2000 = millis();

    const float BETA = 3950.0;
    int raw = analogRead(NTC_PIN);
    if (raw > 0 && raw < 4095) {
      float tc = 1.0f / (log(1.0f / (4095.0f / raw - 1.0f)) / BETA + 1.0f / 298.15f) - 273.15f;
      if (!isnan(tc) && isfinite(tc)) tempC = tc;
    }
  }

  if (millis() - t500 >= 500) {
    t500 = millis();

    safe = validateSensor((float)gasPct, (float)bpm, tempC);
    digitalWrite(LED_RED_PIN,   safe ? LOW  : HIGH);
    digitalWrite(LED_GREEN_PIN, safe ? HIGH : LOW);

    double lat = WOKWI_LAT;
    double lon = WOKWI_LON;

    char gpsBuf[40];
    snprintf(gpsBuf, sizeof(gpsBuf), "Lat:%.4f Lng:%.4f", lat, lon);

    Serial.printf(" Gas=%02d%% HR=%03d T=%.1fC  %s  safe:%s\n",
                  gasPct, bpm, tempC, gpsBuf, safe ? "true" : "false");

    line0 = "HR :" + String(bpm)       + "bpm";
    line1 = "Tmp:" + String(tempC, 1)  + "C Gas:" + String(gasPct) + "%";
    line2 = "Lat:" + String(lat, 4);
    line3 = "Lon:" + String(lon, 4) + (safe ? " SAFE" : " UNSAFE");

    drawIfChanged(0, line0, lastLine0);
    drawIfChanged(1, line1, lastLine1);
    drawIfChanged(2, line2, lastLine2);
    drawIfChanged(3, line3, lastLine3);

#if ENABLE_WIFI
    bool changedEnough =
      (gasPct != lastGas) ||
      (bpm    != lastBpm) ||
      ((int)round(tempC) != (int)round(lastTempSent)) ||
      (safe   != lastSafeSent);

    if (changedEnough && (millis() - tPost >= MIN_POST_INTERVAL_MS)) {
      if (WiFi.status() == WL_CONNECTED) {

        bool ok = sendDataToBackend(
          (float)bpm,
          (float)tempC,
          (float)gasPct,
          safe,
          lat,
          lon
        );

        Serial.printf("HTTP POST → %s\n", ok ? "OK" : "FAIL");

        if (ok) {
          lastGas       = gasPct;
          lastBpm       = bpm;
          lastTempSent  = tempC;
          lastSafeSent  = safe;
          tPost         = millis();
        }

      } else {
        if (millis() - lastWifiAttempt >= WIFI_RETRY_MS) {
          Serial.println("WiFi disconnected, attempting reconnect...");
          lastWifiAttempt = millis();
          connectWiFi();
        }
      }
    }
#endif
  }

  yield();
}
