/********************************************************************
   Fusion-sensor v10 – MQ-2 + HR + GPS + NTC + LEDs  (+ HTTP REST API)
   Versión optimizada para Wokwi (no bloquear, menos latencia)
******************************************************************** sandero version */

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

/* Usa SIEMPRE HTTPS en Railway */
#define BACKEND_URL   "https://secureon-backend-production.up.railway.app"
#define API_ENDPOINT  "/api/secureon/v1/records"

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
TinyGPSPlus      gps;
DisplayActuator  disp;
MQUnifiedsensor  MQ2("ESP32", 3.3, 12, GAS_PIN, "MQ-2");

/* -------- Variables -------------------------------------------- */
int   gasPct = 0,  lastGas = -1;
int   bpm    = 0,  lastBpm = -1;
float tempC  = 25, lastTempSent = -999;
bool  safe   = true, lastSafeSent = true;

/* Cache para UI (evitar redibujados) */
String line0, line1, line2, line3;
String lastLine0, lastLine1, lastLine2, lastLine3;

/* -------- Timers (ms) ------------------------------------------ */
unsigned long t100 = 0, t500 = 0, t2000 = 0, tPost = 0;

/* -------- Intervalos ------------------------------------------- */
#define MIN_POST_INTERVAL_MS   8000UL  // No postear más de 1 vez cada 8 s
#define WIFI_RETRY_MS          15000UL // Reintentar WiFi cada 15 s si cae

unsigned long lastWifiAttempt = 0;

/* -------- Sensor Validation Ranges (same as backend) ---------- */
#define GAS_MIN           10.0
#define GAS_MAX           40.0
#define HEART_RATE_MIN    40.0
#define HEART_RATE_MAX    160.0
#define TEMP_MIN          -50.0
#define TEMP_MAX          100.0

/* =============================================================== */

bool validateSensor(float gasValue, float heartRate, float temperature) {
  bool gasOk  = (gasValue    >= GAS_MIN        && gasValue    <= GAS_MAX);
  bool hrOk   = (heartRate   >= HEART_RATE_MIN && heartRate   <= HEART_RATE_MAX);
  bool tempOk = (temperature >= TEMP_MIN       && temperature <= TEMP_MAX);
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
    yield(); // Evita bloqueo del simulador
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed (non-blocking).");
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
  client.setInsecure();        // para pruebas; en prod usa root CA
  client.setTimeout(4000);

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("HTTP begin failed");
    return false;
  }

  http.setTimeout(5000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // por si acaso
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.setReuse(false);  // evita reuso que a veces causa asserts

  StaticJsonDocument<384> doc;
  doc["sensorId"]         = SENSOR_ID;
  doc["deliveryId"]       = DELIVERY_ID;
  doc["gasValue"]         = gasValue;
  doc["heartRateValue"]   = heartRate;
  doc["temperatureValue"] = temperature;
  doc["latitude"]         = latitude;
  doc["longitude"]        = longitude;
  // "timestamp" es opcional según tu Swagger; lo omitimos

  String payload;
  serializeJson(doc, payload);

  int code = http.POST((uint8_t*)payload.c_str(), payload.length());
  bool ok = false;

  if (code > 0) {
    Serial.printf("HTTP Response code: %d\n", code);

    if (code == 201 || (code >= 200 && code < 300)) {
      ok = true;
      // puedes leer respuesta si quieres:
      // String body = http.getString();
      // Serial.println(body);
    } else if (code >= 300 && code < 400) {
      // Muestra adónde te quería mandar
      String loc = http.getLocation();
      Serial.printf("Redirect to: %s\n", loc.c_str());
    } else {
      // Errores 4xx/5xx
      String body = http.getString();
      Serial.printf("Error response: %s\n", body.c_str());
    }
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

  // Sobrescribir línea: mueve cursor y escribe espacios para “limpiar”
  disp.setCursor(0, row);
  String padding = text;
  while (padding.length() < 20) padding += ' '; // depende de tu LCD; ajusta ancho
  disp.printText(padding);

  // Reescribe contenido final (para evitar restos)
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

  Serial.println("\n=== Fusion Sensor v10 (Wokwi-optimized) ===");
  Serial.printf("Sensor ID: %ld\n", (long)SENSOR_ID);
  Serial.printf("Delivery ID: %ld\n", (long)DELIVERY_ID);

  connectWiFi();

  MQ2.init();

  disp.clear();
  drawIfChanged(0, "Fusion Ready!", lastLine0);
  drawIfChanged(1, " ",             lastLine1);
  drawIfChanged(2, " ",             lastLine2);
  drawIfChanged(3, " ",             lastLine3);

  delay(500);
}

/* =============================================================== */

void loop() {
  /* ---------------- GPS (no bloqueante) ---------------- */
#if ENABLE_GPS
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
#endif

  /* 100 ms : Gas % + BPM ------------------------------- */
  if (millis() - t100 >= 100) {
    t100 = millis();
    gasPct = map(analogRead(GAS_PIN), 0, 4095, 0, 100);
    bpm    = map(analogRead(HR_PIN ), 0, 4095, 0, 200);
  }

  /* 2 s : Temperatura NTC ------------------------------ */
  if (millis() - t2000 >= 2000) {
    t2000 = millis();
    const float BETA = 3950.0;
    int raw = analogRead(NTC_PIN);
    if (raw > 0 && raw < 4095) { // evitar division por cero/inf
      float tc = 1.0f / (log(1.0f / (4095.0f / raw - 1.0f)) / BETA + 1.0f / 298.15f) - 273.15f;
      if (!isnan(tc) && isfinite(tc)) tempC = tc;
    }
  }

  /* 500 ms : UI + lógica de seguridad + (posible) POST -- */
  if (millis() - t500 >= 500) {
    t500 = millis();

    // Validación de sensores
    safe = validateSensor((float)gasPct, (float)bpm, tempC);
    digitalWrite(LED_RED_PIN,   safe ? LOW  : HIGH);
    digitalWrite(LED_GREEN_PIN, safe ? HIGH : LOW);

    /* -------- GPS texto ------------------------------- */
    char gpsBuf[40] = "GPS WAIT";
    double lat = 0.0, lon = 0.0;

#if ENABLE_GPS
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      snprintf(gpsBuf, sizeof(gpsBuf), "Lat:%.4f Lng:%.4f", lat, lon);
    }
#else
    // Sin GPS: mostrar espera pero no bloquear
    snprintf(gpsBuf, sizeof(gpsBuf), "GPS WAIT");
#endif

    /* -------- Consola --------------------------------- */
    Serial.printf(" Gas=%02d%% HR=%03d T=%.1fC  %s  safe:%s\n",
                  gasPct, bpm, tempC, gpsBuf, safe ? "true" : "false");

    /* -------- LCD (solo si cambió) -------------------- */
    line0 = "HR :" + String(bpm)       + "bpm";
    line1 = "Tmp:" + String(tempC, 1)  + "C";
    line2 = "Gas:" + String(gasPct)    + "%";
    line3 = safe ? "SAFE" : "UNSAFE";

    drawIfChanged(0, line0, lastLine0);
    drawIfChanged(1, line1, lastLine1);
    drawIfChanged(2, line2, lastLine2);
    drawIfChanged(3, line3, lastLine3);

    /* -------- HTTP POST (acelerado) ------------------- */
#if ENABLE_WIFI
    // Enviar solo si hay cambios significativos y pasó el intervalo mínimo
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
        // Reintento no bloqueante de WiFi
        if (millis() - lastWifiAttempt >= WIFI_RETRY_MS) {
          Serial.println("WiFi disconnected, attempting reconnect...");
          lastWifiAttempt = millis();
          connectWiFi();
        }
      }
    }
#endif
  }

  yield(); // Respira para el scheduler del simulador
}
