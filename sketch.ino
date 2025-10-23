/********************************************************************
   Fusion-sensor v9 – MQ-2 + HR + GPS + NTC + LEDs  (+ MQTT TLS)
********************************************************************/
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>     // NEW
#include <PubSubClient.h>         // NEW  (instala “PubSubClient” en el IDE)
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <MQUnifiedsensor.h>
#include "DisplayActuator.h"

/* -------- Wi-Fi & MQTT ----------------------------------------- */
#define WIFI_SSID       "Wokwi-GUEST"
#define WIFI_PASSWORD   ""

#define MQTT_BROKER     "b-e777a566-934e-496c-a479-2e4d9c5a17d4-1.mq.us-east-1.amazonaws.com"
#define MQTT_PORT       8883
#define MQTT_USER       "kkita"     
#define MQTT_PASS       "Josue270333333333"
#define MQTT_TOPIC      "test/topic"        

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
WiFiClientSecure netClient;             // NEW
PubSubClient     mqtt(netClient);       // NEW
MQUnifiedsensor  MQ2("ESP32", 3.3, 12, GAS_PIN, "MQ-2");

/* -------- Variables -------------------------------------------- */
int   gasPct = 0,  lastGas = -1;
int   bpm    = 0,  lastBpm = -1;
float tempC  = 25, lastTemp = -999;
bool  safe   = true, lastSafe = true;

/* -------- Timers (ms) ------------------------------------------ */
unsigned long t100 = 0, t500 = 0, t2000 = 0;

/* =============================================================== */
void connectWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(250); }
}

void connectMQTT()
{
  /*  TLS: acepta cualquier certificado; para demo/Wokwi basta.       */
  netClient.setInsecure();                       // NEW
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);

  while (!mqtt.connected()) {
    if (mqtt.connect("fusion-sensor-esp32", MQTT_USER, MQTT_PASS)) break;
    delay(1000);
  }
}
/* =============================================================== */
void setup() {
  Wire.begin(21, 22);
  disp.init();

  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  connectWiFi();          // NEW
  connectMQTT();          // NEW

  MQ2.init();

  disp.clear();  disp.printText("Fusion Ready!");
}

/* =============================================================== */
void loop() {
  mqtt.loop();            // NEW  — mantiene viva la sesión MQTT

  /* ---------------- GPS ---------------- */
  while (Serial2.available()) gps.encode(Serial2.read());

  /* 100 ms : Gas %  + BPM --------------- */
  if (millis() - t100 >= 100) {
    t100 = millis();
    gasPct = map(analogRead(GAS_PIN), 0, 4095, 0, 100);
    bpm    = map(analogRead(HR_PIN ), 0, 4095, 0, 200);
  }

  /* 2 s : Temperatura NTC -------------- */
  if (millis() - t2000 >= 2000) {
    t2000 = millis();
    const float BETA = 3950;
    int raw = analogRead(NTC_PIN);
    float tc = 1 / (log(1 / (4095. / raw - 1)) / BETA + 1 / 298.15) - 273.15;
    if (!isnan(tc)) tempC = tc;
  }

  /* 500 ms : UI + MQTT PUBLISH --------- */
  if (millis() - t500 >= 500) {
    t500 = millis();

    safe = gasPct <= 70;
    digitalWrite(LED_RED_PIN,   safe ? LOW  : HIGH);
    digitalWrite(LED_GREEN_PIN, safe ? HIGH : LOW);

    /* -------- GPS texto --------------- */
    char gpsBuf[40] = "GPS WAIT";
    double lat = 0, lon = 0;
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      snprintf(gpsBuf, sizeof(gpsBuf), "Lat:%.4f Lng:%.4f", lat, lon);
    }

    /* -------- LCD --------------------- */
    disp.clear();
    disp.setCursor(0, 0); disp.printText("HR :" + String(bpm)     + "bpm");
    disp.setCursor(0, 1); disp.printText("Tmp:" + String(tempC,1) + "C");
    disp.setCursor(0, 2); disp.printText("Gas:" + String(gasPct)  + "%");
    disp.setCursor(0, 3); disp.printText(safe ? "SAFE" : "UNSAFE");

    /* -------- Consola ----------------- */
    Serial.printf("Gas=%02d%% HR=%03d T=%.1fC  %s  safe:%s ,unsafe:%s\n",
                  gasPct, bpm, tempC, gpsBuf,
                  safe ? "true" : "false", safe ? "false" : "true");

    int tInt = (int)round(tempC);
    if (gasPct != lastGas || bpm != lastBpm ||
        tInt != lastTemp || safe != lastSafe) {

      /* ----- JSON payload ------------- */
      StaticJsonDocument<256> js;
      js["deviceId"]    = "HR-2947";
      js["heartRate"]   = bpm;
      js["temperature"] = tInt;
      js["gasValue"]    = gasPct;
      js["safe"]        = safe;
      js["unsafe"]      = !safe;
      js["latitude"]    = lat;
      js["longitude"]   = lon;

      char buf[256];
      size_t n = serializeJson(js, buf);

      /* ----- MQTT PUBLISH ------------- */
      if (!mqtt.connected()) connectMQTT();      // reconecta si hace falta
      bool ok = mqtt.publish(MQTT_TOPIC, buf, n);
      Serial.printf("MQTT PUBLISH → %s\n", ok ? "OK" : "FAIL");

      lastGas  = gasPct;
      lastBpm  = bpm;
      lastTemp = tInt;
      lastSafe = safe;
    }
  }
}
