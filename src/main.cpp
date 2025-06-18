#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define MODULE_TYPE "rotating-conveyor"

// --- WiFi & MQTT ---
#define WIFI_SSID "Beeline_MF"
#define WIFI_PASS "$@ndr0nix"
#define MQTT_SERVER "192.168.8.100"

WiFiClient espClient;
PubSubClient client(espClient);
String mac_id = "";

// --- VL53L0X ---
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

// --- Пины мотора ленты (мотор 1) ---
#define CONV_IN1 2
#define CONV_IN2 3

// --- Пины поворотного мотора (мотор 2) ---
#define ROT_IN1 4
#define ROT_IN2 5

// --- TCRT5000 ---
#define TCRT_PIN 6  // цифровой вывод D0 с компаратором

void setup_wifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  mac_id = WiFi.macAddress();
  Serial.println("\nWiFi connected, MAC: " + mac_id);
}

void sendIdentity() {
  String ip = WiFi.localIP().toString();
  client.publish(("module/" + mac_id + "/identity/type").c_str(), MODULE_TYPE);
  client.publish(("module/" + mac_id + "/identity/ip").c_str(), ip.c_str());
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(mac_id.c_str())) {
      Serial.println("Connected to MQTT");
      sendIdentity();
      client.subscribe(("module/" + mac_id + "/command").c_str());
    } else {
      delay(1000);
    }
  }
}

void startConveyor() {
  digitalWrite(CONV_IN1, HIGH);
  digitalWrite(CONV_IN2, LOW);
}

void endConveyor() {
  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
}

void rotateOnce(bool directionRight) {
  // Поворот, пока не увидим белую линию
  if (directionRight) {
    digitalWrite(ROT_IN1, HIGH);
    digitalWrite(ROT_IN2, LOW);
  } else {
    digitalWrite(ROT_IN1, LOW);
    digitalWrite(ROT_IN2, HIGH);
  }

  bool linePassed = false;
  while (!linePassed) {
    int tcrt = digitalRead(TCRT_PIN);
    if (tcrt == LOW) { // Белая метка обнаружена (напряжение упало)
      linePassed = true;
    }
    delay(10);
  }

  digitalWrite(ROT_IN1, LOW);
  digitalWrite(ROT_IN2, LOW);
}

void onMessage(char* topic, byte* payload, unsigned int length) {
  String cmd;
  for (unsigned int i = 0; i < length; i++) cmd += (char)payload[i];

  if (cmd == "start") {
    startConveyor();
  } else if (cmd == "end") {
    endConveyor();
  } else if (cmd == "right") {
    rotateOnce(true);
  } else if (cmd == "left") {
    rotateOnce(false);
  }
}

void sendVL53() {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    char topic[64];
    snprintf(topic, sizeof(topic), "module/%s/sensor/vl53l0x", mac_id.c_str());

    char msg[16];
    snprintf(msg, sizeof(msg), "%d", measure.RangeMilliMeter);
    client.publish(topic, msg);
  }
}

void sendTCRT() {
  int tcrt = digitalRead(TCRT_PIN);
  char topic[64];
  snprintf(topic, sizeof(topic), "module/%s/sensor/TCRT5000", mac_id.c_str());
  client.publish(topic, tcrt == LOW ? "1" : "0");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  pinMode(ROT_IN1, OUTPUT);
  pinMode(ROT_IN2, OUTPUT);
  pinMode(TCRT_PIN, INPUT);

  digitalWrite(CONV_IN1, LOW);
  digitalWrite(CONV_IN2, LOW);
  digitalWrite(ROT_IN1, LOW);
  digitalWrite(ROT_IN2, LOW);

  setup_wifi();
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(onMessage);

  if (!sensor.begin()) {
    Serial.println("VL53L0X not found!");
    while (1);
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  sendVL53();
  sendTCRT();
  delay(1000);
}
