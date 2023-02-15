// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>

#define DHTPIN    7
#define DHTTYPE   DHT11     // DHT 11
#define MOTIONPIN 6
#define LEDPIN    13

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);
int motionState = LOW;
uint32_t delayMS;

char sensorOnMsg[] = "on";
char sensorOffMsg[] = "off";

void sendTelemetry(DynamicJsonDocument value) {
  delay(250); // Delay to let previous message send
  Serial.println("Sending Message: ");
  serializeJson(value, Serial3);
  serializeJsonPretty(value, Serial);
  Serial.println();
  delay(250); // Delay to let previous message send
}

float getTempData() {
  // Get temperature event
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
    return 0;
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    return event.temperature;
  }
}

float getHumidityData() {
  // Get humidity event
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading temperature!"));
    return 0;
  }
  else {
    Serial.print(F("Relative Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
    return event.relative_humidity;
  }
}

char* getMotionData() {
  int val = digitalRead(MOTIONPIN);  // read input value
  if (val == HIGH) {  // check if the input is HIGH
    digitalWrite(LEDPIN, HIGH);  // turn LED ON
    if (motionState == LOW) {
      motionState = HIGH;
      Serial.print(F("Movement Status: "));
      Serial.println(sensorOnMsg);
    }
    return sensorOnMsg;
  }
  else {
    digitalWrite(LEDPIN, LOW); // turn LED OFF
    if (motionState == HIGH) {
      motionState = LOW;
      Serial.print(F("Movement Status: "));
      Serial.println(sensorOffMsg);
    }
    return sensorOffMsg;
  }
  return 0; // Something is wrong...
}

void setup() {
  pinMode(LEDPIN, OUTPUT);        // declare LED as output
  pinMode(MOTIONPIN, INPUT);      // declare sensor as input

  Serial.begin(115200);
  Serial3.begin(115200);
  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;
}

void loop() {
  // Delay between measurements.
  delay(delayMS);
  DynamicJsonDocument doc(100);
  doc["temp"] = getTempData();
  doc["humidity"] = getHumidityData();
  doc["movement"] = getMotionData();
  doc["availability"] = "online";
  sendTelemetry(doc);
}