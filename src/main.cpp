#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <SHT2x.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "config.hpp"

TwoWire wire;
SHT2x sensor;
WiFiClient wifiClient;
ESP8266WebServer server;
PubSubClient mqtt(MQTT_SERVER, 1883, wifiClient);

ConfigType config;
String sensorId = "";
const String mqttPrefix = "homeassistant/sensor/";

float temperature = 0;
float humidity = 0;
float battery = 0;
int batteryPercent = 0;
int batteryRaw = 0;
unsigned long lastRead = 0;
unsigned long setupFinished = 0;

void respondJson(int code, const JsonDocument& json)
{
  String jsonStr;
  serializeJson(json, jsonStr);
  server.send(code, "application/json", jsonStr);
}

void mqttCb(char* topic, byte* payload, unsigned int length)
{
  payload[length] = '\0';
  String data = String((char*) payload);
  Serial.println("Got config from MQTT: " + data);
  StaticJsonDocument<CONFIG_JSON_SIZE> json;
  deserializeJson(json, data);
  bool changed = config.updateFromJson(json);

  if (changed)
  {
    delay(50);
    ESP.deepSleep(1000, RF_DISABLED);
  }
}

bool connectMQTT()
{
  Serial.print("Attempting MQTT connection...");
  mqtt.setCallback(mqttCb);
  if (mqtt.connect(sensorId.c_str(), MQTT_USER, MQTT_PASS))
  {
    Serial.println("connected");
    String configTopic = mqttPrefix + sensorId;
    // subscribe here if needed
    mqtt.subscribe(configTopic.c_str());
    return true;
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
    return false;
  }
}

void publishWDiscovery(String subtopic, String deviceClass, String unit, double value)
{
  String subId = sensorId + "_" + subtopic;
  String configTopic = mqttPrefix + subId + "/config";
  String stateTopic = mqttPrefix + subId + "/state";

  StaticJsonDocument<512> json;
  json["dev"]["identifiers"] = sensorId;
  json["dev"]["model"] = "ESP8266";
  json["dev"]["name"] = "ESP8266 Temp/Humidity Unit";
  json["dev_cla"] = deviceClass;
  json["name"] = config.name + " " + subtopic;
  json["stat_cla"] = "measurement";
  json["stat_t"] = stateTopic;
  json["uniq_id"] = subId;
  json["unit_of_meas"] = unit;

  String jsonStr;
  serializeJson(json, jsonStr);
  mqtt.publish(configTopic.c_str(), jsonStr.c_str(), true);
  mqtt.publish(stateTopic.c_str(), String(value).c_str(), true);
}

void publishMQTT()
{
  publishWDiscovery(POSTFIX_TEMP, "temperature", "°C", temperature);
  publishWDiscovery(POSTFIX_HUMI, "humidity", "%", humidity);
  yield();
  publishWDiscovery(POSTFIX_BATT, "voltage", "V", battery);
  publishWDiscovery(POSTFIX_BPER, "battery", "%", batteryPercent);
  yield();
  mqtt.publish((mqttPrefix + sensorId + "_" + POSTFIX_BRAW).c_str(), String(batteryRaw).c_str(), true);
}

long readAnalog()
{
  long smallest = analogRead(A0), other;
  for (int i = 0; i < PWR_READ_COUNT; i++)
  {
    other = analogRead(A0);
    if (other < smallest)
    {
      smallest = other;
    }
  }
  batteryRaw = smallest;
  return smallest;
}

long getBattery()
{
  long val = readAnalog();
  long mapped = map(val, config.batteryRawLo, config.batteryRawHi, BATTERY_MIN_mV, BATTERY_MAX_mV);
  return mapped;
}

int getBatteryPercent(long mapped)
{
  int percent = map(mapped, BATTERY_MIN_mV, BATTERY_MAX_mV, 0, 100);
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

void readSensor()
{
  sensor.read();
  temperature = sensor.getTemperature();
  humidity = sensor.getHumidity();
  lastRead = millis();
  Serial.println("Sensor readings updated.");
}

void readBattery()
{
  long batt = getBattery();
  battery = batt / 1000.;
  batteryPercent = getBatteryPercent(batt);
  Serial.println("Sensor readings updated.");
}

void handleSerial()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("s")) // get sensor values
    {
      readSensor();
      Serial.print("Temperature: ");
      Serial.println(temperature);
      Serial.print("Humidity: ");
      Serial.println(humidity);
      Serial.print("Last read: ");
      Serial.println(lastRead);
    }
    if (input.startsWith("p")) // get power values
    {
      readBattery();
      int analog = analogRead(A0);
      Serial.print("Raw read value: ");
      Serial.println(analog);
      Serial.print("Battery sense: ");
      Serial.println(battery);
      Serial.print("Battery percent: ");
      Serial.println(batteryPercent);
    }
  }
}

void setupSensors()
{
  Serial.println("Setting up sensor / batterySense pins");
  pinMode(A0, INPUT); // Setup battery sense

  wire.begin();
  sensor.begin(&wire); // Setup sensor (I2C)
  uint8_t stat = sensor.getStatus();
  Serial.print("Sensor status: ");
  Serial.print(stat, HEX);
  Serial.println();
}

void setupOTA()
{
  char tmp[64]; // Setup mqtt id with hostname-mac
  uint8_t mac[6];
  WiFi.macAddress(mac);
  sprintf(tmp, "%s-%02x%02x%02x%02x%02x%02x", config.hostname.c_str(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  sensorId = tmp;
  Serial.println("sensorId set to:" + sensorId);

  Serial.println("Starting OTA Server");
  ArduinoOTA.setHostname(config.hostname.c_str());
  Serial.println("mDNS set to: " + config.hostname);
  ArduinoOTA.begin();
}

void connectWifi()
{
  Serial.printf("Connecting to WiFi: %s ", SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500); 
    Serial.print(".");
  }
  Serial.print("\nConnected, IP: ");
  Serial.println(WiFi.localIP());
}

void setupHttp()
{
  // setup server
  Serial.println("Starting Server.");
  server.on("/", HTTP_GET, []() {
    Serial.println("Request: GET /");
    StaticJsonDocument<256> json;
    readSensor();
    readBattery();
    json["temperature"] = temperature;
    json["humidity"] = humidity;
    json["lastread"] = lastRead;
    json["battery"] = battery;
    json["battery_percent"] = batteryPercent;
    json["battery_raw"] = batteryRaw;
    respondJson(200, json);
  });
  server.on("/lastread", HTTP_GET, []() {
    Serial.println("Request: GET /lastread");
    StaticJsonDocument<256> json;
    json["temperature"] = temperature;
    json["humidity"] = humidity;
    json["lastread"] = lastRead;
    json["battery"] = battery;
    json["battery_percent"] = batteryPercent;
    json["battery_raw"] = batteryRaw;
    respondJson(200, json);
  });
  server.on("/config", HTTP_GET, []() {
    Serial.println("Request: GET /config");
    StaticJsonDocument<CONFIG_JSON_SIZE> json;
    config.getAsJson(json);
    respondJson(200, json);
  });
  server.on("/config", HTTP_POST, []() {
    if (server.hasArg("plain") == false) return;
    Serial.println("Request: POST /config");

    StaticJsonDocument<CONFIG_JSON_SIZE> json;
    String body = server.arg("plain");
    Serial.print("POST: ");
    Serial.println(body);
    deserializeJson(json, body);

    bool changed = config.updateFromJson(json);
    json.clear();
    config.getAsJson(json);
    respondJson(200, json);
    if (changed)
    {
      delay(200);
      ESP.deepSleep(1000, RF_DISABLED);
    }
  });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n\nInit ...");

  config.begin();
  setupSensors();
  readBattery();
  Serial.println("Battery raw reading: " + String(batteryRaw));
  connectWifi();
  setupOTA();
  if (config.mode == MODE_CONFIG) {
    setupHttp();
  }
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(2);

  readSensor();
  while (!connectMQTT()) yield();
  if (config.mode & MODE_MQTT) {
    publishMQTT();
  }
  setupFinished = millis();
}

void loop() {
  ArduinoOTA.handle();
  if (config.mode == MODE_CONFIG)
  {
    handleSerial();
    server.handleClient();
  }
  mqtt.loop();
  delay(5);

  if (config.mode != MODE_CONFIG && (millis() - setupFinished) >= RUNTIME_ms)
  {
    Serial.println("Reached idle, sleeping for " + String(config.sleepSeconds) + "s.");
    ESP.deepSleep(config.sleepSeconds * uS_TO_S_FACTOR, RF_DISABLED);
  }
}
