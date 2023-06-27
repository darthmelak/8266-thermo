#ifndef ThermoConfig_h
#define ThermoConfig_h

#include <ArduinoJson.h>
#include <EEPROM.h>
#include "secrets.h"

#define uS_TO_S_FACTOR 1000000
#define CONFIG_JSON_SIZE 256
#define PWR_READ_COUNT 20
#define BATTERY_MIN_mV 3600
#define BATTERY_MAX_mV 4100
#define RUNTIME_ms 600

#ifndef secrets_h
#define SSID "SSID"
#define PASSWORD "password"
#define MQTT_SERVER "192.168.1.1"
#define MQTT_USER "user"
#define MQTT_PASS "password"
#endif

#define POSTFIX_TEMP "temperature"
#define POSTFIX_HUMI "humidity"
#define POSTFIX_BATT "battery"
#define POSTFIX_BPER "battery-percent"
#define POSTFIX_BRAW "battery-raw"

#define MODE_CONFIG 0b00
#define MODE_MQTT 0b01

#define C_NAME "name"
#define C_HOSTNAME "hostname"
#define C_MODE "mode"
#define C_SLEEP "sleep_sec"
#define C_BATT_L "battery_low"
#define C_BATT_H "battery_high"

#define DEFAULT_NAME "ESP8266+SHT21 Temp/Humidity"
#define DEFAULT_HOSTNAME "esp8266-temperature"
#define DEFAULT_MODE MODE_CONFIG
#define DEFAULT_SLEEP 60
#define DEFAULT_BATT_LOW 674
#define DEFAULT_BATT_HIGH 766

struct ConfigType {
  String name = DEFAULT_NAME;
  String hostname = DEFAULT_HOSTNAME;
  unsigned short mode = DEFAULT_MODE;
  unsigned int sleepSeconds = DEFAULT_SLEEP;
  unsigned int batteryRawLo = DEFAULT_BATT_LOW;
  unsigned int batteryRawHi = DEFAULT_BATT_HIGH;

  // call after Preferences::begin
  void begin() {
    EEPROM.begin(512);
    name = read_eeprom(0, 64, String(DEFAULT_NAME));
    hostname = read_eeprom(64, 64, String(DEFAULT_HOSTNAME));
    mode = read_eeprom(128, 1, String(DEFAULT_MODE)).toInt();
    sleepSeconds = read_eeprom(129, 4, String(DEFAULT_SLEEP)).toInt();
    batteryRawLo = read_eeprom(133, 3, String(DEFAULT_BATT_LOW)).toInt();
    batteryRawHi = read_eeprom(136, 3, String(DEFAULT_BATT_HIGH)).toInt();
  }

  bool updateFromJson(const JsonDocument& json)
  {
    bool changed = false;

    if (json.containsKey(C_NAME))
    {
      String tmp = json[C_NAME];
      if (!name.equals(tmp))
      {
        write_eeprom(0, 64, tmp);
        name = tmp;
        changed = true;
        Serial.println("Updated name:" + tmp);
      }
    }

    if (json.containsKey(C_HOSTNAME))
    {
      String tmp = json[C_HOSTNAME];
      if (!hostname.equals(tmp))
      {
        write_eeprom(64, 64, tmp);
        hostname = tmp;
        changed = true;
        Serial.println("Updated hostname:" + tmp);
      }
    }

    if (json.containsKey(C_MODE))
    {
      unsigned short tmp = json[C_MODE];
      if (mode != tmp)
      {
        write_eeprom(128, 1, String(tmp));
        mode = tmp;
        changed = true;
        Serial.println("Updated mode:" + String(tmp));
      }
    }

    if (json.containsKey(C_SLEEP))
    {
      unsigned int tmp = json[C_SLEEP];
      if (sleepSeconds != tmp)
      {
        write_eeprom(129, 4, String(tmp));
        sleepSeconds = tmp;
        changed = true;
        Serial.println("Updated sleepSeconds:" + String(tmp));
      }
    }

    if (json.containsKey(C_BATT_L))
    {
      unsigned int tmp = json[C_BATT_L];
      if (batteryRawLo != tmp)
      {
        write_eeprom(133, 3, String(tmp));
        batteryRawLo = tmp;
        changed = true;
        Serial.println("Updated batteryRawLo:" + String(tmp));
      }
    }

    if (json.containsKey(C_BATT_H))
    {
      unsigned int tmp = json[C_BATT_H];
      if (batteryRawHi != tmp)
      {
        write_eeprom(136, 3, String(tmp));
        batteryRawHi = tmp;
        changed = true;
        Serial.println("Updated batteryRawHi:" + String(tmp));
      }
    }

    return changed;
  }

  void getAsJson(JsonDocument& res)
  {
    res[C_NAME] = name;
    res[C_HOSTNAME] = hostname;
    res[C_MODE] = mode;
    res[C_SLEEP] = sleepSeconds;
    res[C_BATT_L] = batteryRawLo;
    res[C_BATT_H] = batteryRawHi;
  }

  String read_eeprom(int offset, int len, String defaultValue)
  {
    char buffer[64];
    char charbuff;
    for (int i = 0; i < len; ++i)
    {
      charbuff = char(EEPROM.read(i + offset));
      buffer[i] = isPrintable(charbuff) ? charbuff : 0;
    }
    String res = buffer;
    return (res.length() > 0) ? res : defaultValue;
  }

  void write_eeprom(int offset, int len, String value)
  {
    for (int i = 0; i < len; ++i)
    {
      if ((unsigned)i < value.length())
      {
        EEPROM.write(i + offset, value[i]);
      }
      else
      {
        EEPROM.write(i + offset, 0);
      }
    }

    EEPROM.commit();
  }
};

#endif
