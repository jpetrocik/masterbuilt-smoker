#include "config.h"

#ifdef DEBUG_PROBE
#include <Arduino.h>
#include <ArduinoJson.h>
#include <mqtt.h>

JsonDocument deubg_Json;
char debug_jsonBuffer[128];

#ifdef MQTT_ENABLED
char mqtt_debugTopic[25] = "";
#endif

void debug_sendProbeDebug(int deviceNumber, int input, uint32_t resistance, uint32_t voltage, double temperature)
{

    deubg_Json.clear();
    deubg_Json["device"] = deviceNumber;
    deubg_Json["input"] = input;
    deubg_Json["resistance"] = resistance;
    deubg_Json["voltage"] = voltage;
    deubg_Json["temperature"] = temperature;
    serializeJson(deubg_Json, debug_jsonBuffer, 127);

    Serial.println(debug_jsonBuffer);

#ifdef MQTT_ENABLED
    if (strlen(mqtt_debugTopic) == 0)
    {
        sprintf(mqtt_debugTopic, MQTT_DEBUG_TOPIC, mqtt_chipId());
    }

    mqtt_sendMessage(mqtt_debugTopic, debug_jsonBuffer, false);
#endif,
}

void debug_sendPidSettings(double kp, double ki, double kd)
{

    deubg_Json.clear();
    deubg_Json["kp"] = kp;
    deubg_Json["ki"] = ki;
    deubg_Json["kd"] = kd;
    serializeJson(deubg_Json, debug_jsonBuffer, 127);

    Serial.println(debug_jsonBuffer);

#ifdef MQTT_ENABLED
    if (strlen(mqtt_debugTopic) == 0)
    {
        sprintf(mqtt_debugTopic, MQTT_DEBUG_TOPIC, mqtt_chipId());
    }

    mqtt_sendMessage(mqtt_debugTopic, debug_jsonBuffer, true);
#endif
}

#endif // DEBUG_PROBE