#include "config.h"

#ifdef DEBUG_PROBE
#include <Arduino.h>
#include <ArduinoJson.h>
#include <mqtt.h>

JsonDocument deubg_probeJson;
char debug_jsonBuffer[128];

char mqtt_debugTopic[25] = "";

char *debug_ProbeJson(int deviceNumber, int input, uint32_t resistance, uint32_t voltage, double tempature)
{
    deubg_probeJson["device"] = deviceNumber;
    deubg_probeJson["input"] = input;
    deubg_probeJson["resistance"] = resistance;
    deubg_probeJson["voltage"] = voltage;
    deubg_probeJson["temperature"] = tempature;

    // Serialize the document to the buffer
    serializeJson(deubg_probeJson, debug_jsonBuffer, 127);
    return debug_jsonBuffer;
}

void debug_sendProbeDebug(int deviceNumber, int input, uint32_t resistance, uint32_t voltage, double temperature)
{
    if (strlen(mqtt_debugTopic) == 0)
    {
        sprintf(mqtt_debugTopic, MQTT_DEBUG_TOPIC, mqtt_chipId());
    }

    char *jsonString = debug_ProbeJson(deviceNumber, input, resistance, voltage, temperature);
#ifdef MQTT_ENABLED
    mqtt_sendMessage(mqtt_debugTopic, jsonString);
#endif
}
#endif // DEBUG_PROBE