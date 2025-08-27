#include "config.h"

#ifdef MQTT_ENABLED
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "server.h"

WiFiClient mqtt_wifiClient;
PubSubClient mqtt_client(mqtt_wifiClient);
char *mqtt_statusTopic;
char *mqtt_commandTopic;
bool mqtt_inited = false;

int mqtt_reconnectAttemptCounter = 0;
long mqtt_nextReconnectAttempt = 0;

char clientId[25];

CommandEventHandler mqtt_commandEventHandler;

uint32_t mqtt_chipId()
{
    uint64_t mac = ESP.getEfuseMac();
    uint32_t chipId = ((mac >> 40) & 0xFF) | (((mac >> 32) & 0xFF) << 8);
    return chipId;
}

// callback when a mqtt message is recieved
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    payload[length] = '\0';
    mqtt_commandEventHandler((char *)payload);
}

void mqtt_init(CommandEventHandler commandEventHandler)
{
    if (strlen(MQTT_SERVER) == 0 || mqtt_inited)
        return;

    mqtt_inited = true;

    mqtt_commandEventHandler = commandEventHandler;

    uint32_t chipId = mqtt_chipId();

    snprintf(clientId, sizeof(clientId), "%s%d", MQTT_CLIENT_ID_PREFIX, chipId);

    mqtt_statusTopic = (char *)malloc(strlen(MQTT_STATUS_TOPIC) + 5 /*chipId*/ + 1 /*null*/);
    sprintf(mqtt_statusTopic, MQTT_STATUS_TOPIC, chipId);

    mqtt_commandTopic = (char *)malloc(strlen(MQTT_COMMAND_TOPIC) + 5 /*chipId*/ + 1) /*null*/;
    sprintf(mqtt_commandTopic, MQTT_COMMAND_TOPIC, chipId);

    Serial.println("Connecting to MQTT Server....");
    mqtt_client.setServer(MQTT_SERVER, 1883);
    mqtt_client.setCallback(mqtt_callback);
    // mqtt_connect();
    //  mqtt_client.setKeepAlive(120);
}

void mqtt_connect()
{
    if (!mqtt_client.connected() && mqtt_nextReconnectAttempt < millis())
    {

        if (mqtt_client.connect(clientId, MQTT_DEVICE_TOPIC, 0, true, ""))
        {
            Serial.println("Connected to MQTT Server");
            mqtt_client.subscribe(mqtt_commandTopic);

            char buffer[100];
            sprintf(buffer, "{\"chipId\":%i, \"ipAddress\":\"%s\"}", mqtt_chipId(), WiFi.localIP().toString());
            mqtt_client.publish(MQTT_DEVICE_TOPIC, buffer, true);

            mqtt_reconnectAttemptCounter = 0;
            mqtt_nextReconnectAttempt = 0;
        }
        else
        {
            Serial.print("Failed to connect to ");
            Serial.println(MQTT_SERVER);

            mqtt_reconnectAttemptCounter++;
            mqtt_nextReconnectAttempt = sq(mqtt_reconnectAttemptCounter) * 1000;
            if (mqtt_nextReconnectAttempt > 30000)
                mqtt_nextReconnectAttempt = 30000;

            Serial.print("Will reattempt to connect in ");
            Serial.print(mqtt_nextReconnectAttempt);
            Serial.println(" seconds");

            mqtt_nextReconnectAttempt += millis();
        }
    }
}

void mqtt_loop()
{
    if (!mqtt_client.connected())
    {
        mqtt_connect();
    }
    else
    {
        mqtt_client.loop();
    }
}

void mqtt_sendStatus(const char *jsonStatusMsg)
{
    if (mqtt_client.connected())
    {
        mqtt_client.publish(mqtt_statusTopic, (char *)jsonStatusMsg, true);
    }
}

void mqtt_sendMessage(const char *topic, const char *jsonStatusMsg)
{
    if (mqtt_client.connected())
    {
        mqtt_client.publish(topic, jsonStatusMsg);
    }
}

#endif