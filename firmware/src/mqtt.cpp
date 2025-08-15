#include "config.h"

#ifdef MQTT_ENABLED
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient mqtt_wifiClient;
PubSubClient mqtt_client(mqtt_wifiClient);

int mqtt_reconnectAttemptCounter = 0;
long mqtt_nextReconnectAttempt = 0;

char clientId[25];

// callback when a mqtt message is recieved
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    //TODO use same callback from server.cpp
}

void mqtt_setup()
{
    if (strlen(MQTT_SERVER) == 0)
        return;

    uint64_t mac = ESP.getEfuseMac();
    uint32_t chipId = ((mac >> 40) & 0xFF) | (((mac >> 32) & 0xFF) << 8);
    snprintf(clientId, sizeof(clientId), "%s%d", MQTT_CLIENT_ID_PREFIX, chipId);

    Serial.println("Connecting to MQTT Server....");
    mqtt_client.setServer(MQTT_SERVER, 1883);
    mqtt_client.setCallback(mqtt_callback);
    //  mqtt_client.setKeepAlive(120);

}

void mqtt_connect()
{
    if (!mqtt_client.connected() && mqtt_nextReconnectAttempt < millis())
    {

        if (mqtt_client.connect(clientId))
        {
            Serial.println("Connected to MQTT Server");
            mqtt_client.subscribe(MQTT_SUBSCRIBE_TOPIC);

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

void mqtt_sendStatus()
{
    if (mqtt_client.connected())
    {
        //TODO construct same json as server.cpp
        mqtt_client.publish(MQTT_PUBLISH_TOPIC, (char *)jsonStatusMsg);
    }
}
#endif