#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#ifdef ENABLE_OTA
#include <ArduinoOTA.h>
#endif // ENABLE_OTA

#include "config.h"
#include "server.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

JsonDocument ws_tempData;
SmokerState *ws_smokerState;
WebSocketEventHandler ws_webSocketEventHandler;
long ws_lastClientNotify = 0;
String ws_jsonStringBuffer;

void ws_initWiFi()
{

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Attempting to connect to ");
    Serial.print(SSID);
    Serial.print(".");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(".");
    Serial.print("Address: ");
    Serial.println(WiFi.localIP().toString());
}

void ws_notifyClients(String body)
{
    Serial.println("Notifying all clients");
    ws.textAll(body);
}

void ws_handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;
        String message = (char *)data;
        Serial.print("Recieved: ");
        Serial.println(message);
        if (message.indexOf("setTemp=") >= 0)
        {
            ws_webSocketEventHandler(WebSocketAction::TEMP, message);
        }
        else if (message.indexOf("setKp=") >= 0)
        {
            ws_webSocketEventHandler(WebSocketAction::KP, message);
        }
        else if (message.indexOf("setKd=") >= 0)
        {
            ws_webSocketEventHandler(WebSocketAction::KD, message);
        }
        else if (message.indexOf("setKi=") >= 0)
        {
        }
        else if (message.indexOf("setCookTime=") >= 0)
        {
            ws_webSocketEventHandler(WebSocketAction::COOKTIME, message);
        }
    }
}

void ws_onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        break;
    case WS_EVT_DISCONNECT:
        break;
    case WS_EVT_DATA:
        ws_handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

void ws_initWebSocket()
{
    ws.onEvent(ws_onEvent);
    server.addHandler(&ws);
}

#ifdef ENABLE_OTA
void ws_initOTAUpdates()
{
    Serial.println("Enabling OTA Updates");

    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    ArduinoOTA.setHostname("smoker");

    // No authentication by default
    // ArduinoOTA.setPassword((const char *)"123");

    ArduinoOTA.onStart([]()
                       { Serial.println("OTA Update Start...."); });
    ArduinoOTA.onEnd([]()
                     { Serial.println("\nOTA Update Finished"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
    ArduinoOTA.begin();
}
#endif // ENABLE_OTA

void ws_init(SmokerState *smokerState, WebSocketEventHandler webSocketEventHandler)
{
    ws_smokerState = smokerState;
    ws_webSocketEventHandler = webSocketEventHandler;

    ws_tempData.clear();
    ws_tempData["temperature"] = 0;
    ws_tempData["targetTemperature"] = 0;
    ws_tempData["cookTimer"] = 0;
    ws_tempData["probe1"] = 0;
    ws_tempData["probe2"] = 0;
    ws_tempData["probe3"] = 0;
    ws_tempData["probe4"] = 0;

    ws_jsonStringBuffer.reserve(512);

    //TODO Call back when connected to WiFi and setup WebSocket
    ws_initWiFi();

    ws_initWebSocket();

#ifdef ENABLE_OTA
    ws_initOTAUpdates();
#endif // ENABLE_OTA

    server.begin();
}

void ws_handle(long now)
{

#ifdef ENABLE_OTA
    ArduinoOTA.handle();
#endif // ENABLE_OTA

    if (now - ws_lastClientNotify > 5000)
    {
        ws_lastClientNotify = now;

        ws_tempData["temperature"] = ws_smokerState->temperature;
        ws_tempData["targetTemperature"] = ws_smokerState->targetTemperature;
        ws_tempData["cookTimer"] = ws_smokerState->cookEndTime > 0 ? (ws_smokerState->cookEndTime - now) / 1000: 0;

        ws_tempData.remove("probe1");
        if (ws_smokerState->probe1 > 0.0)
        {
            ws_tempData["probe1"] = ws_smokerState->probe1;
        }

        ws_tempData.remove("probe2");
        if (ws_smokerState->probe2 > 0.0)
        {
            ws_tempData["probe2"] = ws_smokerState->probe2;
        }

        ws_tempData.remove("probe3");
        if (ws_smokerState->probe3 > 0.0)
        {
            ws_tempData["probe1"] = ws_smokerState->probe3;
        }

        ws_tempData.remove("probe4");
        if (ws_smokerState->probe4 > 0.0)
        {
            ws_tempData["probe4"] = ws_smokerState->probe4;
        }

        serializeJson(ws_tempData, ws_jsonStringBuffer);

        ws_notifyClients(ws_jsonStringBuffer);
    }
}