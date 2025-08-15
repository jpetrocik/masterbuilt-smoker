#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "config.h"

#ifdef OTA_ENABLED
#include <ArduinoOTA.h>
#endif

#ifdef MQTT_ENABLED
#include "mqtt.h"
#endif

#include "server.h"
#include "status.h"

enum WifiAction
{
    STARTING,
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    STABLE
};

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

status_state *ws_currentState;
WebSocketEventHandler ws_webSocketEventHandler;
long ws_lastClientNotify = 0;

WifiAction ws_wifiAction = WifiAction::STARTING;

void ws_wiFiEvent(WiFiEvent_t event);

void ws_initWiFi()
{

    // Should only be in UNINIT state when first starting up
    if (ws_wifiAction == WifiAction::STARTING)
    {
        WiFi.onEvent(ws_wiFiEvent);
    }

    ws_wifiAction = WifiAction::CONNECTING;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Attempting to connect to ");
    Serial.println(WIFI_SSID);
}

void ws_notifyClients(char *body)
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

#ifdef OTA_ENABLED
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
#endif // OTA_ENABLED

void ws_init(status_state *state, WebSocketEventHandler webSocketEventHandler)
{
    ws_currentState = state;
    ws_webSocketEventHandler = webSocketEventHandler;

    // TODO Call back when connected to WiFi and setup WebSocket
    ws_initWiFi();

    ws_initWebSocket();

#ifdef OTA_ENABLED
    ws_initOTAUpdates();
#endif // OTA_ENABLED

    server.begin();
}

void ws_loop(long now)
{

    if (ws_wifiAction == WifiAction::CONNECTED)
    {
        ws_webSocketEventHandler(WebSocketAction::WIFI_CONNECTED, "");
        ws_wifiAction = WifiAction::STABLE;

#ifdef MQTT_ENABLED
    mqtt_init();
#endif

    }

    if (ws_wifiAction == WifiAction::DISCONNECTED)
    {
        ws_webSocketEventHandler(WebSocketAction::WIFI_DISCONNECTED, "");
        ws_initWiFi();
    }

#ifdef OTA_ENABLED
    ArduinoOTA.handle();
#endif

#ifdef MQTT_ENABLED
    mqtt_loop();
#endif

    if (now - ws_lastClientNotify > 5000)
    {
        ws_lastClientNotify = now;

        char *jsonString = statusJson(ws_currentState);
        ws_notifyClients(jsonString);
#ifdef MQTT_ENABLED
        mqtt_sendStatus(jsonString);
#endif
    }
}

void ws_wiFiEvent(WiFiEvent_t event)
{
    // Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point");
        ws_wifiAction = WifiAction::DISCONNECTED;
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Connected to ");
        Serial.println(WIFI_SSID);
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        ws_wifiAction = WifiAction::CONNECTED;
        break;
    // case ARDUINO_EVENT_WIFI_READY:
    //     Serial.println("WiFi interface ready");
    //     break;
    // case ARDUINO_EVENT_WIFI_SCAN_DONE:
    //     Serial.println("Completed scan for access points");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_START:
    //     Serial.println("WiFi client started");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_STOP:
    //     Serial.println("WiFi clients stopped");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    //     Serial.println("Connected to access point");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
    //     Serial.println("Authentication mode of access point has changed");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    //     Serial.println("Lost IP address and IP address is reset to 0");
    //     break;
    // case ARDUINO_EVENT_WPS_ER_SUCCESS:
    //     Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
    //     break;
    // case ARDUINO_EVENT_WPS_ER_FAILED:
    //     Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
    //     break;
    // case ARDUINO_EVENT_WPS_ER_TIMEOUT:
    //     Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
    //     break;
    // case ARDUINO_EVENT_WPS_ER_PIN:
    //     Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_START:
    //     Serial.println("WiFi access point started");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_STOP:
    //     Serial.println("WiFi access point  stopped");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    //     Serial.println("Client connected");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    //     Serial.println("Client disconnected");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
    //     Serial.println("Assigned IP address to client");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
    //     Serial.println("Received probe request");
    //     break;
    // case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
    //     Serial.println("AP IPv6 is preferred");
    //     break;
    // case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    //     Serial.println("STA IPv6 is preferred");
    //     break;
    // case ARDUINO_EVENT_ETH_GOT_IP6:
    //     Serial.println("Ethernet IPv6 is preferred");
    //     break;
    // case ARDUINO_EVENT_ETH_START:
    //     Serial.println("Ethernet started");
    //     break;
    // case ARDUINO_EVENT_ETH_STOP:
    //     Serial.println("Ethernet stopped");
    //     break;
    // case ARDUINO_EVENT_ETH_CONNECTED:
    //     Serial.println("Ethernet connected");
    //     break;
    // case ARDUINO_EVENT_ETH_DISCONNECTED:
    //     Serial.println("Ethernet disconnected");
    //     break;
    // case ARDUINO_EVENT_ETH_GOT_IP:
    //     Serial.println("Obtained IP address");
    //     break;
    default:
        break;
    }
}