#include "config.h"

#ifdef MQTT_ENABLED

#ifndef MQTT_H
#define MQTT_H

void mqtt_init(WebSocketEventHandler ws_webSocketEventHandler);
void mqtt_loop();
void mqtt_sendStatus(char* jsonStatusMsg);

#endif  
#endif