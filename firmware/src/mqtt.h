#include "config.h"
#include "server.h"

#ifdef MQTT_ENABLED

#ifndef MQTT_H
#define MQTT_H

void mqtt_init(CommandEventHandler ws_commandEventHandler);
void mqtt_loop();
void mqtt_sendStatus(const char* jsonStatusMsg);
void mqtt_sendMessage(const char* topic, const char *jsonStatusMsg, boolean retain);
uint32_t mqtt_chipId();

#endif  
#endif