#include "config.h"
#include "server.h"

#ifdef MQTT_ENABLED

#ifndef MQTT_H
#define MQTT_H

void mqtt_init(CommandEventHandler ws_commandEventHandler);
void mqtt_loop();
void mqtt_sendStatus(char* jsonStatusMsg);

#endif  
#endif