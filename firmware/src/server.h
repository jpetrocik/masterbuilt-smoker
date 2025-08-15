#ifndef SERVER_H
#define SERVER_H

#include "status.h"

enum WebSocketAction
{
    TEMP,
    COOKTIME,
    KP,
    KD,
    KI,
    WIFI_CONNECTED,
    WIFI_DISCONNECTED
};

typedef void (*WebSocketEventHandler)(WebSocketAction action, String message);

void ws_init(status_state *state, WebSocketEventHandler webSocketEventHandler);
void ws_loop(long now);

#endif // SERVER_H
