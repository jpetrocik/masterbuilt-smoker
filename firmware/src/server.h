#ifndef SERVER_H
#define SERVER_H

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

void ws_init(SmokerState *smokerState, WebSocketEventHandler webSocketEventHandler);
void ws_handle(long now);

#endif // SERVER_H
