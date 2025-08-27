#ifndef SERVER_H
#define SERVER_H

#include "status.h"

enum CommandAction
{
    TEMP,
    COOKTIME,
    KP,
    KD,
    KI,
    WIFI_CONNECTED,
    WIFI_DISCONNECTED,
    PROBE1_LABEL,
    PROBE2_LABEL,
    PROBE3_LABEL,
    PROBE4_LABEL,
};

typedef void (*CommandEventHandler)(char* data);

void ws_init(status_state *state, CommandEventHandler commandEventHandler);
void ws_loop(long now);


#endif // SERVER_H
