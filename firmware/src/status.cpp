#include "status.h"
#include <ArduinoJson.h>
#include "units.h"

JsonDocument status_statusJson;
char *status_jsonBuffer;

void status_init() {
    // status_statusJson.clear();

    status_statusJson["temperature"] = 0;
    status_statusJson["targetTemperature"] = 0;
    status_statusJson["cookTimer"] = 0; // 24 hours in minutes
    status_statusJson["probe1"] = 0;
    status_statusJson["probe2"] = 0;
    status_statusJson["probe3"] = 0;
    status_statusJson["probe4"] = 0;

    status_jsonBuffer = (char*)malloc(128 + 1);
}

char* statusJson(status_state *state)
{

    status_statusJson["temperature"] = units_toLocalTemperature(state->temperature);
    status_statusJson["targetTemperature"] = units_toLocalTemperature(state->targetTemperature);
    status_statusJson["cookTimer"] = state->cookEndTime > 0 ? (state->cookEndTime - millis()) / 1000 : 0;

    status_statusJson.remove("probe1");
    if (state->probe1 > 0.0)
    {
        status_statusJson["probe1"] = units_toLocalTemperature(state->probe1);
    }

    status_statusJson.remove("probe2");
    if (state->probe2 > 0.0)
    {
        status_statusJson["probe2"] = units_toLocalTemperature(state->probe2);
    }

    status_statusJson.remove("probe3");
    if (state->probe3 > 0.0)
    {
        status_statusJson["probe3"] = units_toLocalTemperature(state->probe3);
    }

    status_statusJson.remove("probe4");
    if (state->probe4 > 0.0)
    {
        status_statusJson["probe4"] = units_toLocalTemperature(state->probe4);
    }

    // Serialize the document to the buffer
    serializeJson(status_statusJson, status_jsonBuffer, 128);
    return status_jsonBuffer;
}