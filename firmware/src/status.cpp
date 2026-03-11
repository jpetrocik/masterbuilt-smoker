#include "status.h"
#include <ArduinoJson.h>
#include "units.h"

JsonDocument status_statusJson;
char status_jsonBuffer[128];

void status_init()
{
    // status_statusJson["temperature"] = 0;
    // status_statusJson["targetTemperature"] = 0;
    // status_statusJson["cookTimer"] = 0; // 24 hours in minutes
    // status_statusJson["probe1"] = 0;
    // status_statusJson["probe2"] = 0;
    // status_statusJson["probe3"] = 0;
    // status_statusJson["probe4"] = 0;
}

char* status_stateJson(status_state *state)
{
    status_statusJson["temperature"] = units_toLocalTemperature(state->temperature);
    status_statusJson["targetTemperature"] = units_toLocalTemperature(state->targetTemperature);
    status_statusJson["cookTimer"] = state->cookEndTime > 0 ? (state->cookEndTime - millis()) / 1000 : 0;
    status_statusJson["cookTime"] = state->cookTime > 0 ? (millis() - state->cookTime) / 1000 : 0;
    status_statusJson["dutyCycle"] = state->dutyCycle;

    status_statusJson.remove("probe1");
    if (state->probe1 > 0.0)
    {
        status_statusJson["probe1"] = units_toLocalTemperature(state->probe1);
    }

    status_statusJson.remove("targetProbe1");
    status_statusJson["targetProbe1"] = units_toLocalTemperature(state->targetProbe1);

    status_statusJson.remove("probe2");
    if (state->probe2 > 0.0)
    {
        status_statusJson["probe2"] = units_toLocalTemperature(state->probe2);
    }

    status_statusJson.remove("targetProbe2");
    status_statusJson["targetProbe2"] = units_toLocalTemperature(state->targetProbe2);

    status_statusJson.remove("probe3");
    if (state->probe3 > 0.0)
    {
        status_statusJson["probe3"] = units_toLocalTemperature(state->probe3);
    }

    status_statusJson.remove("targetProbe3");
    status_statusJson["targetProbe3"] = units_toLocalTemperature(state->targetProbe3);

    status_statusJson.remove("probe4");
    if (state->probe4 > 0.0)
    {
        status_statusJson["probe4"] = units_toLocalTemperature(state->probe4);
    }

    status_statusJson.remove("targetProbe4");
    status_statusJson["targetProbe4"] = units_toLocalTemperature(state->targetProbe4);

    status_statusJson.remove("alarmProbe1");
    status_statusJson["alarmProbe1"] = state->alarmProbe1;

    status_statusJson.remove("alarmProbe2");
    status_statusJson["alarmProbe2"] = state->alarmProbe2;

    status_statusJson.remove("alarmProbe3");
    status_statusJson["alarmProbe3"] = state->alarmProbe3;

    status_statusJson.remove("alarmProbe4");
    status_statusJson["alarmProbe4"] = state->alarmProbe4;

    // Serialize the document to the buffer
    serializeJson(status_statusJson, status_jsonBuffer, sizeof(status_jsonBuffer));
    return status_jsonBuffer;
}

