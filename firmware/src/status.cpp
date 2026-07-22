#include "status.h"
#include <ArduinoJson.h>
#include "units.h"

JsonDocument status_statusJson;
char status_jsonBuffer[512];

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
    status_statusJson["smokerTemperature"] = units_toLocalTemperature(state->temperature);
    status_statusJson["smokerTarget"] = units_toLocalTemperature(state->targetTemperature);
    status_statusJson["cookTimer"] = state->cookEndTime > 0 ? (state->cookEndTime - millis()) / 1000 : 0;
    status_statusJson["cookTime"] = state->cookStartTime > 0 ? (millis() - state->cookStartTime) / 1000 : 0;
    status_statusJson["dutyCycle"] = state->dutyCycle;

    status_statusJson.remove("probe1Temperature");
    if (state->probe1 > 0.0)
    {
        status_statusJson["probe1Temperature"] = units_toLocalTemperature(state->probe1);
    }

    status_statusJson.remove("probe1Target");
    status_statusJson["probe1Target"] = units_toLocalTemperature(state->targetProbe1);

    status_statusJson.remove("probe2Temperature");
    if (state->probe2 > 0.0)
    {
        status_statusJson["probe2Temperature"] = units_toLocalTemperature(state->probe2);
    }

    status_statusJson.remove("probe2Target");
    status_statusJson["probe2Target"] = units_toLocalTemperature(state->targetProbe2);

    status_statusJson.remove("probe3Temperature");
    if (state->probe3 > 0.0)
    {
        status_statusJson["probe3Temperature"] = units_toLocalTemperature(state->probe3);
    }

    status_statusJson.remove("probe3Target");
    status_statusJson["probe3Target"] = units_toLocalTemperature(state->targetProbe3);

    status_statusJson.remove("probe4Temperature");
    if (state->probe4 > 0.0)
    {
        status_statusJson["probe4Temperature"] = units_toLocalTemperature(state->probe4);
    }

    status_statusJson.remove("probe4Target");
    status_statusJson["probe4Target"] = units_toLocalTemperature(state->targetProbe4);

    status_statusJson.remove("probe1Alarm");
    status_statusJson["probe1Alarm"] = state->alarmProbe1;

    status_statusJson.remove("probe2Alarm");
    status_statusJson["probe2Alarm"] = state->alarmProbe2;

    status_statusJson.remove("probe3Alarm");
    status_statusJson["probe3Alarm"] = state->alarmProbe3;

    status_statusJson.remove("probe4Alarm");
    status_statusJson["probe4Alarm"] = state->alarmProbe4;

    // Serialize the document to the buffer
    serializeJson(status_statusJson, status_jsonBuffer, sizeof(status_jsonBuffer));
    return status_jsonBuffer;
}

