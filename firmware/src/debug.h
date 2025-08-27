#include "config.h"

#ifndef DEBUG_H
#define DEBUG_H

#ifdef DEBUG_PROBE
#include <Arduino.h>

void debug_sendProbeDebug(int adsDeviceNumber, int input, uint32_t resistance, uint32_t voltage, double tempature);
void debug_sendPidSettings(double kp, double ki, double kd);

#endif
#endif