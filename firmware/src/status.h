#ifndef STATUS_H
#define STATUS_H

#include <Arduino.h>

// current state
struct status_state {
  double temperature = 0;
  double targetTemperature = 0;
  double probe1 = 0;
  double targetProbe1 = 0;
  bool alarmProbe1 = 0;
  double probe2 = 0;
  double targetProbe2 = 0;
  bool alarmProbe2 = 0;
  double probe3 = 0;
  double targetProbe3 = 0;
  bool alarmProbe3 = 0;
  double probe4 = 0;
  double targetProbe4 = 0;
  bool alarmProbe4 = 0;
  long cookEndTime = 0;
  long cookTime = 0;
  double dutyCycle = 0;
}; 

void status_init();
char* status_stateJson(status_state *state);

#endif