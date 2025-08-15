#ifndef STATUS_H
#define STATUS_H

// current state
struct status_state {
  double temperature = 0;
  double probe1 = 0;
  double probe2 = 0;
  double probe3 = 0;
  double probe4 = 0;
  double targetTemperature = 0;
  long cookEndTime = 0;
  long cookTime = 0;
}; 

void status_init();
char* statusJson(status_state *state);

#endif