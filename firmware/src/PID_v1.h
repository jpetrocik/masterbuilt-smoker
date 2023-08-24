#ifndef PID_v1_h
#define PID_v1_h

#include <stdbool.h>
#include "esp_timer.h"

#define millis()  esp_timer_get_time()/1000

typedef enum {
    MANUAL,
    AUTOMATIC,
} pid_mode_t;

typedef enum {
    DIRECT,
    REVERSE,
} pid_direction_t;

typedef enum {
    P_ON_M,
    P_ON_E,
} pid_proportional_mode_t;

typedef struct {
  double original_kp;
  double original_ki;
  double original_kd;
  
  double kp;
  double ki;
  double kd;
  pid_direction_t direction;
  pid_proportional_mode_t pOn;
  double input;
  double output;
  double setpoint;
  unsigned long last_time;
  double output_sum;
  double last_input;
  unsigned long sample_time;
  double min;
  double max;
  pid_mode_t mode;
  bool pOnE;
} pid_c;

void pip_etup(pid_c, double input, double output, double setpoint,
      double kp, double ki, double kd, int pon, int direction);

/*
  * sets PID to either Manual (0) or Auto (non-0)
  */
void pip_set_mode(pid_c, pid_mode_t mode);

/*
  * performs the PID calculation.  it should be
  * called every time loop() cycles. ON/OFF and
  * calculation frequency can be set using SetMode
  *  SetSampleTime respectively
  */
bool pip_compute(pid_c);

/*
  * clamps the output to a specific range. 0-255 by default, but
  * it's likely the user will want to change this depending on
  * the application  
  */
void pip_set_output_limits(pid_c, double min, double max);


/* 
  * overload for specifying proportional mode
  */
void pip_set_tunings(pid_c, double kp, double ki, double kd, int pon);         	  

/*
  * Sets the Direction, or "Action" of the controller. DIRECT
  * means the output will increase when error is positive. REVERSE
  * means the opposite.  it's very unlikely that this will be needed
  * once it is set in the constructor.
  */
void pip_set_direction(pid_c pip, pid_direction_t direction);

/*
* sets the frequency, in Milliseconds, with which 
* the PID calculation is performed.  default is 100
*/
void pip_set_sample_time(pid_c, int sampleTime); 

#endif

