/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/
#include <PID_v1.h>

void pip_initialize(pid_c pid);

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void pip_setup(pid_c pid, double input, double output, double setpoint,
        double kp, double ki, double kd, int pon, int direction)
{ 
    pid.output = output;
    pid.input = input;
    pid.setpoint = setpoint;
    pid.mode = MANUAL;

    pip_set_output_limits(pid, 0, 255);				//default output limit corresponds to
												//the arduino pwm limits

    pid.sample_time = 100;							//default Controller Sample Time is 0.1 seconds

    pip_set_direction(pid, direction);
    pip_set_tunings(pid, kp, ki, kd, pon);

    pid.last_time = millis() - pid.sample_time;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool pip_compute(pid_c pid)
{
   if(pid.mode == MANUAL)
    return false;

   unsigned long now = millis();
   unsigned long timeChange = (now - pid.last_time);
   if(timeChange >= pid.sample_time)
   {
      /*Compute all the working error variables*/
      double input = pid.input;
      double error = pid.setpoint - input;
      double dInput = (input - pid.last_input);
      pid.output_sum += (pid.ki * error);

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      if(!pid.pOnE) 
         pid.output_sum -= pid.kp * dInput;

      if(pid.output_sum > pid.max) 
       pid.output_sum = pid.max;
      else if(pid.output_sum < pid.min) 
         pid.output_sum = pid.min;

      /*Add Proportional on Error, if P_ON_E is specified*/
	   double output;
      if(pid.pOnE) 
         output = pid.kp * error;
      else 
         output = 0;

      /*Compute Rest of PID Output*/
      output += pid.output_sum - pid.kd * dInput;

	   if(output > pid.max) 
         output = pid.max;
      else if(output < pid.min) 
         output = pid.min;
	    pid.output = output;

      /*Remember some variables for next time*/
      pid.last_input = input;
      pid.last_time = now;
	   return true;
   }
   else 
      return false;
}

/* pip_set_tunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void pip_set_tunings(pid_c pid, double kp, double ki, double kd, int POn)
{
   if (kp < 0 || ki < 0 || kd < 0) return;

   pid.pOn = POn;
   pid.pOnE = POn == P_ON_E;

   pid.original_kp = kp; 
   pid.original_ki = ki; 
   pid.original_kd = kd;

   double SampleTimeInSec = ((double)pid.sample_time)/1000;
   pid.kp = kp;
   pid.ki = ki * SampleTimeInSec;
   pid.kd = kd / SampleTimeInSec;

  if(pid.direction == REVERSE)
   {
      pid.kp = (0 - pid.kp);
      pid.ki = (0 - pid.ki);
      pid.kd = (0 - pid.kd);
   }
}

/* pip_set_sample_time(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void pip_set_sample_time(pid_c pid, int sample_time)
{
   if (sample_time > 0)
   {
      double ratio  = (double)sample_time
                      / (double)pid.sample_time;
      pid.ki *= ratio;
      pid.kd /= ratio;
      pid.sample_time = (unsigned long)sample_time;
   }
}

/* pip_set_output_limits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void pip_set_output_limits(pid_c pid, double Min, double Max)
{
   if(Min >= Max) return;
   pid.min = Min;
   pid.max = Max;

   if(pid.mode == AUTOMATIC)
   {
	   if(pid.output > pid.max) 
         pid.output = pid.max;
	   else if(pid.output < pid.min) 
         pid.output = pid.min;

	   if(pid.output_sum > pid.max) 
         pid.output_sum = pid.max;
	   else if(pid.output_sum < pid.min) 
         pid.output_sum = pid.min;
   }
}

/* pip_set_mode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void pip_set_mode(pid_c pid, pid_mode_t mode)
{
    if(mode == AUTOMATIC && pid.mode == MANUAL)
        pip_initialize(pid);
    
    pid.mode = mode;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void pip_initialize(pid_c pid)
{
   pid.output_sum = pid.output;
   pid.last_input = pid.input;
   if(pid.output_sum > pid.max) 
      pid.output_sum = pid.max;
   else if(pid.output_sum < pid.min) 
      pid.output_sum = pid.min;
}

/* pip_set_directions(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pip_set_directions(pid_c pid, pid_direction_t direction)
{
   if(pid.mode == AUTOMATIC && direction != pid.direction)
   {
	   pid.kp = (0 - pid.kp);
      pid.ki = (0 - pid.ki);
      pid.kd = (0 - pid.kd);
   }
   pid.direction = direction;
}
