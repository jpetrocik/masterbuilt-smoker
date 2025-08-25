// SPDX-FileCopyrightText: 2011 Limor Fried/ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// thermistor-1.ino Simple test program for a thermistor for Adafruit Learning System
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit

// #include "driver/adc.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_ADS1X15.h>
#include "config.h"
// #include "esp_adc_cal.h"
#include "server.h"
#include "interface.h"
#include "status.h"
#include "units.h"

double Kp = 300, Ki = 0.05, Kd = 150;

// adc buffers
uint16_t temp_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t temp_index = 0;
uint16_t probe1_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t probe1_index = 0;
uint16_t probe2_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t probe2_index = 0;
uint16_t probe3_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t probe3_index = 0;
uint16_t probe4_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t probe4_index = 0;

// main loop variables
long now;
bool abortError = false;
int16_t reading;
uint32_t voltage;
uint32_t resistance;
status_state currentSmokerState;
long lastProbeRead;

// PID variables
double timeOn;
unsigned long windowStartTime;
PID heatControlPid(&currentSmokerState.temperature, &timeOn, &currentSmokerState.targetTemperature, Kp, Ki, Kd, DIRECT);

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

void computePID()
{
  heatControlPid.Compute();

  if (timeOn < 100)
    timeOn = 0;

  // recalcuate next PID window
  if (now - windowStartTime > PID_WINDOW_SIZE)
    windowStartTime = now;

  if (timeOn > now - windowStartTime)
    digitalWrite(HEAT_PIN, HIGH);
  else
    digitalWrite(HEAT_PIN, LOW);
}

uint32_t readADC_Avg(uint16_t sample, uint16_t adc_buffer[], uint8_t *adc_index_ptr)
{
  int i = 0;
  uint32_t sum = 0;
  uint8_t adc_index = *adc_index_ptr;

  adc_buffer[adc_index++] = sample;
  if (adc_index == ADC_SAMPLE_SIZE)
  {
    adc_index = 0;
  }
  for (i = 0; i < ADC_SAMPLE_SIZE; i++)
  {
    sum += adc_buffer[i];
  }

  *adc_index_ptr = adc_index;

  return sum / ADC_SAMPLE_SIZE;
}

float calculate_Temperature_SH_Value(uint32_t res)
{
  float logRes = log(res); // Pre-Calcul for Log(R2)
  float temp = (1.0 / (TEMP_A + TEMP_B * logRes + TEMP_C * logRes * logRes * logRes));
  return temp - 273.15; // convert Kelvin to *C
}

float calculate_Probe_SH_Value(uint32_t res)
{
  float logRes = log(res); // Pre-Calcul for Log(R2)
  float temp = (1.0 / (PROBE_A + PROBE_B * logRes + PROBE_C * logRes * logRes * logRes));
  return temp - 273.15; // convert Kelvin to *C
}

double readTemperature(Adafruit_ADS1115 *ads, int input, uint16_t series_resistor, uint16_t adc_buffer[], uint8_t *adc_index_ptr)
{
  reading = ads->readADC_SingleEnded(input);

  Serial.println(reading);

  voltage = reading * 187500 / 1000000;

  //TODO Handle missing probe
  if (voltage == 0)
  {
    return 0.0; // avoid division by zero
  }

  voltage = readADC_Avg(voltage, adc_buffer, adc_index_ptr);

  // Voltage is 3.3V, extra zeros are for additional precision during int math
  resistance = (series_resistor * (33000000 / voltage - 10000)) / 10000;

  if (reading < 50)
    return 0.0;

  return calculate_Probe_SH_Value(resistance);
}

float calculate_Temperature_B_Value(uint32_t res)
{
  // B value calculation
  float bVale = res / 100000.0; // (R/Ro)
  bVale = log(bVale);           // ln(R/Ro)
  bVale /= 4400;                // 1/B * ln(R/Ro)
  bVale += 1.0 / (25 + 273.15); // + (1/To)
  bVale = 1.0 / bVale;          // Invert
  bVale -= 273.15;

  return bVale;
}

void handleCommandEvent(char *data)
{

  Serial.print("Recieved: ");
  Serial.println(data);

  if (strncmp(data, "setTemp=", 8) == 0)
  {
    /**
     * When setting the temp to 0, stop cook timer.
     * When setting to temp from 0 to non-zero, start timer
     * Whene setting the temp from non-zero to non-zero, do nothing
     */

    double newRequestedTargetTemp = units_fromLocalTemperature(atoi(&data[8]));
    if (newRequestedTargetTemp == 0.0)
    {
      currentSmokerState.cookTime = 0;
    }
    else if (currentSmokerState.targetTemperature == 0.0)
    {
      currentSmokerState.cookTime = millis();
    }
    currentSmokerState.targetTemperature = newRequestedTargetTemp;
    heatControlPid.SetMode(AUTOMATIC);
  }
  else if (strncmp(data, "setKp=", 6) == 0)
  {
    Kp = atof(&data[6]);
    heatControlPid.SetTunings(Kp, Ki, Kd);
  }
  else if (strncmp(data, "setKd=", 6) == 0)
  {
    Kd = atof(&data[6]);
    heatControlPid.SetTunings(Kp, Ki, Kd);
  }
  else if (strncmp(data, "setKi=", 6) == 0)
  {
    Ki = atof(&data[6]);
    heatControlPid.SetTunings(Kp, Ki, Kd);
  }
  else if (strncmp(data, "setCookTime=", 12) == 0)
  {
    currentSmokerState.cookEndTime = millis() + (atoi(&data[12]) * 60000l);
  }
  #ifdef LCD_SUPPORTED
  else if (strncmp(data, "lcdUpdate", 9) == 0)
  {
    lcd_updateSmokerState();
  }
  else if (strncmp(data, "setProbe1Label=", 15) == 0)
  {
    lcd_setProbeLabel(PROBE1, &data[15]);
  }
  else if (strncmp(data, "setProbe2Label=", 15) == 0)
  {
    lcd_setProbeLabel(PROBE2, &data[15]);
  }
  else if (strncmp(data, "setProbe3Label=", 15) == 0)
  {
    lcd_setProbeLabel(PROBE3, &data[15]);
  }
  else if (strncmp(data, "setProbe4Label=", 15) == 0)
  {
    lcd_setProbeLabel(PROBE4, &data[15]);
  }
  else if (strncmp(data, "wifiConnected", 13) == 0)
  {
    lcd_wifiConnected();
  }
  else if (strncmp(data, "wifiDisconnected", 16) == 0)
  {
    lcd_wifiDisconnected();
  }
  #endif // LCD_SUPPORTED
}

void setup(void)
{
  Serial.begin(115200);

  status_init();

  windowStartTime = millis();

  heatControlPid.SetOutputLimits(0, PID_WINDOW_SIZE);
  heatControlPid.SetMode(MANUAL);

  ws_init(&currentSmokerState, handleCommandEvent);

  pinMode(HEAT_PIN, OUTPUT);

  if (!ads1.begin())
  {
    Serial.println("Failed to initialize ADS1.");
    abortError = true;
  }

  if (!ads2.begin(0x49))
  {
    Serial.println("Failed to initialize ADS2.");
    abortError = true;
  }

  // TODO: perform sanity check on temp probe

#ifdef LCD_SUPPORTED
  lcd_init(&currentSmokerState);
#endif // LCD_SUPPORTED
}

void loop(void)
{

  if (abortError)
  {
    Serial.println("Abort error!");
    digitalWrite(HEAT_PIN, LOW);
    return;
  }

  now = millis();

  // Checks for allowed max target temperature 135C (275F)
  if (currentSmokerState.targetTemperature > MAX_TEMP)
  {
    Serial.println("Target temperature too high, turning off heating.");
    currentSmokerState.targetTemperature = 0;
    digitalWrite(HEAT_PIN, LOW);
  }

  // Checks for allowed min target temperature 37C (98.6F)
  if (currentSmokerState.targetTemperature < MIN_TEMP)
  {
    // Serial.println("Target temperature too low, turning off heating.");
    currentSmokerState.targetTemperature = 0;
    digitalWrite(HEAT_PIN, LOW);
  }

  // Checks for cook time exceeded and reset target temperature
  if (currentSmokerState.cookEndTime > 0 && currentSmokerState.cookEndTime < now)
  {
    Serial.println("Cook time exceeded, turning off heating.");
    currentSmokerState.cookEndTime = 0;
    currentSmokerState.targetTemperature = 0;
    digitalWrite(HEAT_PIN, LOW);
    timeOn = 0;
  }

  // Reads probes every second
  if (now - lastProbeRead > 1000)
  {
    currentSmokerState.temperature = readTemperature(&ads2, 1, TEMP_SERIES_RESISTOR, temp_buffer, &temp_index);
    currentSmokerState.probe1 = readTemperature(&ads1, 0, PROBE_SERIES_RESISTOR, probe1_buffer, &probe1_index);
    currentSmokerState.probe2 = readTemperature(&ads1, 1, PROBE_SERIES_RESISTOR, probe2_buffer, &probe2_index);
    currentSmokerState.probe3 = readTemperature(&ads1, 2, PROBE_SERIES_RESISTOR, probe3_buffer, &probe3_index);
    currentSmokerState.probe4 = readTemperature(&ads1, 3, PROBE_SERIES_RESISTOR, probe4_buffer, &probe4_index);

    lastProbeRead = now;
  }

  if (currentSmokerState.targetTemperature > MIN_TEMP)
  {
    computePID();
  }

  ws_loop(now);

#ifdef LCD_SUPPORTED
  lcd_loop();
#endif // LCD_SUPPORTED
}
