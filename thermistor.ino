// SPDX-FileCopyrightText: 2011 Limor Fried/ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// thermistor-1.ino Simple test program for a thermistor for Adafruit Learning System
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit

#include "esp_adc_cal.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <PID_v1.h>

#define SSID "PNET"
#define PASSWORD "5626278472"

#define SERIES_RESISTOR 47000
#define VOLTAGE_DIVIDE 152988000 // (3.3 / SERIESRESISTOR) * 1000   
#define ADC_SAMPLE_SIZE 100

#define TEMPERATURE_PIN 34
#define THERMISTOR1_PIN 35
#define THERMISTOR2_PIN 32
#define THERMISTOR3_PIN 33
#define THERMISTOR4_PIN 39
#define HEAT_PIN 5

#define THERMISTOR 100000
#define A 0.5256707269e-3
#define B 2.549879363e-4
#define C 0.4157461131e-7

#define PID_WINDOW_SIZE 30000

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

uint16_t adc1_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc1_index = 0;
uint16_t adc2_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc2_index = 0;
uint16_t adc3_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc3_index = 0;
uint16_t adc4_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc4_index = 0;

//main loop variables
uint16_t reading;
uint32_t voltage;
uint32_t resistance;

//current state
double temperature;
double probe1;
double probe2;
double probe3;
double probe4;
double targetTemperature = 0;


//wifi
long wifiConnecting;
long wsTempLogLastLog;

//Define Variables we'll be connecting to
double timeOn;

//Specify the links and initial tuning parameters
bool manualWarming = true;
double Kp = 1, Ki = 0, Kd = 50;
PID heatControlPid(&temperature, &timeOn, &targetTemperature, Kp, Ki, Kd, DIRECT);

unsigned long windowStartTime;

long now;

void setup(void) {
  Serial.begin(9600);

  windowStartTime = millis();

  heatControlPid.SetOutputLimits(0, PID_WINDOW_SIZE);
  heatControlPid.SetMode(MANUAL);

  init_WiFi();

  initWebSocket();

  server.begin();

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(THERMISTOR1_PIN, INPUT);
}

void loop(void) {
  now = millis();
  
  temperature = readTemperature(TEMPERATURE_PIN);  //need to pass buffer array

//  probe1 = readTemperature(THERMISTOR1_PIN);
//  probe2 = readTemperature(THERMISTOR2_PIN);
//  probe3 = readTemperature(THERMISTOR3_PIN);
//  probe4 = readTemperature(THERMISTOR4_PIN);
  

  if (targetTemperature < 38)
  {
    digitalWrite(HEAT_PIN, LOW);
    timeOn = 0;
  }
  else if (manualWarming) 
  {
    if (targetTemperature - temperature <  16) {
      manualWarming = false;
      heatControlPid.SetMode(AUTOMATIC);
    } else {
      digitalWrite(HEAT_PIN, HIGH);
      timeOn = PID_WINDOW_SIZE;
    }
  }
  else 
  {
    computePID();
  }
  
  if (now - wsTempLogLastLog > 5000) {
    wsTempLogLastLog = now;

    JSONVar tempData;
    tempData["temperature"] = (int)toLocalTemperature(temperature);
    tempData["target"] = (int)toLocalTemperature(targetTemperature);
    tempData["heat"] = digitalRead(HEAT_PIN);
    tempData["probe1"] = (int)toLocalTemperature(probe1);
    tempData["probe2"] = (int)toLocalTemperature(probe2);
    tempData["probe3"] = (int)toLocalTemperature(probe3);
    tempData["probe4"] = (int)toLocalTemperature(probe4);
    tempData["dutyCycle"] = ((int)(timeOn*100/PID_WINDOW_SIZE))/100.0;

    String jsonString = JSON.stringify(tempData);
    notifyClients(jsonString);

    Serial.print("Time on ");
    Serial.println(timeOn);

    Serial.print("Window ");
    Serial.println(PID_WINDOW_SIZE - (now - windowStartTime));

  }
}

void computePID() {
  heatControlPid.Compute();

  if (timeOn < 5)
    timeOn = 0;
    
  //recalcuate next PID window
  if (now - windowStartTime > PID_WINDOW_SIZE)
    windowStartTime = now;

  if (timeOn > now - windowStartTime)
    digitalWrite(HEAT_PIN, HIGH);
  else
    digitalWrite(HEAT_PIN, LOW);

}
double readTemperature(int pin) {
  reading = analogRead(pin);
  voltage = readADC_Cal(reading);
  voltage = readADC_Avg(voltage, adc1_buffer, &adc1_index);
  resistance = (VOLTAGE_DIVIDE / voltage) - SERIES_RESISTOR;
  return calculate_Tempature_SH_Value(resistance);
}

double toLocalTemperature(double value) {
  return value * 1.8 + 32;
}

double fromLocalTemperature(double value) {
  return (value - 32) / 1.8;
}


float calculate_Tempature_B_Value(uint32_t res) {
  //B value calculation
  float bVale = res / 100000.0;     // (R/Ro)
  bVale = log(bVale);                  // ln(R/Ro)
  bVale /= 4400;                   // 1/B * ln(R/Ro)
  bVale += 1.0 / (25 + 273.15); // + (1/To)
  bVale = 1.0 / bVale;                 // Invert
  bVale -= 273.15;

  return bVale;
}

float calculate_Tempature_SH_Value(uint32_t res) {
  float logRes = log(res);           // Pre-Calcul for Log(R2)
  float temp = (1.0 / (A + B * logRes + C * logRes * logRes * logRes));
  return  temp - 273.15;             // convert Kelvin to *C
}

uint32_t readADC_Cal(int16_t ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

uint32_t readADC_Avg(uint16_t sample, uint16_t adc_buffer[], uint8_t* adc_index_ptr)
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

void init_WiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Attempting to connect to ");
  Serial.print(SSID);
  Serial.print(".");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    String message = (char*)data;
    Serial.print("Recieved: ");
    Serial.println(message);
    if (message.indexOf("setTemp") >= 0) {
      targetTemperature = fromLocalTemperature(message.substring(8).toInt());
      heatControlPid.SetMode(MANUAL);
      manualWarming = true;
    }
    if (message.indexOf("setDebugTemp") >= 0) {
      temperature = fromLocalTemperature(message.substring(13).toInt());
    }
    if (message.indexOf("setKp") >= 0) {
      Kp = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Kd,Ki);
    }
    if (message.indexOf("setKd") >= 0) {
      Kd = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Kd,Ki);
    }
    if (message.indexOf("setKi") >= 0) {
      Ki = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Kd,Ki);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      break;
    case WS_EVT_DISCONNECT:
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
