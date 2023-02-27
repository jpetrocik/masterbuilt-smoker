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
#include <ArduinoOTA.h>

#define SSID "PNET"
#define PASSWORD "5626278472"

#define ADC_SAMPLE_SIZE 20

#define TEMP_SERIES_RESISTOR 46360
#define TEMP_VOLTAGE_DIVIDE 152988000 // 3.3 * TEMP_SERIES_RESISTOR * 1000   
#define PROBE_SERIES_RESISTOR 89000
#define PROBE_VOLTAGE_DIVIDE 293700000 // 3.3 * PROBE_SERIES_RESISTOR * 1000   

#define TEMPERATURE_PIN 34
#define PROBE1_PIN 35
#define PROBE2_PIN 32
#define PROBE3_PIN 33
#define PROBE4_PIN 39
#define HEAT_PIN 5

#define TEMP_A 0.5256707269e-3
#define TEMP_B 2.549879363e-4
#define TEMP_C 0.4157461131e-7

#define PROBE_A 0.8365697887e-3
#define PROBE_B 1.810007004e-4
#define PROBE_C 1.573802637e-7

//#define PROBE_A 0.2895581821e-3
//#define PROBE_B 2.573472783e-4       
//#define PROBE_C -0.6218439951e-7 

#define PID_WINDOW_SIZE 2000

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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

//main loop variables
uint16_t reading;
uint32_t voltage;
uint32_t resistance;

//current state
double temperature = 0;
double probe1 = 0;
double probe2 = 0;
double probe3 = 0;
double probe4 = 0;
double targetTemperature = 0;
long lastRead;
long cookEndTime = 0;

//wifi
long wifiConnecting;
long wsLastDataLog;

//Define Variables we'll be connecting to
double timeOn;

//Specify the links and initial tuning parameters
double Kp = 150, Ki = 0, Kd = 50;
PID heatControlPid(&temperature, &timeOn, &targetTemperature, Kp, Ki, Kd, DIRECT);
bool abortError = false;

unsigned long windowStartTime;

long now;

void setup(void) {
  Serial.begin(115200);

  windowStartTime = millis();

  heatControlPid.SetOutputLimits(0, PID_WINDOW_SIZE);
  heatControlPid.SetMode(MANUAL);

  init_WiFi();

  initWebSocket();

  initSetup();
  
  server.begin();

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(TEMPERATURE_PIN, INPUT);
  pinMode(PROBE1_PIN, INPUT);
}

void loop(void) {
  ArduinoOTA.handle();

  if (abortError) {
    digitalWrite(HEAT_PIN, LOW);
    return;
  }

  now = millis();

  if (now - lastRead > 1000) 
  {
    temperature = readTemperature(TEMPERATURE_PIN, temp_buffer, &temp_index);
//  probe1 = readTemperature(PROBE1_PIN, probe1_buffer, &probe1_index);
//  probe2 = readTemperature(PROBE2_PIN, probe2_buffer, &probe2_index);
//  probe3 = readTemperature(PROBE3_PIN, probe3_buffer, &probe3_index);
    probe4 = readTemperature(PROBE4_PIN, probe4_buffer, &probe4_index);

    lastRead = now;
  }
  
  if (temperature > 120) { //high temp safety
    abortError = true;
    return;
  }
  
  if (targetTemperature < 22 || cookEndTime < now)
  {
    digitalWrite(HEAT_PIN, LOW);
    timeOn = 0;
  }
  else 
  {
    computePID();
  }
  
  if (now - wsLastDataLog > 5000) {
    wsLastDataLog = now;

    JSONVar tempData;
    tempData["temperature"] = (int)toLocalTemperature(temperature);
    tempData["target"] = (int)toLocalTemperature(targetTemperature);
    tempData["cookTime"] = cookEndTime > 0 ? (cookEndTime - now)/60000 : 0;
    tempData["probe1"] = (int)toLocalTemperature(probe1);
    tempData["probe2"] = (int)toLocalTemperature(probe2);
    tempData["probe3"] = (int)toLocalTemperature(probe3);
    tempData["probe4"] = (int)toLocalTemperature(probe4);
    tempData["dutyCycle"] = timeOn/(double)PID_WINDOW_SIZE;

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

  if (timeOn < 100)
    timeOn = 0;
    
  //recalcuate next PID window
  if (now - windowStartTime > PID_WINDOW_SIZE)
    windowStartTime = now;

  if (timeOn > now - windowStartTime)
    digitalWrite(HEAT_PIN, HIGH);
  else
    digitalWrite(HEAT_PIN, LOW);

}

double readTemperature(int pin, uint16_t adc_buffer[], uint8_t* adc_index_ptr) {
  reading = analogRead(pin);
  voltage = readADC_Cal(reading);
  voltage = readADC_Avg(voltage, adc_buffer, adc_index_ptr);
  if (pin == TEMPERATURE_PIN) {
    resistance = (TEMP_VOLTAGE_DIVIDE / voltage) - TEMP_SERIES_RESISTOR;
    return calculate_Temperature_SH_Value(resistance);
  } else { 
    resistance = (PROBE_VOLTAGE_DIVIDE / voltage) - PROBE_SERIES_RESISTOR;
    return calculate_Probe_SH_Value(resistance);
  }
  
}

double toLocalTemperature(double value) {
  return value * 1.8 + 32;
}

double fromLocalTemperature(double value) {
  return (value - 32) / 1.8;
}

float calculate_Temperature_B_Value(uint32_t res) {
  //B value calculation
  float bVale = res / 100000.0;     // (R/Ro)
  bVale = log(bVale);                  // ln(R/Ro)
  bVale /= 4400;                   // 1/B * ln(R/Ro)
  bVale += 1.0 / (25 + 273.15); // + (1/To)
  bVale = 1.0 / bVale;                 // Invert
  bVale -= 273.15;

  return bVale;
}

float calculate_Temperature_SH_Value(uint32_t res) {
  float logRes = log(res);           // Pre-Calcul for Log(R2)
  float temp = (1.0 / (TEMP_A + TEMP_B * logRes + TEMP_C * logRes * logRes * logRes));
  return  temp - 273.15;             // convert Kelvin to *C
}

float calculate_Probe_SH_Value(uint32_t res) {
  float logRes = log(res);           // Pre-Calcul for Log(R2)
  float temp = (1.0 / (PROBE_A + PROBE_B * logRes + PROBE_C * logRes * logRes * logRes));
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
    if (message.indexOf("setTemp=") >= 0) {
      targetTemperature = fromLocalTemperature(message.substring(8).toInt());
      heatControlPid.SetMode(AUTOMATIC);
    }
    if (message.indexOf("setDebugTemp=") >= 0) {
      temperature = fromLocalTemperature(message.substring(13).toInt());
    }
    if (message.indexOf("setKp=") >= 0) {
      Kp = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Ki,Kd);
    }
    if (message.indexOf("setKd=") >= 0) {
      Kd = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Ki,Kd);
    }
    if (message.indexOf("setKi=") >= 0) {
      Ki = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp,Ki,Kd);
    }
    if (message.indexOf("setCookTime=") >= 0) {
      cookEndTime = millis() + (message.substring(12).toInt() * 60000l);
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

void initSetup() { 
  Serial.println("Enabling OTA Updates");

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  ArduinoOTA.setHostname("smoker");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Start....");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Finished");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
