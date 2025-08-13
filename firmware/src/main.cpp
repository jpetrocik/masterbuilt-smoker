// SPDX-FileCopyrightText: 2011 Limor Fried/ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// thermistor-1.ino Simple test program for a thermistor for Adafruit Learning System
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit

// #include "driver/adc.h"
#include "esp_adc_cal.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <ArduinoOTA.h>
#include <Adafruit_ADS1X15.h>
#include "interface.h"

#define SSID "PNET"
#define PASSWORD "5626278472"

#define ADC_SAMPLE_SIZE 3

#define TEMP_SERIES_RESISTOR 5400
#define PROBE_SERIES_RESISTOR 10000

// #define TEMPERATURE_PIN ADC1_CHANNEL_6
// #define PROBE1_PIN 35
// #define PROBE2_PIN 32
// #define PROBE3_PIN 33
// #define PROBE4_PIN 39
#define HEAT_PIN 5

#define TEMP_A 1.136646777e-3
#define TEMP_B 1.600914823e-4
#define TEMP_C 3.695194917e-7

#define PROBE_A 0.0006631412844551252
#define PROBE_B 0.0002230253514824875
#define PROBE_C 8.079307552907064E-8

// #define PROBE_A 0.2895581821e-3
// #define PROBE_B 2.573472783e-4
// #define PROBE_C -0.6218439951e-7

#define PID_WINDOW_SIZE 2000

double Kp = 300, Ki = 0.05, Kd = 150;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
String jsonString;
JsonDocument tempData;
bool abortError = false;
int16_t reading;
uint32_t voltage;
uint32_t resistance;
uint32_t probe1_resistance;
uint32_t probe2_resistance;
uint32_t probe3_resistance;
uint32_t probe4_resistance;
uint32_t probe1_voltage;
uint32_t probe2_voltage;
uint32_t probe3_voltage;
uint32_t probe4_voltage;

// current state
double temperature = 0;
double probe1 = 0;
double probe2 = 0;
double probe3 = 0;
double probe4 = 0;
double targetTemperature = 0;
long lastRead;
long cookEndTime = 0;

// wifi
long wifiConnecting;
long wsLastDataLog;

// PID
double timeOn;
unsigned long windowStartTime;
PID heatControlPid(&temperature, &timeOn, &targetTemperature, Kp, Ki, Kd, DIRECT);

// Main loop variables

esp_adc_cal_characteristics_t adc1_chars;

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

  voltage = reading * 187500 / 1000000;

  voltage = readADC_Avg(voltage, adc_buffer, adc_index_ptr);

  // Voltage is 3.3V, extra zeros are for additional precision during int math
  resistance = (series_resistor * (33000000 / voltage - 10000)) / 10000;

  if (reading < 50)
    return 0.0;

  return calculate_Probe_SH_Value(resistance);
}

double toLocalTemperature(double value)
{
  return value * 1.8 + 32;
}

double fromLocalTemperature(double value)
{
  return (value - 32) / 1.8;
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

void init_WiFi()
{

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Attempting to connect to ");
  Serial.print(SSID);
  Serial.print(".");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void notifyClients(String body)
{
  ws.textAll(body);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    String message = (char *)data;
    Serial.print("Recieved: ");
    Serial.println(message);
    if (message.indexOf("setTemp=") >= 0)
    {
      targetTemperature = fromLocalTemperature(message.substring(8).toInt());
      heatControlPid.SetMode(AUTOMATIC);
    }
    if (message.indexOf("setDebugTemp=") >= 0)
    {
      temperature = fromLocalTemperature(message.substring(13).toInt());
    }
    if (message.indexOf("setKp=") >= 0)
    {
      Kp = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp, Ki, Kd);
    }
    if (message.indexOf("setKd=") >= 0)
    {
      Kd = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp, Ki, Kd);
    }
    if (message.indexOf("setKi=") >= 0)
    {
      Ki = message.substring(6).toDouble();
      heatControlPid.SetTunings(Kp, Ki, Kd);
    }
    if (message.indexOf("setCookTime=") >= 0)
    {
      cookEndTime = millis() + (message.substring(12).toInt() * 60000l);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
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

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void initSetup()
{
  Serial.println("Enabling OTA Updates");

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  ArduinoOTA.setHostname("smoker");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]()
                     { Serial.println("OTA Update Start...."); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nOTA Update Finished"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
}

void setup(void)
{
  Serial.begin(115200);

  windowStartTime = millis();

  heatControlPid.SetOutputLimits(0, PID_WINDOW_SIZE);
  heatControlPid.SetMode(MANUAL);

  init_WiFi();

  initWebSocket();

  initSetup();

  server.begin();

  pinMode(HEAT_PIN, OUTPUT);

  if (!ads1.begin())
  {
    Serial.println("Failed to initialize ADS1.");
  }

  if (!ads2.begin(0x49))
  {
    Serial.println("Failed to initialize ADS2.");
  }

#ifdef LCD_SUPPORTED
  init_interface();
#endif // LCD_SUPPORTED
}

void loop(void)
{
  ArduinoOTA.handle();

  if (abortError)
  {
    digitalWrite(HEAT_PIN, LOW);
    return;
  }

  now = millis();

  // 275 (135) Max allowed target temperature
  if (targetTemperature > 135)
    targetTemperature = 135;

  if (now - lastRead > 1000)
  {
    // temperature = readTemperature(&ads1, 0, TEMP_SERIES_RESISTOR, temp_buffer, &temp_index);
    probe1 = readTemperature(&ads1, 0 /*1*/, PROBE_SERIES_RESISTOR, probe1_buffer, &probe1_index);
    probe1_voltage = voltage;
    probe1_resistance = resistance;
    // probe2 = readTemperature(&ads1, 2, PROBE_SERIES_RESISTOR, probe2_buffer, &probe2_index);
    // probe2_voltage = voltage;
    // probe2_resistance = resistance;
    // probe3 = readTemperature(&ads1, 3, PROBE_SERIES_RESISTOR, probe3_buffer, &probe3_index);
    // probe3_voltage = voltage;
    // probe3_resistance = resistance;
    probe4 = readTemperature(&ads2, 0, PROBE_SERIES_RESISTOR, probe4_buffer, &probe4_index);
    probe4_voltage = voltage;
    probe4_resistance = resistance;

    lastRead = now;
  }

  if (targetTemperature < 37 || cookEndTime < now)
  {
    digitalWrite(HEAT_PIN, LOW);
    timeOn = 0;
  }
  else
  {
    computePID();
  }

  if (now - wsLastDataLog > 5000)
  {
    wsLastDataLog = now;

    tempData["temperature"] = (int)toLocalTemperature(temperature);
    tempData["temperature_r"] = -1;
    tempData["target"] = (int)toLocalTemperature(targetTemperature);
    tempData["cookTime"] = cookEndTime > 0 ? (cookEndTime - now) / 60000 : 0;
    tempData.remove("probe1");
    if (probe1 > 0.0)
    {
      tempData["probe1"] = (int)toLocalTemperature(probe1);
      // tempData["probe1_voltage"] = probe1_voltage;
      // tempData["probe1_resistance"] = probe1_resistance;
    }
    // tempData["probe2"] = (int)toLocalTemperature(probe2);
    // tempData["probe2_voltage"] = probe2_voltage;
    // tempData["probe2_resistance"] = probe2_resistance;
    // tempData["probe3"] = (int)toLocalTemperature(probe3);
    // tempData["probe3_voltage"] = probe3_voltage;
    // tempData["probe3_resistance"] = probe3_resistance;
    tempData.remove("probe4");
    if (probe4 > 0.0)
    {
      tempData["probe4"] = (uint32_t)toLocalTemperature(probe4);
      // tempData["probe4_voltage"] = probe4_voltage;
      // tempData["probe4_resistance"] = probe4_resistance;
    }
    // tempData["dutyCycle"] = timeOn/(double)PID_WINDOW_SIZE;

    serializeJson(tempData, jsonString);

    notifyClients(jsonString);

#ifdef LCD_SUPPORTED
    lcd_update_temps((int)toLocalTemperature(temperature), (int)toLocalTemperature(probe1),
                     (int)toLocalTemperature(probe2), (int)toLocalTemperature(probe3),
                     (int)toLocalTemperature(probe4));

    lcd_task_handler();
#endif // LCD_SUPPORTED
  }
}
