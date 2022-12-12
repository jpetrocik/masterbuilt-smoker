// SPDX-FileCopyrightText: 2011 Limor Fried/ladyada for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// thermistor-1.ino Simple test program for a thermistor for Adafruit Learning System
// https://learn.adafruit.com/thermistor/using-a-thermistor by Limor Fried, Adafruit Industries
// MIT License - please keep attribution and consider buying parts from Adafruit

#include "esp_adc_cal.h"

// the value of the 'other' resistor
#define SERIES_RESISTOR 47000
#define VOLTAGE_DIVIDE 152988000 // (3.3 / SERIESRESISTOR) * 1000   
#define ADC_SAMPLE_SIZE 10

// What pin to connect the sensor to
#define THERMISTOR1_PIN 34
//#define THERMISTOR2_PIN 34
#define HEAT_PIN 5

#define A 0.5256707269e-3
#define B 2.549879363e-4       
#define C  0.4157461131e-7
#define THERMISTOR 100000

#define TEMP_HIGH_LIMIT 76
#define TEMP_LOW_LIMIT 74

uint16_t adc1_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc1_index = 0;
uint16_t adc2_buffer[ADC_SAMPLE_SIZE] = {0};
uint8_t adc2_index = 0;


uint16_t reading;
uint32_t voltage;
uint32_t resistance;
float temperature1;
float temperature2;
float steinhart_constant; 
bool heatOn = false;

void setup(void) {
  Serial.begin(9600);

  steinhart_constant = A + B*log(THERMISTOR);
  
  pinMode(HEAT_PIN,OUTPUT);
  pinMode(THERMISTOR1_PIN,INPUT);
//  pinMode(THERMISTOR2_PIN,INPUT);

//  digitalWrite(HEAT_PIN, HIGH);

}

void loop(void) {

  reading = analogRead(THERMISTOR1_PIN);
  voltage = readADC_Cal(reading);
  resistance = (VOLTAGE_DIVIDE / voltage) - SERIES_RESISTOR;
  temperature1 = calculate_Tempature_SH_Value(resistance);
  Serial.print((temperature1) * 1.8 + 32, 2);
  Serial.print(",");
  
  voltage = readADC_Avg(voltage, adc1_buffer, &adc1_index);
  resistance = (VOLTAGE_DIVIDE / voltage) - SERIES_RESISTOR;
  temperature1 = calculate_Tempature_SH_Value(resistance);
  Serial.print((temperature1) * 1.8 + 32, 2);
  Serial.print(",");

  if (temperature1 > TEMP_HIGH_LIMIT && heatOn) {
    Serial.println(",0");
    digitalWrite(HEAT_PIN, LOW);
    heatOn = false;
   } else if (temperature1 < TEMP_LOW_LIMIT && !heatOn) {
    Serial.println(",1");
    digitalWrite(HEAT_PIN, HIGH);
    heatOn = true;
  } else if (heatOn) {
    Serial.println(",1");
  } else {
    Serial.println(",0");
  }
  delay(1000);
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
  float temp = (1.0 / (A + B*logRes + C*logRes*logRes*logRes)); 
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
