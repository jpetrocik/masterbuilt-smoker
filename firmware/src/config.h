#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID "PNET"
#define WIFI_PASSWORD "5626278472"

//#define LCD_SUPPORTED  // Uncomment to enable LCD support
#define OTA_ENABLED

#define ADC_SAMPLE_SIZE 10

#define TEMP_SERIES_RESISTOR 22000
#define PROBE_SERIES_RESISTOR 10000

#define HEAT_PIN 5
#define LED_PIN 2 //TODO Consider adding LED to HEAT curcuit

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  81(27.2)          48407
//  174(78.9)         8456
//  286(141)          2094
#define TEMP_A -0.2613164095e-3
#define TEMP_B 3.670228955e-4
#define TEMP_C -2.935194202e-7
#define TEMP_BETA 3568.45

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  81(27.2)           112222
//  174(78.9)          14000
//  286(141)           2619
#define PROBE_A 0.1772729832e-3
#define PROBE_B 2.952734660e-4
#define PROBE_C -1.789284692e-7
#define PROBE_BETA 4256.98

#define MIN_TEMP 37.0
#define MAX_TEMP 135.0
#define ABORT_TEMP 175.0

// #define B_MODEL

#define PID_WINDOW_SIZE 2000

#define FERINHEIT // Uncomment to use Fahrenheit instead of Celsius

#define MQTT_ENABLED
#define MQTT_SERVER "mqtt.petrocik.net"
#define MQTT_CLIENT_ID_PREFIX "smoker"
#define MQTT_STATUS_TOPIC "smoker/%d/status"
#define MQTT_COMMAND_TOPIC "smoker/%d/command"
#define MQTT_DEBUG_TOPIC "smoker/%d/debug"
#define MQTT_PRESENCE_TOPIC "smoker/%d/presence"

#define ADS1 1
#define ADS2 2

//#define DEBUG_PROBE

#endif // CONFIG_H
