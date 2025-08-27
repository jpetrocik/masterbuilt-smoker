#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID "PNET"
#define WIFI_PASSWORD "5626278472"

//#define LCD_SUPPORTED  // Uncomment to enable LCD support
#define OTA_ENABLED

#define ADC_SAMPLE_SIZE 10

#define TEMP_SERIES_RESISTOR 10000
#define PROBE_SERIES_RESISTOR 10000

#define HEAT_PIN 5
#define LED_PIN 2 //TODO Consider adding LED to HEAT curcuit

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  22c 56000ohm
//  79c  7934
// 137c  1510ohm
#define TEMP_A 1.086460802e-3
#define TEMP_B 1.636467458e-4
#define TEMP_C 3.921441666e-7
#define TEMP_BETA 3563.39

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  24c  107021ohm
//  78c  13354ohm
// 137c   2581ohm
#define PROBE_A 0.4732619361e-3
#define PROBE_B 2.504413606e-4
#define PROBE_C -0.05321188939e-7
#define PROBE_BETA 4141.75

#define MIN_TEMP 37.0
#define MAX_TEMP 135.0
#define ABORT_TEMP 175.0

// #define B_MODEL

#define PID_WINDOW_SIZE 2000

#define FERINHEIT // Uncomment to use Fahrenheit instead of Celsius

#define MQTT_ENABLED
#define MQTT_SERVER "petrocik.net"
#define MQTT_CLIENT_ID_PREFIX "smoker"
#define MQTT_STATUS_TOPIC "smoker/%d/status"
#define MQTT_COMMAND_TOPIC "smoker/%d/command"
#define MQTT_DEBUG_TOPIC "smoker/%d/debug"
#define MQTT_DEVICE_TOPIC "smoker/device"

#define ADS1 1
#define ADS2 2

#define DEBUG_PROBE

#endif // CONFIG_H
