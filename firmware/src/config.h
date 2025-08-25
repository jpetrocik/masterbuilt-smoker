#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID "PNET"
#define WIFI_PASSWORD "5626278472"

//#define LCD_SUPPORTED  // Uncomment to enable LCD support

#define ADC_SAMPLE_SIZE 3

#define TEMP_SERIES_RESISTOR 10000
#define PROBE_SERIES_RESISTOR 10000

#define HEAT_PIN 5
#define LED_PIN 2 //TODO Consider adding LED to HEAT curcuit

#define TEMP_A 1.136646777e-3
#define TEMP_B 1.600914823e-4
#define TEMP_C 3.695194917e-7

#define PROBE_A 0.0007541609823298523
#define PROBE_B 0.00021424300720139587
#define PROBE_C 8.639335870951432e-8

#define MIN_TEMP 37.0
#define MAX_TEMP 135.0
#define ABORT_TEMP 175.0

#define PID_WINDOW_SIZE 2000

#define FERINHEIT // Uncomment to use Fahrenheit instead of Celsius

#define MQTT_ENABLED
#define MQTT_SERVER "petrocik.net"
#define MQTT_CLIENT_ID_PREFIX "smoker"
#define MQTT_STATUS_TOPIC "smoker/%d/status"
#define MQTT_COMMAND_TOPIC "smoker/%d/command"
#define MQTT_DEVICE_TOPIC "smoker/device"


#endif // CONFIG_H
