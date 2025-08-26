#ifndef CONFIG_H
#define CONFIG_H

#define WIFI_SSID "PNET"
#define WIFI_PASSWORD "5626278472"

//#define LCD_SUPPORTED  // Uncomment to enable LCD support

#define ADC_SAMPLE_SIZE 10

#define TEMP_SERIES_RESISTOR 10000
#define PROBE_SERIES_RESISTOR 10000

#define HEAT_PIN 5
#define LED_PIN 2 //TODO Consider adding LED to HEAT curcuit

#define TEMP_A 0.0018973070382391657
#define TEMP_B 0.000028062360019658755
#define TEMP_C 9.298865188226613e-7

#define PROBE_A 0.0008347965991908151
#define PROBE_B 0.0001986718882689551
#define PROBE_C 1.487238280483084e-7

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
