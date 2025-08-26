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

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  75f 48303ohm
// 172f  8824ohm
// 259f  1853ohm
#define TEMP_A 1.934498636e-3
#define TEMP_B 0.2199374955e-4
#define TEMP_C 9.514088853e-7
#define TEMP_BETA 3791.59

//https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
//  75f  107021ohm
// 172f   13354ohm
// 259f   3038ohm
#define PROBE_A 0.8741501404e-3
#define PROBE_B 1.926411642e-4
#define PROBE_C 1.675424292e-7
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
#define MQTT_DEVICE_TOPIC "smoker/device"


#endif // CONFIG_H
