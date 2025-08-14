#ifndef CONFIG_H
#define CONFIG_H

#define SSID "PNET2"
#define PASSWORD "5626278472"

#define LCD_SUPPORTED  // Uncomment to enable LCD support

#define ADC_SAMPLE_SIZE 3

#define TEMP_SERIES_RESISTOR 5400
#define PROBE_SERIES_RESISTOR 10000

#define HEAT_PIN 5

#define TEMP_A 1.136646777e-3
#define TEMP_B 1.600914823e-4
#define TEMP_C 3.695194917e-7

#define PROBE_A 0.0006631412844551252
#define PROBE_B 0.0002230253514824875
#define PROBE_C 8.079307552907064E-8

#define MIN_TEMP 37.0
#define MAX_TEMP 135.0

#define PID_WINDOW_SIZE 2000

#define FERINHEIT // Uncomment to use Fahrenheit instead of Celsius

#define MQTT_SERVER "petrocik.net"
#define MQTT_CLIENT_ID_PREFIX "smoker"
#define MQTT_PUBLISH_TOPIC "smoker/status"
#define MQTT_SUBSCRIBE_TOPIC "smoker/action"

#define MQTT_ENABLED

// current state
struct SmokerState {
  double temperature = 0;
  double probe1 = 0;
  double probe2 = 0;
  double probe3 = 0;
  double probe4 = 0;
  double targetTemperature = 0;
  long cookEndTime = 0;
  long cookTime = 0;
};

#endif // CONFIG_H
