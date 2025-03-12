#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <PID_v1.h>
#include <../components/cjson/cJSON.h>

#define TAG "SMOKER"

#define TEMP_PROBE                   ADC_CHANNEL_6
#define MEAT_PROBE1                  ADC_CHANNEL_5
#define MEAT_PROBE2                  ADC_CHANNEL_7
#define MEAT_PROBE3                  ADC_CHANNEL_3
#define MEAT_PROBE4                  ADC_CHANNEL_4

#define HEAT_PIN 5

#define ADC_SAMPLE_SIZE 8

#define VCC 3300
#define TEMP_SERIES_RESISTOR 5400
#define TEMP_VOLTAGE_DIVIDE 17820000 // VCC * TEMP_SERIES_RESISTOR
#define PROBE_SERIES_RESISTOR 5460
#define PROBE_VOLTAGE_DIVIDE 18564000 // VCC * PROBE_SERIES_RESISTOR   


#define TEMP_A 1.136646777e-3
#define TEMP_B 1.600914823e-4
#define TEMP_C 3.695194917e-7

#define PROBE_A -0.4885255109e-3
#define PROBE_B 4.082924384e-4
#define PROBE_C -5.463408188e-7

#define toF(x)  x * 1.8 + 32;
#define toC(x)  (x - 32) / 1.8;


//adc buffers
static uint16_t temp_buffer[ADC_SAMPLE_SIZE] = {0};
static uint8_t temp_index = 0;
static uint16_t probe1_buffer[ADC_SAMPLE_SIZE] = {0};
static uint8_t probe1_index = 0;
static uint16_t probe2_buffer[ADC_SAMPLE_SIZE] = {0};
static uint8_t probe2_index = 0;
static uint16_t probe3_buffer[ADC_SAMPLE_SIZE] = {0};
static uint8_t probe3_index = 0;
static uint16_t probe4_buffer[ADC_SAMPLE_SIZE] = {0};
static uint8_t probe4_index = 0;

//main loop variables
static double targetTemperature = 0;
static long cookEndTime = 0;

static adc_cali_handle_t adc1_cali_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL;


static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static double readTemperature(adc_channel_t channel, uint16_t adc_buffer[], uint8_t* adc_index_ptr);
static float calculate_Temperature_B_Value(uint32_t res);
static float calculate_Temperature_SH_Value(uint32_t res);
static float calculate_Probe_SH_Value(uint32_t res);


void app_main(void)
{
cJSON *json = cJSON_Parse("{\"title\":\"Hello World\"}");

    double temperature = 0;
    double probe1 = 0;
    double probe2 = 0;
    double probe3 = 0;
    double probe4 = 0;
    long lastRead = 0;
    long now;
    bool abortError = false;

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, TEMP_PROBE, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MEAT_PROBE1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MEAT_PROBE2, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MEAT_PROBE3, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MEAT_PROBE4, &config));

    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_11, &adc1_cali_handle);


    while (1) {
        now = millis();

        if (now - lastRead > 1000) 
        {
            temperature = readTemperature(ADC_CHANNEL_6, temp_buffer, &temp_index);
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, ADC_CHANNEL_6, temperature);
            probe1 = readTemperature(ADC_CHANNEL_5, probe1_buffer, &probe1_index);
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, ADC_CHANNEL_5, probe1);
            probe2 = readTemperature(ADC_CHANNEL_7, probe2_buffer, &probe2_index);
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, ADC_CHANNEL_7, probe2);
            probe3 = readTemperature(ADC_CHANNEL_3, probe3_buffer, &probe3_index);
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, ADC_CHANNEL_3, probe3);
            probe4 = readTemperature(ADC_CHANNEL_4, probe4_buffer, &probe4_index);
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %f mV", ADC_UNIT_1 + 1, ADC_CHANNEL_4, probe4);

            lastRead = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        // ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        // // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        // if (do_calibration1) {
        //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][0], &voltage[0][0]));
        //     ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
        // }

        // ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
        // // ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0][1]);
        // if (do_calibration1) {
        //     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw[0][1], &voltage[0][1]));
        //     ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage[0][1]);
        // }
        // vTaskDelay(pdMS_TO_TICKS(1000));

    }

    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1) {
        example_adc_calibration_deinit(adc1_cali_handle);
    }

}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static double readTemperature(adc_channel_t channel, uint16_t adc_buffer[], uint8_t* adc_index_ptr) {
    int reading;
    int voltage;
    uint32_t resistance;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, channel, &reading));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, reading, &voltage));

    return voltage;
    // if (channel == ADC_CHANNEL_6) {
    //     resistance = (TEMP_SERIES_RESISTOR * voltage) / - (VCC - voltage);
    //     return calculate_Temperature_SH_Value(resistance);
    // } else { 
    //     resistance = (PROBE_SERIES_RESISTOR * voltage) / (VCC - voltage); //- PROBE_SERIES_RESISTOR;
    //     return calculate_Probe_SH_Value(resistance);
    // }
}

static float calculate_Temperature_B_Value(uint32_t res) {
  //B value calculation
  float bVale = res / 100000.0;     // (R/Ro)
  bVale = log(bVale);                  // ln(R/Ro)
  bVale /= 4400;                   // 1/B * ln(R/Ro)
  bVale += 1.0 / (25 + 273.15); // + (1/To)
  bVale = 1.0 / bVale;                 // Invert
  bVale -= 273.15;

  return bVale;
}

static float calculate_Temperature_SH_Value(uint32_t res) {
  float logRes = log(res);           // Pre-Calcul for Log(R2)
  float temp = (1.0 / (TEMP_A + TEMP_B * logRes + TEMP_C * logRes * logRes * logRes));
  return  temp - 273.15;             // convert Kelvin to *C
}

static float calculate_Probe_SH_Value(uint32_t res) {
  float logRes = log(res);           // Pre-Calcul for Log(R2)
  float temp = (1.0 / (PROBE_A + PROBE_B * logRes + PROBE_C * logRes * logRes * logRes));
  return  temp - 273.15;             // convert Kelvin to *C
}

//uint32_t readADC_Cal(int16_t ADC_Raw)
//{
//  return esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars);
//}

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
