#include "app_priv.h"
#include <stdlib.h>
#include <stdio.h>
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "app_driver";
static esp_adc_cal_characteristics_t adc_chars;
static bool rand_seeded = false;

void app_driver_init(void)
{
    /* ADC init (12-bit, atten 6dB) - adjust to your sensor wiring */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEMP_ADC_CHANNEL, ADC_ATTEN_DB_6);

    /* Calibrate ADC (Vref 1100 mV default) */
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    /* Buttons inputs (active LOW) */
    gpio_reset_pin(CO2Switch_GPIO);
    gpio_set_direction(CO2Switch_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CO2Switch_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(HumiditySwitch_GPIO);
    gpio_set_direction(HumiditySwitch_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HumiditySwitch_GPIO, GPIO_PULLUP_ONLY);

    /* Buzzer output */
    gpio_reset_pin(buzzer_gpio);
    gpio_set_direction(buzzer_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(buzzer_gpio, 0);
    
    ESP_LOGI(TAG, "Driver initialized");
}

void buzzer_set_state(bool on){
    gpio_set_level(buzzer_gpio, on? 1:0); 
}  

/* Read temperature from MCP9700-like sensor:
 * raw ADC -> voltage (mV) -> temp = (V_mV - 500) / 10
 * returns int °C, or -1 on error
 */
int read_temp(void)
{
    int raw = adc1_get_raw(TEMP_ADC_CHANNEL);
    if (raw < 0) {
        ESP_LOGW(TAG, "adc1_get_raw failed");
        return -1;
    }
    if (raw < 50) {
        ESP_LOGW(TAG, "ADC raw too small, possible sensor disconnected (raw=%d)", raw);
        return -1;
    }
    uint32_t voltage_mV = esp_adc_cal_raw_to_voltage((uint32_t)raw, &adc_chars);
    float temp = ((float)voltage_mV - 500.0f) / 10.0f;
    ESP_LOGD(TAG, "ADC raw=%d, V=%umV, temp=%.2fC", raw, (unsigned)voltage_mV, temp);
    return (int)(temp + 0.5f);
}

/* read_CO2: active-low button -> pressed = abnormal */
int read_CO2(void)
{
    if (gpio_get_level(CO2Switch_GPIO) == 0){
    //switch pressed = CO2 high
    return 1000 + (rand()%(4000-1000+1));
    } else {
        //switch release = normal CO2 range
        return 400 + (rand()%(1000-400+1));
}}

/* read_humidity: active-low button -> pressed = abnormal */
int read_humidity(void)
{
    if (gpio_get_level(CO2Switch_GPIO) == 0){
    //switch pressed = humiidty high
    return 70 + (rand()%(100-70+1));
    } else {
        //switch release = normal humidity range
        return 30 + (rand()%(69-30+1));
}}
