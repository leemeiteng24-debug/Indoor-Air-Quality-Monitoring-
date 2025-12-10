#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "esp_rmaker_core.h"
#include "esp_rmaker_standard_devices.h"
#include "esp_rmaker_standard_params.h"
#include "esp_rmaker_ota.h"

#include "app_network.h"
#include "app_insights.h"
#include "app_priv.h"
#include "app_driver.h"

static const char *TAG = "RM_ALL";

/* Timing / throttle settings */
#define SAMPLE_MS           3000   /* sensor read freq (local) */
#define REPORT_INTERVAL_MS 15000   /* minimum cloud report interval per sensor */
#define ALERT_PRINT_MS      3000

/* RTOS objects (must match app_priv.h externs) */
MessageBufferHandle_t sensor_buffer = NULL;
MessageBufferHandle_t alert_buffer = NULL;
SemaphoreHandle_t monitor_mutex = NULL;
EventGroupHandle_t alertEvent = NULL;
Sensor_packet_t normal_data  = {0};
Sensor_packet_t abnormal_data = {0};

/* RainMaker handles and cached params */
static esp_rmaker_node_t   *node = NULL;
static esp_rmaker_device_t *temp_device = NULL;
static esp_rmaker_device_t *humidity_device = NULL;
static esp_rmaker_device_t *co2_device = NULL;
static esp_rmaker_device_t *buzzer_device = NULL;

static esp_rmaker_param_t *rmaker_temp_param = NULL;
static esp_rmaker_param_t *rmaker_hum_param  = NULL;
static esp_rmaker_param_t *rmaker_co2_param  = NULL;

static esp_rmaker_param_t *TempAlert_param = NULL;
static esp_rmaker_param_t *HumidityAlert_param = NULL;
static esp_rmaker_param_t *CO2Alert_param = NULL;

static esp_rmaker_param_t *power_param = NULL;

/* Rate-limiting variables */
static int last_reported_temp = INT32_MIN;
static int last_reported_hum  = INT32_MIN;
static int last_reported_co2  = INT32_MIN;
static TickType_t last_report_tick_temp = 0;
static TickType_t last_report_tick_hum  = 0;
static TickType_t last_report_tick_co2  = 0;

/* Forwarded driver functions (app_driver.c) */
extern void app_driver_init(void);
extern int read_temp(void);
extern int read_humidity(void);
extern int read_CO2(void);

/* Buzzer write callback for voice/cloud control */
static esp_err_t buzzer_write_cb(const esp_rmaker_device_t *dev, const esp_rmaker_param_t *param,
                                 const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (strcmp(esp_rmaker_param_get_name(param), "Power") == 0) {
        bool on = val.val.b;

        buzzer_set_state(on);                     // <-- THIS controls GPIO

        esp_rmaker_param_update_and_report(param, val);
    }
    return ESP_OK;
}

/* Safe report helpers: only send to cloud on change or after interval */
static void safe_report_temp(int temp)
{
    TickType_t now = xTaskGetTickCount();
    if (temp != last_reported_temp || (now - last_report_tick_temp) * portTICK_PERIOD_MS >= REPORT_INTERVAL_MS) {
        if (rmaker_temp_param) {
            esp_rmaker_param_update_and_report(rmaker_temp_param, esp_rmaker_float((float)temp));
            last_reported_temp = temp;
            last_report_tick_temp = now;
            ESP_LOGD(TAG, "Reported Temp: %d", temp);
        }
    }
}
static void safe_report_hum(int hum)
{
    TickType_t now = xTaskGetTickCount();
    if (hum != last_reported_hum || (now - last_report_tick_hum) * portTICK_PERIOD_MS >= REPORT_INTERVAL_MS) {
        if (rmaker_hum_param) {
            esp_rmaker_param_update_and_report(rmaker_hum_param, esp_rmaker_float((float)hum));
            last_reported_hum = hum;
            last_report_tick_hum = now;
            ESP_LOGD(TAG, "Reported Hum: %d", hum);
        }
    }
}
static void safe_report_co2(int co2)
{
    TickType_t now = xTaskGetTickCount();
    if (co2 != last_reported_co2 || (now - last_report_tick_co2) * portTICK_PERIOD_MS >= REPORT_INTERVAL_MS) {
        if (rmaker_co2_param) {
            esp_rmaker_param_update_and_report(rmaker_co2_param, esp_rmaker_float((float)co2));
            last_reported_co2 = co2;
            last_report_tick_co2 = now;
            ESP_LOGD(TAG, "Reported CO2: %d", co2);
        }
    }
}

static void task1_sensor_monitor(void *pv)
{
    while (1) {
        int temp_value = read_temp();
        normal_data.temp = temp_value;
        abnormal_data.temp = temp_value;

        safe_report_temp(temp_value);

        if (xSemaphoreTake(monitor_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            /* Normal data always goes to sensor buffer */
            xMessageBufferSend(sensor_buffer, &normal_data, sizeof(normal_data), 0);

            if (temp_value > 40 || temp_value < 18) {
                xMessageBufferSend(alert_buffer, &abnormal_data, sizeof(abnormal_data), portMAX_DELAY);
                xEventGroupSetBits(alertEvent, temp_BITs);
            } else {
                xEventGroupSetBits(alertEvent, normal_BITS);
            }
            xSemaphoreGive(monitor_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void task2_co2_humidity(void *pv)
{
    while (1) {
        int CO2 = read_CO2();
        int humidity = read_humidity();

        normal_data.CO2 = CO2;
        normal_data.humidity = humidity;

        abnormal_data.CO2 = CO2;
        abnormal_data.humidity = humidity;

        safe_report_co2(CO2);
        safe_report_hum(humidity);

        if (CO2 > 1500) {
            xEventGroupSetBits(alertEvent, CO2_BITS);
            xMessageBufferSend(alert_buffer, &abnormal_data, sizeof(abnormal_data), portMAX_DELAY);
        }
        if (humidity > 90) {
            xEventGroupSetBits(alertEvent, humidity_BITS);
            xMessageBufferSend(alert_buffer, &abnormal_data, sizeof(abnormal_data), portMAX_DELAY);
        }

        xMessageBufferSend(sensor_buffer, &normal_data, sizeof(normal_data), 0);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void task3_alert(void *pv)
{
    Sensor_packet_t alertData;

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(alertEvent,
                                               temp_BITs | CO2_BITS | humidity_BITS,
                                               pdTRUE, pdFALSE, portMAX_DELAY);

        size_t r = xMessageBufferReceive(alert_buffer, &alertData, sizeof(alertData), pdMS_TO_TICKS(500));

        if (r == sizeof(alertData)) {
            if (bits & temp_BITs) {
                ESP_LOGW(TAG, "Temp ALERT: %d C", alertData.temp);
                esp_rmaker_param_update_and_report(TempAlert_param, esp_rmaker_int(1));
            }
            if (bits & CO2_BITS) {
                ESP_LOGW(TAG, "CO2 ALERT: %d ppm", alertData.CO2);
                esp_rmaker_param_update_and_report(CO2Alert_param, esp_rmaker_int(1));
            }
            if (bits & humidity_BITS) {
                ESP_LOGW(TAG, "Humidity ALERT: %d %%", alertData.humidity);
                esp_rmaker_param_update_and_report(HumidityAlert_param, esp_rmaker_int(1));
            }

            ESP_LOGW(TAG, "WARNING!!! Activating buzzer");
            gpio_set_level(buzzer_gpio, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(buzzer_gpio, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}


/* Task: printer (print latest sensor from buffer occasionally) */
static void printer_task(void *pv)
{
    Sensor_packet_t pkt;
    while (1) {
        size_t r = xMessageBufferReceive(sensor_buffer, &pkt, sizeof(pkt), pdMS_TO_TICKS(2000));
        if (r == sizeof(pkt)) {
            ESP_LOGI(TAG, "Temp: %d C, CO2: %d ppm, Hum: %d %%", pkt.temp, pkt.CO2, pkt.humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* RainMaker init: create node, devices, params and start services */
static void rainmaker_init_and_start(void)
{
    esp_rmaker_config_t cfg = { .enable_time_sync = false };
    node = esp_rmaker_node_init(&cfg, "Indoor Air Quality", "Multisensor Hub");
    if (!node) {
        ESP_LOGE(TAG, "esp_rmaker_node_init failed");
        abort();
    }

    /* Temperature (standard) */
    temp_device = esp_rmaker_temp_sensor_device_create("Temperature", NULL, 0);

    /* Humidity */
    humidity_device = esp_rmaker_device_create("Humidity", "esp.device.sensor", NULL);
    esp_rmaker_param_t *hum_p = esp_rmaker_param_create("Humidity", "esp.param.humidity", esp_rmaker_float(0), PROP_FLAG_READ);
    if (humidity_device && hum_p) esp_rmaker_device_add_param(humidity_device, hum_p);

    /* CO2 */
    co2_device = esp_rmaker_device_create("CO2", "esp.device.sensor", NULL);
    esp_rmaker_param_t *co2_p = esp_rmaker_param_create("CO2", "esp.param.co2", esp_rmaker_float(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    if (co2_device && co2_p) esp_rmaker_device_add_param(co2_device, co2_p);

    /* Buzzer switch (voice control) */
    buzzer_device = esp_rmaker_switch_device_create("Buzzer", buzzer_write_cb, NULL);

    /* Alerts */
    TempAlert_param = esp_rmaker_param_create("Temp Alert", "esp.param.alert_temp", esp_rmaker_int(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    HumidityAlert_param = esp_rmaker_param_create("Humidity Alert", "esp.param.alert_hum", esp_rmaker_int(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);
    CO2Alert_param = esp_rmaker_param_create("CO2 Alert", "esp.param.alert_co2", esp_rmaker_int(0), PROP_FLAG_READ | PROP_FLAG_PERSIST);

    if (TempAlert_param && temp_device) esp_rmaker_device_add_param(temp_device, TempAlert_param);
    if (HumidityAlert_param && humidity_device) esp_rmaker_device_add_param(humidity_device, HumidityAlert_param);
    if (CO2Alert_param && co2_device) esp_rmaker_device_add_param(co2_device, CO2Alert_param);

    /* Add devices to node */
    esp_rmaker_node_add_device(node, temp_device);
    esp_rmaker_node_add_device(node, humidity_device);
    esp_rmaker_node_add_device(node, co2_device);
    esp_rmaker_node_add_device(node, buzzer_device);

    /* Cache params for efficient updates */
    rmaker_temp_param = esp_rmaker_device_get_param_by_name(temp_device, ESP_RMAKER_DEF_TEMPERATURE_NAME);
    rmaker_hum_param  = esp_rmaker_device_get_param_by_name(humidity_device, "Humidity");
    rmaker_co2_param  = esp_rmaker_device_get_param_by_name(co2_device, "CO2");

    /* Start RainMaker agent, OTA and insights */
    /* If you want Google Home integration, enable the component in menuconfig and add esp_rmaker_google_home to components.
       If you have the Google Home support enabled in your SDK, uncomment the next line:
       esp_rmaker_enable_google_home();
    */
#ifdef CONFIG_ESP_RMAKER_GOOGLE_HOME_ENABLED
    /* If the component and Kconfig are available */
    esp_rmaker_enable_google_home();
#endif

    esp_rmaker_start();
    esp_rmaker_ota_enable_default();
    app_insights_enable();
    ESP_LOGI(TAG, "RainMaker started");
}

/* Application entry */
void app_main(void)
{
    esp_err_t err;

    /* Initialize TCP/IP and event loop (required by networking & RainMaker) */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Init hardware drivers */
    app_driver_init();

    /* NVS init */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Network helper (provisioning) */
    app_network_init();

    /* Create RTOS objects */
    monitor_mutex = xSemaphoreCreateMutex();
    sensor_buffer = xMessageBufferCreate(sizeof(Sensor_packet_t) * 12);
    alert_buffer  = xMessageBufferCreate(sizeof(Sensor_packet_t) * 6);
    alertEvent    = xEventGroupCreate();

    if (!monitor_mutex || !sensor_buffer || !alert_buffer || !alertEvent) {
        ESP_LOGE(TAG, "Failed to create RTOS objects");
        return;
    }

    /* Init RainMaker and devices */
    rainmaker_init_and_start();

    /* Start Wi-Fi / provisioning */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "app_network_start failed: 0x%x", err);
        abort();
    }

    /* Create tasks */
    xTaskCreate(task1_sensor_monitor, "task1_sensor_monitor", 4096, NULL, 5, NULL);
    xTaskCreate(task2_co2_humidity,  "task2_co2_humidity",  3072, NULL, 5, NULL);
    xTaskCreate(task3_alert,  "task3_alert",  3072, NULL, 4, NULL);
    xTaskCreate(printer_task,"printer_task",3072, NULL, 1, NULL);

    ESP_LOGI(TAG, "System ready — RainMaker + OTA (safe reporting). Waiting provisioning if needed.");
}