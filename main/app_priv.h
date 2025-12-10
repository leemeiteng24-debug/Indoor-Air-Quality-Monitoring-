#pragma once
#ifndef APP_PRIV_H
#define APP_PRIV_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

/* GPIO used by app */
#define buzzer_gpio         GPIO_NUM_8
#define CO2Switch_GPIO      GPIO_NUM_2
#define HumiditySwitch_GPIO GPIO_NUM_4
/* ADC channel used for temperature (map to your board) */
#define TEMP_ADC_CHANNEL    ADC1_CHANNEL_1

/* Event bits */
#define temp_BITs      (1 << 0)
#define CO2_BITS       (1 << 1)
#define humidity_BITS  (1 << 2)
#define normal_BITS    (1 << 3)

/* Shared sensor packet */
typedef struct {
    int temp;       /* °C */
    int humidity;   /* % */
    int CO2;        /* ppm */
    uint8_t status; /* 0 normal, 1 abnormal */
} Sensor_packet_t;

/* RTOS shared objects (defined in app_main.c) */
extern MessageBufferHandle_t sensor_buffer;
extern MessageBufferHandle_t alert_buffer;
extern SemaphoreHandle_t monitor_mutex;
extern EventGroupHandle_t alertEvent;

/* Global packets (defined in app_main.c) */
extern Sensor_packet_t normal_data;
extern Sensor_packet_t abnormal_data;

/* Driver API */
void app_driver_init(void);
int read_temp(void);       /* °C */
int read_humidity(void);   /* % */
int read_CO2(void);        /* ppm */

void buzzer_set_state(bool on);

#endif // APP_PRIV_H
