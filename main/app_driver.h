#ifndef APP_DRIVER_H
#define APP_DRIVER_H

#include <stdint.h>

// Init ADC + GPIO
void app_driver_init(void);

// Sensor read functions (return integer values)
int read_temp(void);       // °C
int read_humidity(void);   // %
int read_CO2(void);        // ppm

esp_err_t app_driver_set_gpio(const char *param_name, bool state);

#endif // APP_DRIVER_H
