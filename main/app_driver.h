#ifndef APP_DRIVER_H
#define APP_DRIVER_H

#include <stdint.h>

// Init ADC + GPIO
void app_driver_init(void);

// Sensor read functions (return integer values)
int read_temp(void);       // °C
int read_humidity(void);   // %
int read_CO2(void);        // ppm

#endif // APP_DRIVER_H
