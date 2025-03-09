#ifndef _BME280_SUPPORT_
#define _BME280_SUPPORT_
#include "bme280_defs.h"

/* Setup the BME280 driver with the STM32-specific callbacks. */
int8_t bme280_setup(void);

/* Readout the data. In case of BMP280 humidity is not read. */
int bme280_read_measurements(struct bme280_data *out_data);

/* Implemented with SysTick, intf_ptr is ignored */
void BME280_delay_us(uint32_t period, void *intf_ptr);

#endif
