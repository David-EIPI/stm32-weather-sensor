/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bme280_support.c
* Date: 2016/07/04
* Revision: 1.0.6 $
*
* Usage: Sensor Driver support file for BME280 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/

#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"

#define BME280_ADDRESS1 0xEC
#define BME280_ADDRESS2 0xEE

int8_t BME280_I2C_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

int8_t BME280_I2C_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/********************End of I2C/SPI function declarations***********************/


/*	Brief : The delay routine
 *	\param : delay in us
*/
void BME280_delay_us(uint32_t period, void *intf_ptr);

static struct bme280_intf_desc {
    /*! I2C address
     * Interface will try all addresses and
     * store the first one detected
     */
    uint8_t addr;

    /*
     *	Target interface handle
     * */
    I2C_HandleTypeDef *intf;
} bme280_intf_desc = {
		.addr = 0,
		.intf = &hi2c1,
};

/*----------------------------------------------------------------------------*
 *  struct bme280_t parameters can be accessed by using bme280
 *	bme280_t having the following parameters
 *	Bus write function pointer: BME280_WR_FUNC_PTR
 *	Bus read function pointer: BME280_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Interface of the sensor: .intf
 *---------------------------------------------------------------------------*/
static struct bme280_dev bme280 = {
	.intf = BME280_I2C_INTF,
	.write = BME280_I2C_write,
	.read = BME280_I2C_read,
	.delay_us = BME280_delay_us,
	.intf_ptr = &bme280_intf_desc,
};

static struct bme280_settings settings = {0};

static uint32_t req_delay = 0;

/*
 * Precomputed systicks per 1 microsecond
 * */
uint32_t SysTicks_us = 0;

/*	Brief : The delay routine
 *	\param : delay in us
*/
void BME280_delay_us(uint32_t period, void *intf_ptr)
{
	if (period > 1000) {
		HAL_Delay(period / 1000);
		period %= 1000;
	}
	uint32_t loop_usec = 500; /* Should be smaller than SysTick reload value, usually =1000us */
	do {
		if (period < loop_usec) {
			loop_usec = period;
		}
		period -= loop_usec;
		uint32_t start = SysTick->VAL;
		uint32_t ticks = loop_usec * SysTicks_us;
		while (start - SysTick->VAL < ticks) ;
	} while (period > 0);
}

/************** I2C/SPI buffer length ******/

#define	I2C_BUFFER_LEN 28


 /*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t BME280_I2C_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
	int32_t iError = BME280_OK;
	struct bme280_intf_desc *intf = (struct bme280_intf_desc *)intf_ptr;
	I2C_HandleTypeDef *hi2c = intf->intf;

//	while (HAL_I2C_IsDeviceReady(hi2c, BME280_ADDRESS, 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(hi2c,						// i2c handle
    						  intf->addr,	                // i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
							  (uint8_t*)(reg_data),		// write data provided in reg_data
							  len,							// write how many bytes
							  100);							// timeout

	if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and <0 returned as failure
    	iError = (BME280_E_COMM_FAIL);
    }
	return (int8_t)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of data byte of to be read
 */
int8_t BME280_I2C_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
	int32_t iError = BME280_OK;
//	uint8_t array[I2C_BUFFER_LEN] = {0};
	struct bme280_intf_desc *intf = (struct bme280_intf_desc *)intf_ptr;
	I2C_HandleTypeDef *hi2c = intf->intf;

//	uint8_t stringpos = 0;
//	array[0] = reg_addr;

//	do {
//		status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(BME280_ADDRESS), 3, 100);
//	} while (status != HAL_OK);

    status = HAL_I2C_Mem_Read(hi2c,						// i2c handle
    						  intf->addr,				// i2c address, left aligned
							  (uint8_t)reg_addr,			// register address
							  I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
//							  (uint8_t*)(&array),			// write returned data to this variable
							  (uint8_t*)(reg_data),			// write returned data to this variable
							  len,							// how many bytes to expect returned
							  100);							// timeout

    if (status != HAL_OK)
    {
    	// The BME280 API calls for 0 return value as a success, and <0 returned as failure
    	iError = (BME280_E_COMM_FAIL);
    }
/*
    for (stringpos = 0; stringpos < len; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
*/
	return (int8_t)iError;
}


/*--------------------------------------------------------------------------*
*	Initialize, reset and update the settings.
*	May be called multiple times.
*-------------------------------------------------------------------------*/

int8_t bme280_setup(void)
{
	int8_t rslt;
	SysTicks_us = SystemCoreClock / 1000000U;

	/* Try the first address ... */
	((struct bme280_intf_desc *)bme280.intf_ptr)->addr = BME280_ADDRESS1;
	rslt = bme280_init(&bme280);

	/* ... and the second if the first one failed. */
	if (rslt != BME280_OK) {
		((struct bme280_intf_desc *)bme280.intf_ptr)->addr = BME280_ADDRESS2;
		rslt = bme280_init(&bme280);
	}

	if (rslt != BME280_OK) {
		return rslt;
	}

	rslt = bme280_get_sensor_settings(&settings, &bme280);
	if (rslt != BME280_OK) {
		return rslt;
	}

#if 1
	/* Configuring the over-sampling rate, filter coefficient and standby time */
    /* Overwrite the desired settings */
    settings.filter = BME280_FILTER_COEFF_OFF;

    /* Over-sampling rate for humidity, temperature and pressure */
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;

    /* Setting the standby time */
    settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280);
	if (rslt != BME280_OK) {
		return rslt;
	}
#endif
    /* Always set the power mode after setting the configuration */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_SLEEP, &bme280);
	if (rslt != BME280_OK) {
		return rslt;
	}

    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
         *  and the oversampling configuration. */
    rslt = bme280_cal_meas_delay(&req_delay, &settings);
	if (rslt != BME280_OK) {
		req_delay = 0;
		return rslt;
	}
	return rslt;
}

int bme280_read_measurements(struct bme280_data *out_data, uint8_t read_humidity)
{
	int rslt;

	rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280);
    if (rslt != BME280_OK) {
        return rslt;
    }

    /* Wait for the measurement to complete and read data */
    bme280.delay_us(req_delay, bme280.intf_ptr);

    /* Read humidity data only if chip id matches BME280 */
    rslt = bme280_get_sensor_data(bme280.chip_id == BME280_CHIP_ID ? BME280_ALL : BME280_PRESS|BME280_TEMP,
    		out_data, &bme280);
    if (rslt != BME280_OK) {
    	return rslt;
    }

    return BME280_OK;
}
