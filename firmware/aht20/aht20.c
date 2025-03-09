/**
  ******************************************************************************
  * @file           : aht20.c
  * @author         : David Shirvanyants
  * @date           : Feb 18, 2025
  * @brief          : Driver for AOSONG AHT20 humidity/temperature sensor.
  * @version        : 1.0
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*
 * This file has 2 sections.
 * First section is the portable part, second is STM32-specific.
 *
 * First section is loosely based on the reference code provided by the manufacturer.
 * Ref: http://www.aosong.com/en/products-32.html
 */


#include "aht20.h"
#include <assert.h>

/*
 * Useful macros
 */
#define RETURN_ON_ERROR(res) \
	if (AHT_NO_ERROR != (res)) \
		return res;


/**
 * Device specific constants
 **/

/**
 * Values are coming from the manufacturer-supplied driver source.
 **/
#define MAIN_ADDR           0x70

#define READY_MASK          0x18
#define REG_RESET_MASK      0xB0

#define CALIBRATION_STATUS  0x08
#define BUSY_STATUS         0x80

#define INIT_REG1	        0x1B
#define INIT_REG2	        0x1C
#define INIT_REG3	        0x1E

#define RESET_CMD           0xA8
#define CALIBRATE_CMD       0xBE
#define MEASURE_CMD         0xAC

#define BUSY_TIMEOUT        100

/*
 * Read and store sensor data
 */
static
aht_status_t aht20_read_bytes(aht20_dev_t *dev, int len)
{
	if (len > sizeof(dev->buffer))
		len = sizeof(dev->buffer);
/*
	dev->buffer = MAIN_ADDR | 1;
	aht_status_t result = dev->write(dev->buffer, 1, dev->user_ptr);
*/
//	if (AHT_NO_ERROR == result)
		aht_status_t result = dev->read(dev->buffer, len, dev->user_ptr);

	return result;
}

/*
 * Send the command
 */
static
aht_status_t aht20_write_cmd(aht20_dev_t *dev, uint8_t cmd, uint8_t param1, uint8_t param2)
{
//	dev->buffer[0] = MAIN_ADDR;
	dev->buffer[0] = cmd;
	dev->buffer[1] = param1;
	dev->buffer[2] = param2;
	aht_status_t result = dev->write(dev->buffer, 3, dev->user_ptr);
	return result;
}

/*
 * This reset procedure is copied from the manufacturer's source
 */
static
aht_status_t aht20_reset_reg(aht20_dev_t *dev, uint8_t reg)
{
	aht_status_t result = aht20_write_cmd(dev, reg, 0, 0);
	RETURN_ON_ERROR(result)

	dev->delay_ms(5, dev->user_ptr);

	result = aht20_read_bytes(dev, 3);
	RETURN_ON_ERROR(result)

	dev->delay_ms(10, dev->user_ptr);

	result = aht20_write_cmd(dev, REG_RESET_MASK | reg, dev->buffer[1], dev->buffer[2]);
	RETURN_ON_ERROR(result)

	return result;
}

/*
 * The usual reset at startup, followed by the calibration command.
 */
static
aht_status_t aht20_soft_reset(aht20_dev_t *dev)
{
	aht_status_t result = aht20_write_cmd(dev, RESET_CMD, 0, 0);
	RETURN_ON_ERROR(result)

	dev->delay_ms(10, dev->user_ptr);
	result = aht20_write_cmd(dev, CALIBRATE_CMD, 0x08, 0);
	RETURN_ON_ERROR(result)

	dev->delay_ms(10, dev->user_ptr);
	return result;
}

/*
 * Validate device handle and run initialization sequence.
 */
static
aht_status_t aht20_initialize(aht20_dev_t *dev)
{
	assert(dev != NULL);
	assert(dev->read != NULL);
	assert(dev->write != NULL);
	assert(dev->delay_ms != NULL);

	aht_status_t result = aht20_soft_reset(dev);
	return result;
}

/*
 * This is some sort of readiness test, as documented by the manufacturer.
 */
static
aht_status_t aht20_check_ready(aht20_dev_t *dev)
{

	/* Read chip status */
	aht_status_t result = aht20_read_bytes(dev, 1);
	RETURN_ON_ERROR(result)

	/* Check calibration status */
	if ((dev->buffer[0] & READY_MASK) != READY_MASK) {
		result = aht20_reset_reg(dev, INIT_REG1);
		RETURN_ON_ERROR(result)
		result = aht20_reset_reg(dev, INIT_REG2);
		RETURN_ON_ERROR(result)
		result = aht20_reset_reg(dev, INIT_REG3);
		RETURN_ON_ERROR(result)
		dev->delay_ms(10, dev->user_ptr);

		result = aht20_read_bytes(dev, 1);
		RETURN_ON_ERROR(result)
	}

	/* Verify calibration status and request re-calibration if necessary */
	if (0 == (dev->buffer[0] & CALIBRATION_STATUS)) {
		result = aht20_write_cmd(dev, CALIBRATE_CMD, 0x08, 0);
		RETURN_ON_ERROR(result)
		dev->delay_ms(10, dev->user_ptr);
	}

	return result;
}

/*
 * Copied from the manufacturer's reference code.
 * Computes x^8 + x^5 + x^4 + 1 8-bit CRC code.
 */

static
uint8_t calc_crc8(const uint8_t *data, int len)
{
    uint8_t crc=0xFF;

    uint8_t byte;
    for(byte=0; byte < len; byte++) {

    	crc ^= data[byte];

    	uint8_t i;
    	for(i = 8; i > 0; --i) {

    		if(crc & 0x80) {
    			crc = (crc << 1) ^ 0x31;
    		} else {
    			crc = (crc << 1);
    		}

    	}
    }

    return crc;
}

/*
 * Check data CRC8 and convert to human readable values.
 * Input data must be present in dev->buffer.
 */
static
aht_status_t aht20_check_convert_data(aht20_dev_t *dev, aht20_data_t *data)
{
	uint8_t crc = calc_crc8(dev->buffer, 6);
	if (crc != dev->buffer[6])
		return AHT_CRC_ERROR;

	if (BUSY_STATUS & dev->buffer[0])
		return AHT_BUSY_ERROR;


	int32_t temp_20bit = 0;
	int32_t hum_20bit = 0;

	hum_20bit |= (dev->buffer[1] << 12) | (dev->buffer[2] << 4) | (dev->buffer[3] >> 4);

	temp_20bit |= ((dev->buffer[3] & 0x0f) << 16) | (dev->buffer[4] << 8) | dev->buffer[5];

	/* Divide by 16 both enumerator and denominator to avoid overflow */
	data->humidity = (hum_20bit * (100 * 100 / 16)) / (1048576 / 16);
	data->temperature = ((temp_20bit * (100 * 200 / 16) ) / (1048576 / 16) ) - (50 * 100);
	return AHT_NO_ERROR;
}

/*
 * Main readout routine.
 * Check calibration status, then send one-shot measurement command.
 */
static
aht_status_t aht20_read_data(aht20_dev_t *dev, aht20_data_t *data)
{
	/* Check chip's calibration and initialization status */
	aht_status_t result = aht20_check_ready(dev);
	RETURN_ON_ERROR(result)

	/* Request measurement */
	result = aht20_write_cmd(dev, MEASURE_CMD, 0x33, 0x00);
	RETURN_ON_ERROR(result)

	/* Documented minimum required delay */
	dev->delay_ms(80, dev->user_ptr);

	int timeout_counter = BUSY_TIMEOUT;

	/* Wait if the sensor is still busy */
	result = aht20_read_bytes(dev, 1);
	RETURN_ON_ERROR(result)

	while (BUSY_STATUS & dev->buffer[0]) {
		dev->delay_ms(1, dev->user_ptr);

		result = aht20_read_bytes(dev, 1);
		RETURN_ON_ERROR(result)

		if (timeout_counter-- <= 0) {
			return AHT_TIMEOUT_ERROR;
		}
	}

	result = aht20_read_bytes(dev, 7);
	RETURN_ON_ERROR(result)

	result = aht20_check_convert_data(dev, data);

	if (data->humidity > 10000 || data->temperature > 10000 || data->temperature < -5000)
		result = AHT_COMM_ERROR;

	return result;
}

/*******
 *   STM32-specific support functions.
 ******/

/*
 * Core source header file includes STM32 library files and exports I2C handler.
 *  */
#include "main.h"

/* I2C receive/transmit timeout, ms */
#define COMM_TIMEOUT 100

aht_status_t AHT20_I2C_read(uint8_t *data, uint32_t len, void *user_ptr)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)user_ptr;

	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(hi2c, MAIN_ADDR, data, len, COMM_TIMEOUT);

	if (HAL_TIMEOUT == status)
		return AHT_TIMEOUT_ERROR;
	if (HAL_OK != status)
		return AHT_COMM_ERROR;

	return AHT_NO_ERROR;
}


aht_status_t AHT20_I2C_write(uint8_t *data, uint32_t len, void *user_ptr)
{
	I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)user_ptr;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, MAIN_ADDR, data, len, COMM_TIMEOUT);

	if (HAL_TIMEOUT == status)
		return AHT_TIMEOUT_ERROR;
	if (HAL_OK != status)
		return AHT_COMM_ERROR;

	return AHT_NO_ERROR;
}

void AHT20_delay_ms(uint32_t period, void *user_ptr)
{
	HAL_Delay(period);
}

static struct aht20_dev aht20_dev = {
	.write = AHT20_I2C_write,
	.read = AHT20_I2C_read,
	.delay_ms = AHT20_delay_ms,
	.user_ptr = &hi2c1,
};

/**
  * @brief Setup the sensor before first use.
  * @retval 0 on success, error code otherwise
  */
aht_status_t aht20_setup(void)
{
	return aht20_initialize(&aht20_dev);
}

/**
  * @brief Initiate one-shot measurement and read its result.
  * @retval 0 on success, error code otherwise
  */
aht_status_t aht20_read_measurements(aht20_data_t *data)
{
	return aht20_read_data(&aht20_dev, data);
}
