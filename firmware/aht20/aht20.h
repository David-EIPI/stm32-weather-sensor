/**
  ******************************************************************************
  * @file           : aht20.h
  * @author         : David Shirvanyants
  * @date           : Feb 18, 2025
  * @brief          : Header for AOSONG AHT20 humidity/temperature sensor.
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

/**
 * Product page: http://www.aosong.com/en/products-32.html
 */

#ifndef AHT20_H_
#define AHT20_H_

#include <stdint.h>

/*
 * Device specific constants
 */



/* C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/***
 * @name Type definitions.
 ***/

/*
 * @enum AHT20 communication status code
 */
typedef enum aht_status {
	AHT_NO_ERROR,
	AHT_COMM_ERROR,
	AHT_TIMEOUT_ERROR,
	AHT_CRC_ERROR,
	AHT_BUSY_ERROR,

} aht_status_t;

/***
 * @name Function pointer types
 ***/

/*
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[out] data          : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] user_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef aht_status_t (*aht20_read_fptr_t)(uint8_t *data, uint32_t len, void *user_ptr);

/*
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] data          : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] user_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
typedef aht_status_t (*aht20_write_fptr_t)(uint8_t *data, uint32_t len, void *user_ptr);

/*
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] user_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *
 */
typedef void (*aht20_delay_ms_fptr_t)(uint32_t period, void *user_ptr);

/***
 * @name Data structures
 ***/

/*
 * @brief aht20 sensor output data structure: humidity and temperature
 * humidity data
 */
typedef struct aht20_data
{
    /* Temperature, degC x 100 */
    int32_t temperature;

    /* Relative humidity, %RH x 100 */
    uint32_t humidity;
} aht20_data_t;


/*
 * @brief aht20 device structure
 */
typedef struct aht20_dev
{
    /*
     * Pointer to the user data to be passed to the read/write/delay functions.
     * This will usually be a pointer to the I2C interface handle.
     */

    void *user_ptr;

    /* Read function pointer */
    aht20_read_fptr_t read;

    /* Write function pointer */
    aht20_write_fptr_t write;

    /* Delay function pointer */
    aht20_delay_ms_fptr_t delay_ms;

    /* Buffer for data exchange.
     * Largest data block in AHT20 communication is 7 bytes, so 8 bytes (including alignment byte) should be enough.
     * */
    uint8_t buffer[8];
} aht20_dev_t;


/*
 * Exported functions
 */

/* CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief Setup the sensor before first use.
  * @retval 0 on success, error code otherwise
  */
aht_status_t aht20_setup(void);

/**
  * @brief Initiate one-shot measurement and read its result.
  * @retval 0 on success, error code otherwise
  */
aht_status_t aht20_read_measurements(aht20_data_t *data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* AHT20_H_ */
