/*
 * IMU.h
 *
 *  Created on: Nov 17, 2024
 *      Author: etogb
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32g4xx_hal.h"

// Polling frequency in Hz (configurable)
#define IMU_POLLING_FREQUENCY 100
#define IMU_POLLING_DELAY_MS (1000 / IMU_POLLING_FREQUENCY)

#define IMU_TEMP_OUT_H        0x1D //good
#define IMU_ACCEL_XOUT_H      0x1F //good
#define IMU_GYRO_XOUT_H       0x25 //good

#define IMU_INT_CONFIG_REG	  0x14 //good
#define IMU_INT_ACTIVE_HIGH   0x07 //latched interrupt pin, push/pull, active high

#define IMU_PWR_MGMT_0        0x4E //good
#define IMU_ENABLE_ACCEL_GYRO 0x0F // enable both gyro and accelerometer

#define IMU_WHO_AM_I_REG      0x75 //good
#define IMU_WHO_AM_I_EXPECTED 0x42 //good

#define IMU_INT_STATUS_REG	  0x2D //read interrupts to clear

#define IMU_GYRO_CONFIG_REG   0x4F
#define IMU_GYRO_CONFIG_DATA  0x48 //500dps, 100 Hz

#define IMU_ACCEL_CONFIG_REG   0x50
#define IMU_ACCEL_CONFIG_DATA  0x48 //4g, 100 Hz

#define IMU_INT_CONFIG0_REG    0x63
#define IMU_INT_CONFIG0_DATA   0x20 //set to clear status bit when data is read

#define IMU_INT_SOURCE_REG      0x65 //good
#define IMU_INT_ENABLE_DATA_RDY 0x08

#define IMU_DEVICE_CONFIG_REG   0x11
#define IMU_DEVICE_CONFIG_RESET 0x01 //reset device


// SPI read/write bit
#define IMU_READ_BIT 0x80

// IMU handle structure
typedef struct {
    SPI_HandleTypeDef *hspi;      // SPI handle
    GPIO_TypeDef *cs_port;        // GPIO port for CS pin
    uint16_t cs_pin;              // GPIO pin for CS
    GPIO_TypeDef *int_port;       // GPIO port for interrupt pin
    uint16_t int_pin;             // GPIO pin for interrupt
    int16_t accel[3];             // Accelerometer data
    int16_t gyro[3];              // Gyroscope data
    int16_t temperature;            // Temperature in degrees Celsius
} IMU_HandleTypeDef;

// Function prototypes
HAL_StatusTypeDef IMU_Init(IMU_HandleTypeDef *imu);
HAL_StatusTypeDef IMU_ReadAccel(IMU_HandleTypeDef *imu);
HAL_StatusTypeDef IMU_ReadGyro(IMU_HandleTypeDef *imu);
HAL_StatusTypeDef IMU_ReadAll(IMU_HandleTypeDef *imu);
void IMU_InterruptHandler(IMU_HandleTypeDef *imu, uint16_t GPIO_pin);

#endif /* INC_IMU_H_ */
