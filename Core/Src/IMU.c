/*
 * IMU.c
 *
 *  Created on: Nov 17, 2024
 *      Author: etogb
 */

#include "IMU.h"
#include <stdio.h> // For debugging with printf

// Internal helper function: Write a register
static HAL_StatusTypeDef IMU_WriteRegister(IMU_HandleTypeDef *imu, uint8_t reg, uint8_t value) {
    uint8_t data[2] = { reg, value };

    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(imu->hspi, data, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);

    return status;
}

// Internal helper function: Burst read
static HAL_StatusTypeDef IMU_BurstRead(IMU_HandleTypeDef *imu, uint8_t reg, uint8_t *buffer, uint8_t length) {

    uint8_t tx_data[length+1];
    uint8_t rx_data[length+1];
    tx_data[0] = reg | IMU_READ_BIT;

    for(uint8_t i = 1; i <= length; i++) {
    	tx_data[i] = (tx_data[i-1]+1);
    }

    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(imu->hspi, tx_data, rx_data, length + 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);

    for(uint8_t i = 1; i <= length; i++) {
    	*(buffer+(i-1)) = rx_data[i];
	}


    return status;
}

// Initialize the IMU
HAL_StatusTypeDef IMU_Init(IMU_HandleTypeDef *imu) {
    uint8_t who_am_i = 0;
//    uint8_t data_buffer[14];
//
//    for(uint8_t i = 0; i < 14; i++) {
//    	data_buffer[i] = 0;
//    }

    if (IMU_WriteRegister(imu, 0x76, 0x00) != HAL_OK) { //userbank 0 select
	 //   return HAL_ERROR;
	}
	HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_DEVICE_CONFIG_REG, IMU_DEVICE_CONFIG_RESET) != HAL_OK) {
    	//   return HAL_ERROR;
	}

    HAL_Delay(5);

    if (IMU_BurstRead(imu, IMU_WHO_AM_I_REG, &who_am_i, 1) != HAL_OK || who_am_i != IMU_WHO_AM_I_EXPECTED) {
        //return HAL_ERROR;
    }
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_PWR_MGMT_0, IMU_ENABLE_ACCEL_GYRO) != HAL_OK) {
     //   return HAL_ERROR;
    }
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_GYRO_CONFIG_REG, IMU_GYRO_CONFIG_DATA) != HAL_OK) {
     //   return HAL_ERROR;
    }
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_ACCEL_CONFIG_REG, IMU_ACCEL_CONFIG_DATA) != HAL_OK) {
     //   return HAL_ERROR;
    }
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_INT_CONFIG0_REG, IMU_INT_CONFIG0_DATA) != HAL_OK) {
	 //   return HAL_ERROR;
	}
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_INT_SOURCE_REG, IMU_INT_ENABLE_DATA_RDY) != HAL_OK) {
       // return HAL_ERROR;
    }
    HAL_Delay(1);

    if (IMU_WriteRegister(imu, IMU_INT_CONFIG_REG, IMU_INT_ACTIVE_HIGH) != HAL_OK) {
        //return HAL_ERROR;
    }
    HAL_Delay(1);


    return HAL_OK;
}

// Read accelerometer data
HAL_StatusTypeDef IMU_ReadAccel(IMU_HandleTypeDef *imu) {
    uint8_t raw_data[6];
    if (IMU_BurstRead(imu, IMU_ACCEL_XOUT_H, raw_data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    imu->accel[0] = (raw_data[0] << 8) | raw_data[1];
    imu->accel[1] = (raw_data[2] << 8) | raw_data[3];
    imu->accel[2] = (raw_data[4] << 8) | raw_data[5];

    return HAL_OK;
}

// Read gyroscope data
HAL_StatusTypeDef IMU_ReadGyro(IMU_HandleTypeDef *imu) {
    uint8_t raw_data[6];
    if (IMU_BurstRead(imu, IMU_GYRO_XOUT_H, raw_data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    imu->gyro[0] = (raw_data[0] << 8) | raw_data[1];
    imu->gyro[1] = (raw_data[2] << 8) | raw_data[3];
    imu->gyro[2] = (raw_data[4] << 8) | raw_data[5];

    return HAL_OK;
}

// Read all data (accelerometer, gyroscope, and temperature)
HAL_StatusTypeDef IMU_ReadAll(IMU_HandleTypeDef *imu) {
	uint8_t length = 14;
    uint8_t raw_data[length];
    uint8_t intStatus = 0;
    if (IMU_BurstRead(imu, IMU_TEMP_OUT_H, raw_data, length) != HAL_OK) {
        return HAL_ERROR;
    }

    imu->accel[0] = (raw_data[2] << 8) | raw_data[3];
    imu->accel[1] = (raw_data[4] << 8) | raw_data[5];
    imu->accel[2] = (raw_data[6] << 8) | raw_data[7];

    imu->temperature = (raw_data[0] << 8) | raw_data[1];

    imu->gyro[0] = (raw_data[8] << 8) | raw_data[9];
    imu->gyro[1] = (raw_data[10] << 8) | raw_data[11];
    imu->gyro[2] = (raw_data[12] << 8) | raw_data[13];

    if (IMU_BurstRead(imu, IMU_INT_STATUS_REG, &intStatus, 1) != HAL_OK) {
		return HAL_ERROR;
	}

    return HAL_OK;
}

// Interrupt handler
void IMU_InterruptHandler(IMU_HandleTypeDef *imu, uint16_t GPIO_pin) {
    if (GPIO_pin != imu->int_pin) return;

    if (IMU_ReadAll(imu) == HAL_OK) {
        // Data successfully read
    } else {
        // Handle read error
    }
}
