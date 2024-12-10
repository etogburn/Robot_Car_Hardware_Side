/*
 * CAN_Driver.h
 *
 *  Created on: Nov 22, 2024
 *      Author: etogb
 */

#ifndef INC_CAN_DRIVER_H_
#define INC_CAN_DRIVER_H_

#include "stm32g4xx_hal.h"
#include "main.h"
#include <stdint.h>

// CAN message structure
typedef struct {
    uint32_t id;        // Identifier
    uint8_t data[8];    // Data payload
    uint8_t length;     // Length of the data
    uint8_t format;     // Standard or Extended
    uint8_t type;       // Data or Remote frame
} CAN_Message;

// Initialization and deinitialization
void CAN_Init(FDCAN_HandleTypeDef *hcan);
void CAN_DeInit(FDCAN_HandleTypeDef *hcan);

// Sending and receiving messages
HAL_StatusTypeDef CAN_SendMessage(FDCAN_HandleTypeDef *hcan, CAN_Message *msg);
HAL_StatusTypeDef CAN_ReceiveMessage(FDCAN_HandleTypeDef *hcan, CAN_Message *msg);

void CAN_SendWakeMessage(FDCAN_HandleTypeDef *hcan);

// Error handling
void CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan);


#endif /* INC_CAN_DRIVER_H_ */
