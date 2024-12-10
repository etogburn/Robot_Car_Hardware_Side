/*
 * CAN_Driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: etogb
 */


#include "CAN_Driver.h"

// Initialize the CAN peripheral
void CAN_Init(FDCAN_HandleTypeDef *hcan) {
	HAL_GPIO_WritePin(nCAN_STBY_GPIO_Port, nCAN_STBY_Pin, GPIO_PIN_SET);

    FDCAN_FilterTypeDef filterConfig;


    filterConfig.IdType = FDCAN_EXTENDED_ID;
	filterConfig.FilterIndex = 0;
	filterConfig.FilterType = FDCAN_FILTER_MASK;
	filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filterConfig.FilterID1 = 0x00;
	filterConfig.FilterID2 = 0x00;

    // Configure CAN filter
//    filterConfig.FilterBank = 0;                         // Filter bank index
//    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;     // Mask mode
//    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    // 32-bit scale
//    filterConfig.FilterIdHigh = 0x0000;                  // Filter ID
//    filterConfig.FilterIdLow = 0x0000;                   // Filter ID
//    filterConfig.FilterMaskIdHigh = 0x0000;              // Mask ID
//    filterConfig.FilterMaskIdLow = 0x0000;               // Mask ID
//    filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Assign FIFO0
//    filterConfig.FilterActivation = ENABLE;              // Enable filter

    if (HAL_FDCAN_ConfigFilter(hcan, &filterConfig) != HAL_OK) {
        // Handle error
    }

    // Start the CAN peripheral
    if (HAL_FDCAN_Start(hcan) != HAL_OK) {
        // Handle error
    }

    // Enable interrupt for receiving messages
    if (HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        // Handle error
    }

}

// Deinitialize the CAN peripheral
void CAN_DeInit(FDCAN_HandleTypeDef *hcan) {
    if (HAL_FDCAN_DeInit(hcan) != HAL_OK) {
        // Handle error
    }
}

// Send a CAN message
HAL_StatusTypeDef CAN_SendMessage(FDCAN_HandleTypeDef *hcan, CAN_Message *msg) {
    FDCAN_TxHeaderTypeDef txHeader;

	txHeader.Identifier = 0x00;
	txHeader.IdType = FDCAN_EXTENDED_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = msg->length; //FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_FD_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;
    // Configure the Tx header
//    txHeader.StdId = (msg->format == 0) ? msg->id : 0;
//    txHeader.ExtId = (msg->format == 1) ? msg->id : 0;
//    txHeader.IDE = (msg->format == 0) ? CAN_ID_STD : CAN_ID_EXT;
//    txHeader.RTR = (msg->type == 0) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
//    txHeader.DLC = msg->length;

    // Transmit the message
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &txHeader, msg->data);
}

// Receive a CAN message
HAL_StatusTypeDef CAN_ReceiveMessage(FDCAN_HandleTypeDef *hcan, CAN_Message *msg) {
    FDCAN_RxHeaderTypeDef rxHeader;

    // Receive the message
    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rxHeader, msg->data) != HAL_OK) {
        return HAL_ERROR;
    }

    // Populate the message structure
    msg->id = rxHeader.Identifier;
    msg->format = (rxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
    //msg->type = (rxHeader.RTR == CAN_RTR_DATA) ? 0 : 1;
    msg->length = rxHeader.DataLength;

    return HAL_OK;
}

void CAN_SendWakeMessage(FDCAN_HandleTypeDef *hcan) {
	uint32_t currentTime = HAL_GetTick();
	static uint32_t lastTime = 0;



	if(currentTime - lastTime > 5) {

		CAN_Message message = {
				.id = 0x44AA,
				.data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x44, 0x55},
				.length = 8,
				.format = 1,
				.type = 0
		};

		CAN_SendMessage(hcan, &message);

		lastTime = currentTime;
	}
}

// Callback for CAN errors
void CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan) {
    // Implement error handling
}
