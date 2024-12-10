/*
 * serial_commands.h
 *
 *  Created on: Nov 16, 2024
 *      Author: etogb
 */

#ifndef INC_SERIAL_COMMANDS_H_
#define INC_SERIAL_COMMANDS_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32g4xx_hal.h" // Replace with your STM32 series HAL header

// Constants
#define START_BYTE 0xAA
#define PACKET_SIZE 6

#define RX_BUFFER_SIZE 16 //maximum bytes per receive
#define CMD_BUF_SIZE 16 //number of commands in the buffer

#define MAX_DATA_SIZE 8

// Command structure
typedef struct {
    uint16_t command; // Command ID
    uint8_t data[MAX_DATA_SIZE];   // Associated value
    uint8_t length;
    bool newCommand;
    bool invalid;
} SerialCommand_t;

typedef struct {
	uint8_t recieveIdx;
	uint8_t sendIdx;
	uint8_t processIdx;
	SerialCommand_t command[CMD_BUF_SIZE];
} SerialBuffer_t;
// Expose the current command to the main loop
extern SerialCommand_t current_command;

// Function prototypes
void SerialCommands_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);
void SerialCommands_HandleUARTInterrupt(void);
void SerialCommands_SetupRecieve(void);
void SerialCommands_Send(uint16_t command, int16_t value);
void SerialCommands_BigSend(uint8_t *input, uint8_t length);
void SerialCommands_DoEvents();
SerialCommand_t * SerialCommands_GetCommand();

#endif /* INC_SERIAL_COMMANDS_H_ */
