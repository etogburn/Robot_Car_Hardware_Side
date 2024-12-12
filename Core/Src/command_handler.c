/*
 * command_handler.c
 *
 *  Created on: Nov 16, 2024
 *      Author: etogb
 */
#include "command_handler.h"
#include "stm32g4xx_hal.h" // Replace with your STM32 series HAL header, if needed

static DecodedPacket_t response;
// Function to initialize the Command Handler

static void SetResponse(uint16_t command, uint8_t *data) {
	response.invalid = false;
	response.command = command;
	response.length = sizeof(*data);
	memset(response.data, 0, MAX_DATA_SIZE);
	memcpy(response.data, data, sizeof(*data));
}

void CommandHandler_Init(void) {
    // Initialize any peripherals or variables related to command handling
    // Example: GPIO, Timers, etc.
}

// Function to process a received command
void CommandHandler_ProcessCommand(ComsInterface_t *interface, DriveSystem *drive) {
    // Check for NULL pointer
	DecodedPacket_t command = Comm_GetPacket(interface);


    int16_t value = (command.data[0] << 8) | command.data[1];


    if (!command.invalid) {
    	// Handle the command based on the command ID
		switch (command.command) {

			case 0x0001: // Example: Motor 1 Speed
				{
					SetResponse(COMMAND_READY, 0xFFFF);
					Comm_Send(interface, &response);
				}
				break;
			case 0x0101: // Example: Motor 1 Speed
				{
					DriveSystem_SetLeftSpeed(drive, value);
				}
				break;

			case 0x0102: // Example: Motor 2 Speed
				{
					DriveSystem_SetRightSpeed(drive, value);
				}
				break;

			case 0x0103: // Example: Enable Motor Blade
				{
					DriveSystem_Stop(drive);
				}
				break;

			default: // Unknown command
				// Handle invalid or unrecognized commands
				break;
		}
    }
}




