/*
 * command_handler.c
 *
 *  Created on: Nov 16, 2024
 *      Author: etogb
 */
#include "command_handler.h"
#include "stm32g4xx_hal.h" // Replace with your STM32 series HAL header, if needed



// Function to initialize the Command Handler
void CommandHandler_Init(void) {
    // Initialize any peripherals or variables related to command handling
    // Example: GPIO, Timers, etc.
}

// Function to process a received command
void CommandHandler_ProcessCommand(SerialCommand_t *command, DriveSystem *drive) {
    // Check for NULL pointer
    if (command == NULL) {
        return;
    }
    int16_t value = (command->data[0] << 8) | command->data[1];

    if (command->invalid && command->newCommand) {
    	SerialCommands_Send(COMMAND_INVALID, value);
    }

    if (command->newCommand && !command->invalid) {
    	// Handle the command based on the command ID
		switch (command->command) {

			case 0x0001: // Example: Motor 1 Speed
				{
					SerialCommands_Send(COMMAND_READY, value);
				}
				break;
			case 0x0101: // Example: Motor 1 Speed
				{
					DriveSystem_SetLeftSpeed(drive, value);
					//SerialCommands_Send(COMMAND_OK, command->value);
				}
				break;

			case 0x0102: // Example: Motor 2 Speed
				{
					DriveSystem_SetRightSpeed(drive, value);
					//SerialCommands_Send(COMMAND_OK, command->value);
				}
				break;

			case 0x0103: // Example: Enable Motor Blade
				{
					DriveSystem_Stop(drive);
					//SerialCommands_Send(COMMAND_OK, command->value);
				}
				break;

			default: // Unknown command
				// Handle invalid or unrecognized commands
				SerialCommands_Send(COMMAND_NOACTION, value);
				break;
		}
    }

    command->newCommand = false;

}


