/*
 * command_handler.h
 *
 *  Created on: Nov 16, 2024
 *      Author: etogb
 */

#ifndef INC_COMMAND_HANDLER_H_
#define INC_COMMAND_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "serial_commands.h" // Include the SerialCommand_t definition
#include "drive_system.h"

#define COMMAND_OK 0x00FF
#define COMMAND_INVALID 0x00FE
#define COMMAND_NOACTION 0x00FD
#define COMMAND_READY 0x0001

// Function prototypes
void CommandHandler_Init(void);
void CommandHandler_ProcessCommand(SerialCommand_t *command, DriveSystem *drive);


#endif /* INC_COMMAND_HANDLER_H_ */
