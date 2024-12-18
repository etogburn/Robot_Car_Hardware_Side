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
#include "Coms_Handler.h"
#include "robot_system.h"

#define COMMAND_OK 0x00FF
#define COMMAND_INVALID 0x00FE
#define COMMAND_NOACTION 0x00FD
#define COMMAND_READY 0x0001

typedef void (*CommandFunction_t)(DecodedPacket_t *packet, RobotSystem *robot);

typedef struct {
    uint16_t commandID;
    CommandFunction_t handler;
} CommandTableEntry_t;

void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);

// Function prototypes
void CommandHandler_Init(void);
void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot);


#endif /* INC_COMMAND_HANDLER_H_ */
