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

#define COMMAND_0x0000 {0x0000, Handle_WakeUp}
#define COMMAND_0x0001 {0x0001, Handle_Ready}
#define COMMAND_0x0002 {0x0002, Handle_Shutdown}
#define COMMAND_0x0100 {0x0100, Handle_SetMotorEnable}
#define COMMAND_0x0101 {0x0101, Handle_SetMotorSpeed} //set both motors speed
#define COMMAND_0x0102 {0x0102, Handle_SetMotorStop} //stop both motors
#define COMMAND_0x0103 {0x0103, Handle_SetOneMotorSpeed} //set left motor speed
#define COMMAND_0x0104 {0x0104, Handle_SetOneMotorSpeed} //set right motor speed
#define COMMAND_0x0180 {0x0180, Handle_GetMotorSpeed}
#define COMMAND_0x0181 {0x0181, Handle_GetMotorPosition}


void Handle_WakeUp(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_Ready(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_Shutdown(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorEnable(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorStop(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetOneMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);

// Function prototypes
void CommandHandler_Init(void);
void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot);


#endif /* INC_COMMAND_HANDLER_H_ */
