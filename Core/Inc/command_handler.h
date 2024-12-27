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
#define COMMAND_WAKE 0x0000
#define COMMAND_SHUTDOWN 0x0002
#define COMMAND_GETBATVOLT 0x0080
#define COMMAND_MOTORENABLE 0x0100
#define COMMAND_SETMOTORSPEED 0x0101
#define COMMAND_SETLEFTSPEED 0x0103
#define COMMAND_SETRIGHTSPEED 0x0104
#define COMMAND_MOTORSTOP 0x0102
#define COMMAND_GETMOTORSPEED 0x0180
#define COMMAND_GETMOTORPOSITION 0x0181
#define COMMAND_GETACCELVALS 0x0201
#define COMMAND_GETGYROVALS 0x0210

typedef void (*CommandFunction_t)(DecodedPacket_t *packet, RobotSystem *robot);

typedef struct {
    uint16_t commandID; //2 byte hex command
    uint16_t expLength; //expected data length. 0 means any length
    CommandFunction_t handler; //function to call
} CommandTableEntry_t;

#define COMMAND_0x0000 {COMMAND_WAKE, 0, Handle_WakeUp}
#define COMMAND_0x0001 {COMMAND_READY, 0, Handle_Ready}
#define COMMAND_0x0002 {COMMAND_SHUTDOWN, 0, Handle_Shutdown}
#define COMMAND_0x0080 {COMMAND_GETBATVOLT, 0, Handle_GetBatVolt}
#define COMMAND_0x0100 {COMMAND_MOTORENABLE, 1, Handle_SetMotorEnable}
#define COMMAND_0x0101 {COMMAND_SETMOTORSPEED, 4, Handle_SetMotorSpeed} //set both motors speed
#define COMMAND_0x0102 {COMMAND_MOTORSTOP, 0, Handle_SetMotorStop} //stop both motors
#define COMMAND_0x0103 {COMMAND_SETLEFTSPEED, 2, Handle_SetOneMotorSpeed} //set left motor speed
#define COMMAND_0x0104 {COMMAND_SETRIGHTSPEED, 2, Handle_SetOneMotorSpeed} //set right motor speed
#define COMMAND_0x0180 {COMMAND_GETMOTORSPEED, 0, Handle_GetMotorSpeed}
#define COMMAND_0x0181 {COMMAND_GETMOTORPOSITION, 0, Handle_GetMotorPosition}
#define COMMAND_0x0201 {COMMAND_GETACCELVALS, 0, Handle_GetAccelVals}
#define COMMAND_0x0210 {COMMAND_GETGYROVALS, 0, Handle_GetGyroVals}


void Handle_WakeUp(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_Ready(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_Shutdown(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorEnable(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorStop(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetOneMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetAccelVals(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetGyroVals(DecodedPacket_t *packet, RobotSystem *robot);
void Handle_GetBatVolt(DecodedPacket_t *packet, RobotSystem *robot);

// Function prototypes
void CommandHandler_Init(void);
void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot);


#endif /* INC_COMMAND_HANDLER_H_ */
