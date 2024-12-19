/*
 * command_handler.c
 *
 *  Created on: Nov 16, 2024
 *      Author: etogb
 */
#include "command_handler.h"
#include "stm32g4xx_hal.h" // Replace with your STM32 series HAL header, if needed

static DecodedPacket_t response;

CommandTableEntry_t commandTable[] = {
		COMMAND_0x0000,
		COMMAND_0x0001,
		COMMAND_0x0002,
		COMMAND_0x0100,
		COMMAND_0x0101,
		COMMAND_0x0102,
		COMMAND_0x0103,
		COMMAND_0x0104,
		COMMAND_0x0180,
		COMMAND_0x0181
};

// Function to initialize the Command Handler

static void SendResponse(ComsInterface_t *interface) {
	Comm_Send(interface, &response);
}

static void SetResponse(uint16_t command, uint8_t length, uint8_t *data) {
	response.invalid = false;
	response.command = command;
	response.length = sizeof(*data);
	memset(response.data, 0, MAX_DATA_SIZE);
	memcpy(response.data, data, sizeof(*data));
}

static void Response_OK() {
	SetResponse(COMMAND_OK, 0, NULL);
}

static void Response_Invalid() {
	SetResponse(COMMAND_INVALID, 0, NULL);
}

static int16_t makeInt16_t(uint8_t *val1, uint8_t *val2) {
	return (*val1 << 8) | *val2;
}

void CommandHandler_Init(void) {
    // Initialize any peripherals or variables related to command handling
    // Example: GPIO, Timers, etc.
}
// Function to process a received command
void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot) {
    // Check for NULL pointer
	DecodedPacket_t packet = Comm_GetPacket(interface);

	if(packet.invalid) {
		Response_Invalid();
		SendResponse(interface);
		return;
	}

	for (int i = 0; i < sizeof(commandTable) / sizeof(CommandTableEntry_t); i++) {
		if (commandTable[i].commandID == packet.command) {
			commandTable[i].handler(&packet, robot);
			SendResponse(interface);
			return;
		}
	}
}

void Handle_SetMotorEnable(DecodedPacket_t *packet, RobotSystem *robot) {
	if(packet->length != 1) {
		Response_Invalid();
		return;
	}

	uint8_t data = packet->data[0] > 0 ? 1 : 0;
	RobotSystem_SetEnablePin(robot, data);
}

void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {

	if(packet->length != 4) {
		Response_Invalid();
		return;
	}

	int16_t leftSpeed = makeInt16_t(&packet->data[0], &packet->data[1]);
	int16_t rightSpeed = makeInt16_t(&packet->data[2], &packet->data[3]);
	RobotSystem_SetSpeed(robot, leftSpeed, rightSpeed);

	Response_OK();
}

void Handle_SetMotorStop(DecodedPacket_t *packet, RobotSystem *robot) {
	RobotSystem_SetSpeed(robot, 0, 0);

	Response_OK();
}

void Handle_SetOneMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {

	if(packet->length != 2) {
		Response_Invalid();
		return;
	}

	int16_t speed = makeInt16_t(&packet->data[0], &packet->data[1]);

	if(packet->command == 0x0103) {
		RobotSystem_SetLeftSpeed(robot, speed);
	} else if(packet->command == 0x0104) {
		RobotSystem_SetRightSpeed(robot, speed);
	}
}

void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t leftPos;
	int16_t rightPos;
	RobotSystem_GetMotorPosition(robot, &leftPos, &rightPos);

	uint8_t data[4];

	data[0] = leftPos >> 8;
	data[1] = leftPos & 0xFF;
	data[2] = rightPos >> 8;
	data[3] = rightPos & 0xFF;

	SetResponse(0x0180, sizeof(data), data);
}

void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t leftSpeed;
	int16_t rightSpeed;
	RobotSystem_GetMotorSpeed(robot, &leftSpeed, &rightSpeed);

	uint8_t data[4];
	data[0] = leftSpeed >> 8;
	data[1] = leftSpeed & 0xFF;
	data[2] = rightSpeed >> 8;
	data[3] = rightSpeed & 0xFF;

	SetResponse(0x0180, sizeof(data), data);
}

void Handle_WakeUp(DecodedPacket_t *packet, RobotSystem *robot) {
	SetResponse(COMMAND_READY, 0, NULL);
}

void Handle_Ready(DecodedPacket_t *packet, RobotSystem *robot) {
	SetResponse(COMMAND_READY, 0, NULL);
}

void Handle_Shutdown(DecodedPacket_t *packet, RobotSystem *robot) {
	Response_OK();
}

// Function to process a received command
// void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot) {
//     // Check for NULL pointer
// 	DecodedPacket_t command = Comm_GetPacket(interface);


//     int16_t value = (command.data[0] << 8) | command.data[1];


//     if (!command.invalid) {
//     	// Handle the command based on the command ID
// 		switch (command.command) {

// 			case 0x0001: // Example: Motor 1 Speed
// 				{
// 					SetResponse(COMMAND_READY, 0xFFFF);
// 					Comm_Send(interface, &response);
// 				}
// 				break;
// 			case 0x0101: // Example: Motor 1 Speed
// 				{
// 					DriveSystem_SetLeftSpeed(robot, value);
// 				}
// 				break;

// 			case 0x0102: // Example: Motor 2 Speed
// 				{
// 					DriveSystem_SetRightSpeed(robot, value);
// 				}
// 				break;

// 			case 0x0103: // Example: Enable Motor Blade
// 				{
// 					DriveSystem_Stop(robot);
// 				}
// 				break;

// 			default: // Unknown command
// 				// Handle invalid or unrecognized commands
// 				break;
// 		}
//     }
// }


