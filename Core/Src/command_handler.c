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
		COMMAND_0x0080,
		COMMAND_0x0100,
		COMMAND_0x0101,
		COMMAND_0x0102,
		COMMAND_0x0103,
		COMMAND_0x0104,
		COMMAND_0x0180,
		COMMAND_0x0181,
		COMMAND_0x0201,
		COMMAND_0x0210
};

// Function to initialize the Command Handler

static void SendResponse(ComsInterface_t *interface) {
	Comm_Send(interface, &response);
}

static void SetResponse(uint16_t command, uint8_t length, uint8_t *data) {
	if(data == NULL) {
		response.length = 0;
	} else {
		response.length = length;
		memset(response.data, 0, MAX_DATA_SIZE);
		memcpy(response.data, data, response.length);
	}
	response.invalid = false;
	response.command = command;
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

static void int16_tToUint8_t(int16_t *input, uint8_t *output, uint8_t num) {
	uint8_t maxLoop = num;

	for(uint8_t i = 0; i < maxLoop; i++) {
		output[2*i] = (input[i] & 0xFF00) >> 8;
		output[2*i+1] = input[i] & 0xFF;
	}
}

void CommandHandler_Init(void) {
    // Initialize any peripherals or variables related to command handling
    // Example: GPIO, Timers, etc.
}
// Function to process a received command
void CommandHandler_ProcessCommand(ComsInterface_t *interface, RobotSystem *robot) {
    // Check for NULL pointer
	DecodedPacket_t packet = Comm_GetPacket(interface);

	if(!packet.isNew) return;

	if(packet.invalid) {
		Response_Invalid();
		SendResponse(interface);
		return;
	}

	RobotSystem_ResetWatchdog(robot);

	for (int i = 0; i < sizeof(commandTable) / sizeof(CommandTableEntry_t); i++) {
		if (commandTable[i].commandID == packet.command) {
			if(commandTable[i].expLength == 0 || commandTable[i].expLength == packet.length) {
				commandTable[i].handler(&packet, robot);
			} else {
				Response_Invalid();
			}
			SendResponse(interface);
			return;
		}
	}
}

void Handle_SetMotorEnable(DecodedPacket_t *packet, RobotSystem *robot) {
	uint8_t data = packet->data[0] > 0 ? 1 : 0;

	if(data) {
		RobotSystem_Enable(robot);
	} else {
		RobotSystem_Disable(robot);
	}

	Response_OK();
}

void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {
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
	int16_t speed = makeInt16_t(&packet->data[0], &packet->data[1]);

	if(packet->command == COMMAND_SETLEFTSPEED) {
		RobotSystem_SetLeftSpeed(robot, speed);
	} else if(packet->command == COMMAND_SETRIGHTSPEED) {
		RobotSystem_SetRightSpeed(robot, speed);
	}

	Response_OK();
}

void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t wheelPos[2];
	uint8_t data[4];

	RobotSystem_GetMotorPosition(robot, &wheelPos[0], &wheelPos[1]);

	int16_tToUint8_t(wheelPos, data, 2);

	SetResponse(COMMAND_GETMOTORPOSITION, 4, data);
}

void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t speed[2];
	uint8_t data[4];

	RobotSystem_GetMotorSpeed(robot, &speed[0], &speed[1]);

	int16_tToUint8_t(speed, data, 2);

	SetResponse(COMMAND_GETMOTORSPEED, 4, data);
}

void Handle_WakeUp(DecodedPacket_t *packet, RobotSystem *robot) {
	SetResponse(COMMAND_READY, 0, NULL);
}

void Handle_Ready(DecodedPacket_t *packet, RobotSystem *robot) {
	SetResponse(COMMAND_READY, 0, NULL);
}

void Handle_Shutdown(DecodedPacket_t *packet, RobotSystem *robot) {
	RobotSystem_Shutdown(robot);
	Response_OK();
}

void Handle_GetAccelVals(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t accel[3];
	uint8_t data[6];

	RobotSystem_GetAccelVals(robot, accel);

	int16_tToUint8_t(accel, data, 3);

	SetResponse(COMMAND_GETACCELVALS, 6, data);
}

void Handle_GetGyroVals(DecodedPacket_t *packet, RobotSystem *robot) {
	int16_t gyro[3];
	uint8_t data[6];

	RobotSystem_GetGyroVals(robot, gyro);

	int16_tToUint8_t(gyro, data, 3);

	SetResponse(COMMAND_GETGYROVALS, 6, data);
}

void Handle_GetBatVolt(DecodedPacket_t *packet, RobotSystem *robot) {
	uint16_t batVolt;
	uint8_t data[2];

	RobotSystem_GetBatVolt(robot, &batVolt);

	data[0] = batVolt >> 8;
	data[1] = batVolt & 0xFF;
	SetResponse(COMMAND_GETBATVOLT, 2, data);
}

