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
    {0x0101, Handle_SetMotorSpeed},
	{0x0164, Handle_GetMotorSpeed},
	{0x0165, Handle_GetMotorPosition}
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
		SetResponse(COMMAND_INVALID, 0, NULL);
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

void Handle_SetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {

	if(packet->length != 4) return;

	int16_t leftSpeed = makeInt16_t(&packet->data[0], &packet->data[1]);
	int16_t rightSpeed = makeInt16_t(&packet->data[2], &packet->data[3]);

	RobotSystem_SetSpeed(robot, leftSpeed, rightSpeed);

	SetResponse(COMMAND_OK, 0, NULL);

}

void Handle_GetMotorPosition(DecodedPacket_t *packet, RobotSystem *robot) {

}

void Handle_GetMotorSpeed(DecodedPacket_t *packet, RobotSystem *robot) {

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


