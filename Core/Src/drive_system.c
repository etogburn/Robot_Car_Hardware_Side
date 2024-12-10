/*
 * drive_system.c
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#include "drive_system.h"

// Initialize the drive system with configurations for the left and right motors
void DriveSystem_Init(DriveSystem *driveSystem, Motor leftMotorConfig, Motor rightMotorConfig) {
    // Copy configurations into the drive system
    driveSystem->leftWheel = leftMotorConfig;
    driveSystem->rightWheel = rightMotorConfig;

    HAL_DAC_Start(driveSystem->currentLimitDAC, driveSystem->currentLimitDACChannel);

    DriveSystem_SetCurrentLimit(driveSystem, MAX_CURRENT_LIMIT);

    // Initialize the left and right motors
    Motor_Init(&driveSystem->leftWheel);
    Motor_Init(&driveSystem->rightWheel);

    DriveSystem_Enable(driveSystem);
}

// Set the speeds of the left and right motors
void DriveSystem_SetSpeed(DriveSystem *driveSystem, int16_t leftSpeed, int16_t rightSpeed) {
    Motor_SetSpeed(&driveSystem->leftWheel, leftSpeed);
    Motor_SetSpeed(&driveSystem->rightWheel, rightSpeed);
}

void DriveSystem_SetLeftSpeed(DriveSystem *driveSystem, int16_t leftSpeed) {
	Motor_SetSpeed(&driveSystem->leftWheel, leftSpeed);
}

void DriveSystem_SetRightSpeed(DriveSystem *driveSystem, int16_t rightSpeed) {
	Motor_SetSpeed(&driveSystem->rightWheel, rightSpeed);
}

// Stop both motors in the drive system
void DriveSystem_Stop(DriveSystem *driveSystem) {
    Motor_Stop(&driveSystem->leftWheel);
    Motor_Stop(&driveSystem->rightWheel);
}

void DriveSystem_Calculate(DriveSystem *driveSystem) {
	Motor_Calculate(&driveSystem->leftWheel);
	Motor_Calculate(&driveSystem->rightWheel);
}

void DriveSystem_InterruptHandler(DriveSystem *driveSystem, TIM_HandleTypeDef *htim) {
	if (htim == driveSystem->leftWheel.HallTimer) {
		Motor_Update(&driveSystem->leftWheel);
	}

	if (htim == driveSystem->rightWheel.HallTimer) {
		Motor_Update(&driveSystem->rightWheel);
	}
}

void DriveSystem_Enable (DriveSystem *driveSystem) {
	DriveSystem_SetEnablePin(driveSystem, true);
}

void DriveSystem_Disable (DriveSystem *driveSystem) {
	DriveSystem_SetEnablePin(driveSystem, false);
}

void DriveSystem_SetEnablePin(DriveSystem *driveSystem, bool onOrOff) {
	bool setValue = true;
	if(onOrOff) setValue = true;
	else setValue = false;

	HAL_GPIO_WritePin(driveSystem->Enable_Port, driveSystem->Enable_Pin, setValue);
}

void DriveSystem_SetCurrentLimit(DriveSystem *driveSystem, uint16_t currentLimit) {
	if(currentLimit > MAX_CURRENT_LIMIT) {
		currentLimit = MAX_CURRENT_LIMIT;
	}
	driveSystem->currentLimit = currentLimit;

	uint32_t voltage = currentLimit * SHUNT_RESISTOR * RESISTORS / RESISTOR2;
	uint32_t dac_value = 4095 * voltage / 3300; // Convert voltage to digital value
	HAL_DAC_SetValue(driveSystem->currentLimitDAC, driveSystem->currentLimitDACChannel, DAC_ALIGN_12B_R, dac_value);
	HAL_Delay(25);
}

