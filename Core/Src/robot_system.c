/*
 * robot_system.c
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#include "robot_system.h"

// Initialize the robot system with configurations for the left and right motors
void RobotSystem_Init(RobotSystem *robotSystem, Motor leftMotorConfig, Motor rightMotorConfig) {
    // Copy configurations into the robot system
    robotSystem->leftWheel = leftMotorConfig;
    robotSystem->rightWheel = rightMotorConfig;

    HAL_DAC_Start(robotSystem->currentLimitDAC, robotSystem->currentLimitDACChannel);

    RobotSystem_SetCurrentLimit(robotSystem, MAX_CURRENT_LIMIT);

    // Initialize the left and right motors
    Motor_Init(&robotSystem->leftWheel);
    Motor_Init(&robotSystem->rightWheel);

    RobotSystem_Enable(robotSystem);
}

// Set the speeds of the left and right motors
void RobotSystem_SetSpeed(RobotSystem *robotSystem, int16_t leftSpeed, int16_t rightSpeed) {
    Motor_SetSpeed(&robotSystem->leftWheel, leftSpeed);
    Motor_SetSpeed(&robotSystem->rightWheel, rightSpeed);
}

void RobotSystem_SetLeftSpeed(RobotSystem *robotSystem, int16_t leftSpeed) {
	Motor_SetSpeed(&robotSystem->leftWheel, leftSpeed);
}

void RobotSystem_SetRightSpeed(RobotSystem *robotSystem, int16_t rightSpeed) {
	Motor_SetSpeed(&robotSystem->rightWheel, rightSpeed);
}

// Stop both motors in the robot system
void RobotSystem_Stop(RobotSystem *robotSystem) {
    Motor_Stop(&robotSystem->leftWheel);
    Motor_Stop(&robotSystem->rightWheel);
}

void RobotSystem_Calculate(RobotSystem *robotSystem) {
	Motor_Calculate(&robotSystem->leftWheel);
	Motor_Calculate(&robotSystem->rightWheel);
}

void RobotSystem_InterruptHandler(RobotSystem *robotSystem, TIM_HandleTypeDef *htim) {
	if (htim == robotSystem->leftWheel.HallTimer) {
		Motor_Update(&robotSystem->leftWheel);
	}

	if (htim == robotSystem->rightWheel.HallTimer) {
		Motor_Update(&robotSystem->rightWheel);
	}
}

void RobotSystem_Enable (RobotSystem *robotSystem) {
	RobotSystem_SetEnablePin(robotSystem, true);
}

void RobotSystem_Disable (RobotSystem *robotSystem) {
	RobotSystem_SetEnablePin(robotSystem, false);
}

void RobotSystem_SetEnablePin(RobotSystem *robotSystem, bool onOrOff) {
	bool setValue = true;
	if(onOrOff) setValue = true;
	else setValue = false;

	HAL_GPIO_WritePin(robotSystem->Enable_Port, robotSystem->Enable_Pin, setValue);
}

void RobotSystem_SetCurrentLimit(RobotSystem *robotSystem, uint16_t currentLimit) {
	if(currentLimit > MAX_CURRENT_LIMIT) {
		currentLimit = MAX_CURRENT_LIMIT;
	}
	robotSystem->currentLimit = currentLimit;

	uint32_t voltage = currentLimit * SHUNT_RESISTOR * RESISTORS / RESISTOR2;
	uint32_t dac_value = 4095 * voltage / 3300; // Convert voltage to digital value
	HAL_DAC_SetValue(robotSystem->currentLimitDAC, robotSystem->currentLimitDACChannel, DAC_ALIGN_12B_R, dac_value);
	HAL_Delay(25);
}

