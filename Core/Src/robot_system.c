/*
 * robot_system.c
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#include "robot_system.h"

// Initialize the robot system with configurations for the left and right motors

void RobotSystem_Init(RobotSystem *robotSystem, Motor leftMotorConfig, Motor rightMotorConfig, IMU_HandleTypeDef imuConfig, SystemDiagnostics_t sysConfig) {
    // Copy configurations into the robot system
    robotSystem->leftWheel = leftMotorConfig;
    robotSystem->rightWheel = rightMotorConfig;
    robotSystem->imu = imuConfig;
    robotSystem->sys = sysConfig;

    HAL_DAC_Start(robotSystem->currentLimitDAC, robotSystem->currentLimitDACChannel);

    RobotSystem_SetCurrentLimit(robotSystem, MAX_CURRENT_LIMIT);

    SysDiag_Init(&robotSystem->sys);

    // Initialize the left and right motors
    IMU_Init(&robotSystem->imu);
    Motor_Init(&robotSystem->leftWheel);
    Motor_Init(&robotSystem->rightWheel);

    RobotSystem_Enable(robotSystem);
}

// Set the speeds of the left and right motors
void RobotSystem_SetSpeed(RobotSystem *robotSystem, int16_t leftSpeed, int16_t rightSpeed) {
	if(robotSystem->motorsEnabled) {
		Motor_SetSpeed(&robotSystem->leftWheel, leftSpeed);
		Motor_SetSpeed(&robotSystem->rightWheel, rightSpeed);
	}
}

void RobotSystem_SetLeftSpeed(RobotSystem *robotSystem, int16_t leftSpeed) {
	if(robotSystem->motorsEnabled) {
		Motor_SetSpeed(&robotSystem->leftWheel, leftSpeed);
	}
}

void RobotSystem_SetRightSpeed(RobotSystem *robotSystem, int16_t rightSpeed) {
	if(robotSystem->motorsEnabled) {
		Motor_SetSpeed(&robotSystem->rightWheel, rightSpeed);
	}
}

// Stop both motors in the robot system
void RobotSystem_Stop(RobotSystem *robotSystem) {
    Motor_Stop(&robotSystem->leftWheel);
    Motor_Stop(&robotSystem->rightWheel);
}

void RobotSystem_GetMotorPosition(RobotSystem *robotSystem, int16_t *leftPos, int16_t *rightPos) {
	Motor_GetDistance(&robotSystem->leftWheel, leftPos);
	Motor_GetDistance(&robotSystem->rightWheel, rightPos);
}

void RobotSystem_GetMotorSpeed(RobotSystem *robotSystem, int16_t *leftSpeed, int16_t *rightSpeed) {
	Motor_GetSpeed(&robotSystem->leftWheel, leftSpeed);
	Motor_GetSpeed(&robotSystem->rightWheel, rightSpeed);
}

void RobotSystem_Calculate(RobotSystem *robotSystem) {
	SysDiag_SleepWatchdog(&robotSystem->sys);
	Motor_Calculate(&robotSystem->leftWheel);
	Motor_Calculate(&robotSystem->rightWheel);
	RobotSystem_WheelFaultHandler(robotSystem);
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
	robotSystem->motorsEnabled = true;
	RobotSystem_SetEnablePin(robotSystem, true);
	HAL_Delay(20);
}

void RobotSystem_Disable (RobotSystem *robotSystem) {
	robotSystem->motorsEnabled = false;
	RobotSystem_Stop(robotSystem);
	RobotSystem_SetEnablePin(robotSystem, false);

	HAL_Delay(20);
}

void RobotSystem_SetEnablePin(RobotSystem *robotSystem, bool onOrOff) {
	bool setValue = 0;
	if(onOrOff) setValue = true;
	else setValue = false;

	HAL_GPIO_WritePin(robotSystem->Enable_Port, robotSystem->Enable_Pin, setValue);
	//__HAL_TIM_SET_COMPARE(robotSystem->FaultTimer, TIM_CHANNEL_1, setValue);
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

void RobotSystem_WheelFaultHandler(RobotSystem *robotSystem) {
	if(Motor_GetFaultStatus(&robotSystem->leftWheel)) {
		RobotSystem_ResetEnablePin(robotSystem);
		//RobotSystem_SetLeftSpeed(robotSystem, 0);
	}

	if(Motor_GetFaultStatus(&robotSystem->rightWheel)) {
		RobotSystem_ResetEnablePin(robotSystem);
		//RobotSystem_SetRightSpeed(robotSystem, 0);
	}
}

void RobotSystem_ResetEnablePin(RobotSystem *robotSystem) {
	if(robotSystem->motorsEnabled) {
		RobotSystem_SetEnablePin(robotSystem, false);

		for(uint16_t i = 0; i < 10; i++) {

		}
		// Optional: Stop the timer if you want to halt further operation
		RobotSystem_SetEnablePin(robotSystem, true);
	}
}

void RobotSystem_ImuInterruptHandler(RobotSystem *robotSystem, uint16_t GPIO_Pin) {
	IMU_InterruptHandler(&robotSystem->imu, GPIO_Pin);
}

void RobotSystem_GetAccelVals(RobotSystem *robotSystem, int16_t *accel) {
	//if(sizeof(accel)/sizeof(accel[0]) != 3) return;
	accel[0] = robotSystem->imu.accel[0];
	accel[1] = robotSystem->imu.accel[1];
	accel[2] = robotSystem->imu.accel[2];
	//memcpy(accel, robotSystem->imu.accel, sizeof(robotSystem->imu.accel));

}

void RobotSystem_GetGyroVals(RobotSystem *robotSystem, int16_t *gyro) {
	//if(sizeof(gyro)/sizeof(gyro[0]) != 3) return;

	gyro[0] = robotSystem->imu.gyro[0];
	gyro[1] = robotSystem->imu.gyro[1];
	gyro[2] = robotSystem->imu.gyro[2];

	//memcpy(gyro, robotSystem->imu.gyro, sizeof(robotSystem->imu.gyro));
}

void RobotSystem_GetTempVals(RobotSystem *robotSystem, int16_t *temp) {
	*temp = robotSystem->imu.temperature;
}

void RobotSystem_ResetWatchdog(RobotSystem *robotSystem) {
	SysDiag_ResetWatchDog(&robotSystem->sys);
}

void RobotSystem_GetBatVolt(RobotSystem *robotSystem, uint16_t *batVolt) {
	SysDiag_GetBatVoltage(&robotSystem->sys, batVolt);
}

void RobotSystem_Shutdown(RobotSystem *robotSystem) {
	SysDiag_Kill(&robotSystem->sys);
}
