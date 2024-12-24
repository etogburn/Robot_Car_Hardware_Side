/*
 * robot_system.h
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#ifndef INC_ROBOT_SYSTEM_H_
#define INC_ROBOT_SYSTEM_H_

#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "motor_control.h" // Include the motor control header for Motor structure
#include "IMU.h"
#include "stm32g4xx_hal.h"
//#include "serial_commands.h"

//values for current limit calculation
#define SHUNT_RESISTOR 20 //milliohms
#define RESISTOR1 1000 //kohms*10
#define RESISTOR2 75 //kohms * 10
#define RESISTORS (RESISTOR1+RESISTOR2)

#define MAX_CURRENT_LIMIT 5

// Define the RobotSystem structure
typedef struct {
    Motor leftWheel;  // Left wheel motor
    Motor rightWheel; // Right wheel motor
    IMU_HandleTypeDef imu;
    GPIO_TypeDef *Enable_Port;  // GPIO Port for motor enable
    uint16_t Enable_Pin;
    DAC_HandleTypeDef *currentLimitDAC;
    uint16_t currentLimitDACChannel;
    uint16_t currentLimit;
    bool motorsEnabled;
} RobotSystem;

// Robot system functions
void RobotSystem_Init(RobotSystem *robotSystem, Motor leftMotorConfig, Motor rightMotorConfig, IMU_HandleTypeDef imuConfig);
void RobotSystem_SetSpeed(RobotSystem *robotSystem, int16_t leftSpeed, int16_t rightSpeed);
void RobotSystem_Enable (RobotSystem *robotSystem);
void RobotSystem_Disable (RobotSystem *robotSystem);
void RobotSystem_SetEnablePin(RobotSystem *robotSystem, bool onOrOff);
void RobotSystem_SetLeftSpeed(RobotSystem *robotSystem, int16_t leftSpeed);
void RobotSystem_SetRightSpeed(RobotSystem *robotSystem, int16_t rightSpeed);
void RobotSystem_Stop(RobotSystem *robotSystem);
void RobotSystem_GetMotorPosition(RobotSystem *robotSystem, int16_t *leftPos, int16_t *rightPos);
void RobotSystem_GetMotorSpeed(RobotSystem *robotSystem, int16_t *leftSpeed, int16_t *rightSpeed);
void RobotSystem_SetCurrentLimit(RobotSystem *robotSystem, uint16_t currentLimit);
void RobotSystem_WheelFaultHandler(RobotSystem *robotSystem);
void RobotSystem_ResetEnablePin(RobotSystem *robotSystem);
void RobotSystem_ImuInterruptHandler(RobotSystem *robotSystem, uint16_t GPIO_Pin);
void RobotSystem_GetAccelVals(RobotSystem *robotSystem, int16_t *accel);
void RobotSystem_GetGyroVals(RobotSystem *robotSystem, int16_t *gyro);
void RobotSystem_GetTempVals(RobotSystem *robotSystem, int16_t *temp);

//void RobotSystem_InterruptHandler(RobotSystem *robotSystem, uint16_t GPIO_Pin);
void RobotSystem_InterruptHandler(RobotSystem *robotSystem, TIM_HandleTypeDef *htim);
void RobotSystem_Calculate(RobotSystem *robotSystem);

#endif /* INC_DRIVE_SYSTEM_H_ */


