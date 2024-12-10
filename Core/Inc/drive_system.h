/*
 * drive_system.h
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#ifndef INC_DRIVE_SYSTEM_H_
#define INC_DRIVE_SYSTEM_H_

#include <stdbool.h>
#include "main.h"
#include "motor_control.h" // Include the motor control header for Motor structure
#include "stm32g4xx_hal.h"
//#include "serial_commands.h"

//values for current limit calculation
#define SHUNT_RESISTOR 20 //milliohms
#define RESISTOR1 1000 //kohms*10
#define RESISTOR2 75 //kohms * 10
#define RESISTORS (RESISTOR1+RESISTOR2)

#define MAX_CURRENT_LIMIT 10

// Define the DriveSystem structure
typedef struct {
    Motor leftWheel;  // Left wheel motor
    Motor rightWheel; // Right wheel motor
    GPIO_TypeDef *Enable_Port;  // GPIO Port for Hall Sensor 1
    uint16_t Enable_Pin;
    DAC_HandleTypeDef *currentLimitDAC;
    uint16_t currentLimitDACChannel;
    uint16_t currentLimit;
} DriveSystem;

// Drive system functions
void DriveSystem_Init(DriveSystem *driveSystem, Motor leftMotorConfig, Motor rightMotorConfig);
void DriveSystem_SetSpeed(DriveSystem *driveSystem, int16_t leftSpeed, int16_t rightSpeed);
void DriveSystem_Enable (DriveSystem *driveSystem);
void DriveSystem_Disable (DriveSystem *driveSystem);
void DriveSystem_SetEnablePin(DriveSystem *driveSystem, bool onOrOff);
void DriveSystem_SetLeftSpeed(DriveSystem *driveSystem, int16_t leftSpeed);
void DriveSystem_SetRightSpeed(DriveSystem *driveSystem, int16_t rightSpeed);
void DriveSystem_Stop(DriveSystem *driveSystem);

void DriveSystem_SetCurrentLimit(DriveSystem *driveSystem, uint16_t currentLimit);

//void DriveSystem_InterruptHandler(DriveSystem *driveSystem, uint16_t GPIO_Pin);
void DriveSystem_InterruptHandler(DriveSystem *driveSystem, TIM_HandleTypeDef *htim);
void DriveSystem_Calculate(DriveSystem *driveSystem);

#endif /* INC_DRIVE_SYSTEM_H_ */


