/*
 * Motor_control.h
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32g4xx_hal.h"
#include <stdbool.h>

#define PHASE_A_IDX 0
#define PHASE_B_IDX 1
#define PHASE_C_IDX 2

#define PWM_MAX_VAL 1000
#define PWM_MIN_VAL 175 //value just before the motor starts turning.
#define PWM_MAX_CHANGE 800

#define PID_CALC_TIMING 20 //ms
#define MAX_MOTOR_RPM 8000 //RPM*10
#define MIN_MOTOR_RPM 1000 //RPM*10
#define RPM_INPUT_DIVISOR 100
#define GEARBOX_RATIO 44
#define COMMUTATION_SEQ_PER_REV 3
#define HALL_STATES 6

#define SPEED_CALC_HALL_TO_REV (HALL_STATES * COMMUTATION_SEQ_PER_REV * GEARBOX_RATIO/ RPM_INPUT_DIVISOR)

#define HALL_TICKS_PER_REV (HALL_STATES * COMMUTATION_SEQ_PER_REV * GEARBOX_RATIO)

#define MAX_MOTOR_SPEED MAX_MOTOR_RPM
#define MIN_MOTOR_SPEED MIN_MOTOR_RPM

#define OUTPUT_TO_PWM_MULT 1000
#define OUTPUT_MAX_VAL (PWM_MAX_VAL-PWM_MIN_VAL)*OUTPUT_TO_PWM_MULT

#define MAX_INTEGRAL_VAL 2500
#define DER_FILTER_GAIN 80


typedef struct {
    int64_t hallCount;
    int64_t lastHallCount;
    uint32_t lastHallTime;
    uint32_t totalHallTime;
    int64_t set_speed; //target speed for a given loop
	int16_t pwmVal;                 // Current motor speed
	int16_t lastPwmVal;
	uint32_t lastUpdateTime;
	int64_t integral;
	int64_t maxIntegral;
	int64_t previousError;
	int64_t previousFilter;
	int64_t filterGain;
	int32_t Kp; // Proportional gain
	int32_t Ki; // Integral gain
	int32_t Kd; // Derivative gain
} PID;

// Define a Motor structure
typedef struct {
	uint16_t Hall1_Channel;
	uint16_t Hall2_Channel;
	uint16_t Hall3_Channel;
	GPIO_TypeDef *Fault_Port;  // GPIO Port for Hall Sensor 1
	uint16_t Fault_Pin;
    GPIO_TypeDef *Hall1_Port;  // GPIO Port for Hall Sensor 1
    uint16_t Hall1_Pin;        // GPIO Pin for Hall Sensor 1
    GPIO_TypeDef *Hall2_Port;  // GPIO Port for Hall Sensor 2
    uint16_t Hall2_Pin;        // GPIO Pin for Hall Sensor 2
    GPIO_TypeDef *Hall3_Port;  // GPIO Port for Hall Sensor 3
    uint16_t Hall3_Pin;        // GPIO Pin for Hall Sensor 3
    TIM_HandleTypeDef *HallTimer;
    TIM_HandleTypeDef *Timer;  // Timer for PWM control
    uint16_t phaseChannel[3];
    uint8_t commutationOrder[6];
    uint8_t hallState;
    int16_t distance;
    bool isDirInverted;
    bool direction;
    int64_t target_speed; //final target speed
    int64_t current_speed;
	int64_t acceleration;     //rpm change per pid loop
    PID pid;
} Motor;


// Motor control functions
void Motor_Init(Motor *motor);
void Motor_SetSpeed(Motor *motor, int64_t duty_cycle);
void Motor_SetPwm(Motor *motor, int16_t duty_cycle);
void Motor_Stop(Motor *motor);
void Motor_Update(Motor *motor);
void Motor_Calculate(Motor *motor);
void Motor_GetDistance(Motor *motor, int16_t *distance);
void Motor_GetSpeed(Motor *motor, int16_t *speed);

void SetPhases(Motor *motor, uint8_t highPhase, uint8_t lowPhase, bool direction);
void SetPhase(Motor *motor, uint8_t phase,  bool OnorOff, bool HighOrLow);

void ReadHallSensors(Motor *motor);
void CalculateHallTiming(Motor *motor);
void CalculateDistance(Motor *motor);


#endif /* INC_MOTOR_CONTROL_H_ */
