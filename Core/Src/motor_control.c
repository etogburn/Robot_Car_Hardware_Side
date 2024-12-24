/*
 * Motor_control.c
 *
 *  Created on: Nov 15, 2024
 *      Author: etogb
 */

#include "motor_control.h"

// Initialize a motor
void Motor_Init(Motor *motor) {
    // Example: Initialize GPIOs or timers for the motor
	motor->pid.hallCount = 0;
	motor->pid.lastHallCount = 0;
	motor->pid.set_speed = 0;
	motor->pid.integral = 0;
	motor->pid.previousError = 0;
	motor->pid.previousFilter = 0;
	motor->pid.lastUpdateTime = 0;
	motor->pid.lastPwmVal = 0;
	motor->pid.lastHallTime = 0;
	motor->pid.totalHallTime = 0;
	motor->pid.pwmVal = 0;
	motor->pid.filterGain = DER_FILTER_GAIN;
	motor->pid.maxIntegral = MAX_INTEGRAL_VAL;
	motor->isFault = false;
	motor->faultRecovery = false;


	HAL_TIM_IC_Start_IT(motor->HallTimer, motor->Hall1_Channel);
	HAL_TIM_IC_Start_IT(motor->HallTimer, motor->Hall2_Channel);
	HAL_TIM_IC_Start_IT(motor->HallTimer, motor->Hall3_Channel);

	Motor_Stop(motor);
	ReadHallSensors(motor);
}

// Set the speed of a motor
void Motor_SetSpeed(Motor *motor, int64_t target_speed) {

	if (target_speed == motor->target_speed) return;

	if (target_speed > MAX_MOTOR_SPEED) target_speed = MAX_MOTOR_SPEED;
	if (target_speed < -MAX_MOTOR_SPEED) target_speed = -MAX_MOTOR_SPEED;
	if (target_speed > -MIN_MOTOR_SPEED && target_speed < MIN_MOTOR_SPEED)
			target_speed = 0;

	//motor->pid.integral = 0;
	//motor->pid.previousError = 0;

	motor->target_speed = target_speed;

	Motor_Calculate(motor);
}


void Motor_SetPwm(Motor *motor, int16_t duty_cycle) {

	if(duty_cycle > PWM_MAX_VAL) duty_cycle = PWM_MAX_VAL;
	else if(duty_cycle < 0) duty_cycle = 0;

	motor->pid.pwmVal = duty_cycle;

	__HAL_TIM_SET_COMPARE(motor->Timer, motor->phaseChannel[PHASE_A_IDX], motor->pid.pwmVal);
	__HAL_TIM_SET_COMPARE(motor->Timer, motor->phaseChannel[PHASE_B_IDX], motor->pid.pwmVal);
	__HAL_TIM_SET_COMPARE(motor->Timer, motor->phaseChannel[PHASE_C_IDX], motor->pid.pwmVal);

	if(motor->pid.lastPwmVal == 0 && motor->pid.lastPwmVal != motor->pid.pwmVal)
		Motor_Update(motor);
	motor->pid.lastPwmVal = motor->pid.pwmVal;

}

// Stop a motor
void Motor_Stop(Motor *motor) {
    // Set speed to zero to stop the motor
    Motor_SetSpeed(motor, 0);
    SetPhase(motor, PHASE_A_IDX, false, false);
    SetPhase(motor, PHASE_B_IDX, false, false);
    SetPhase(motor, PHASE_C_IDX, false, false);
}

void Motor_Calculate(Motor *motor) {
	uint32_t currentTime = HAL_GetTick(); // Get the current time in milliseconds
	if(Motor_GetFaultStatus(motor)) {

	}
	// Update PID every certain time cycle
	if ((currentTime - motor->pid.lastUpdateTime) >= PID_CALC_TIMING) {

		if(motor->faultRecovery) {
			Motor_CurrFaultHandler(motor);
			return;
		}
		CalculateDistance(motor);
		if(motor->pid.set_speed > motor->target_speed) {
			if(motor->pid.set_speed > 0 && motor->pid.set_speed - motor->acceleration < 0 && motor->target_speed < 0) {
				if(motor->current_speed > motor->pid.set_speed + motor->acceleration) {
					//waiting for the system to slow down enough to change directions
				}
				else {
					motor->pid.set_speed = 0;
				}
			}
			else if(motor->pid.set_speed - motor->acceleration <= motor->target_speed) {
				motor->pid.set_speed = motor->target_speed;
			}
			else {
				motor->pid.set_speed -= motor->acceleration;
			}
			motor->pid.integral = 0;
		} else if(motor->pid.set_speed < motor->target_speed) {
			if(motor->pid.set_speed < 0 && motor->pid.set_speed + motor->acceleration > 0 && motor->target_speed > 0) {
				if(motor->current_speed < motor->pid.set_speed - motor->acceleration) {
					//waiting for the system to slow down enough to change directions
				}
				else {
					motor->pid.set_speed = 0;
				}
			}
			else if(motor->pid.set_speed + motor->acceleration >= motor->target_speed) {
				motor->pid.set_speed = motor->target_speed;
			}
			else {
				motor->pid.set_speed += motor->acceleration;
			}
			motor->pid.integral = 0;
		}

		//set motor direction based the sign of the set_speed variable
		if(motor->pid.set_speed < 0) {
			motor->direction = false;
		}
		else
		{
			motor->direction = true;
		}

		// Calculate current speed
		motor->current_speed = (motor->pid.hallCount - motor->pid.lastHallCount)
				* ( 1000 * 1000 * 60 / ((int64_t)motor->pid.totalHallTime))/SPEED_CALC_HALL_TO_REV; // speed in counts per minute

		motor->pid.totalHallTime = 0;
		motor->pid.lastHallCount = motor->pid.hallCount;

		//do no calculations if speed is set to 0
		if(motor->pid.set_speed == 0) {
			Motor_SetPwm(motor,0);
			return;
		}

		// Calculate error
		int64_t error = (motor->pid.set_speed - motor->current_speed);
		// PID calculations
		motor->pid.integral += error * PID_CALC_TIMING/1000; // Integral term, scaled for 50ms

		//clamp integral term to avoid integral windup
		if(motor->pid.integral > motor->pid.maxIntegral) motor->pid.integral = motor->pid.maxIntegral;
		if(motor->pid.integral < -1*motor->pid.maxIntegral) motor->pid.integral = -1*motor->pid.maxIntegral;

		//use derivitave filter to calculate derivative term
		int64_t currentFilter = motor->pid.filterGain * motor->pid.previousFilter/100
								+ (100-motor->pid.filterGain)*(error - motor->pid.previousError)/100;

		int64_t derivative = (currentFilter) * 1000/PID_CALC_TIMING; // Derivative term
		motor->pid.previousFilter = currentFilter;
		motor->pid.previousError = error;

		// Compute the new PWM value using the PID formula
		int64_t output = motor->pid.Kp * error + motor->pid.Ki * motor->pid.integral + motor->pid.Kd * derivative;

		// Clamp the output to the valid PWM range
		if(motor->pid.set_speed > 0) {
			if (output > OUTPUT_MAX_VAL) output = OUTPUT_MAX_VAL;
			if (output < 0) output = 0;
		}
		else if(motor->pid.set_speed < 0) {
			if (output < -OUTPUT_MAX_VAL) output = -OUTPUT_MAX_VAL;
			if (output > 0) output = 0;
			output*=-1;
		}
		else {
			output = 0;
		}

		//shift the output itself to avoid the deadband range of the motor.
		//output is also divided by a factor to allow for more granular kp, ki, and kd values
		output = output/OUTPUT_TO_PWM_MULT + PWM_MIN_VAL;
		if(output - motor->pid.pwmVal > PWM_MAX_CHANGE) output = motor->pid.pwmVal + PWM_MAX_CHANGE;
		if(motor->pid.pwmVal - output > PWM_MAX_CHANGE) output = motor->pid.pwmVal - PWM_MAX_CHANGE;
		// Apply the PWM value
		Motor_SetPwm(motor, (int16_t)output);

		// Update the last update time
		motor->pid.lastUpdateTime = currentTime;
	}
}

// Update the motor state (optional, e.g., for monitoring or control logic)
void Motor_Update(Motor *motor) {

	CalculateHallTiming(motor);

	ReadHallSensors(motor);

	bool direction = motor->isDirInverted ? !motor->direction : motor->direction;

	if(motor->hallState == motor->commutationOrder[0]) {
		SetPhases(motor, PHASE_A_IDX, PHASE_C_IDX, direction);
	}
	else if(motor->hallState == motor->commutationOrder[1]) {
		SetPhases(motor, PHASE_A_IDX, PHASE_B_IDX, direction);
	}
	else if(motor->hallState == motor->commutationOrder[2]) {
		SetPhases(motor, PHASE_C_IDX, PHASE_B_IDX, direction);
	}
	else if(motor->hallState == motor->commutationOrder[3]) {
		SetPhases(motor, PHASE_C_IDX, PHASE_A_IDX, direction);
	}
	else if(motor->hallState == motor->commutationOrder[4]) {
		SetPhases(motor, PHASE_B_IDX, PHASE_A_IDX, direction);
	}
	else if(motor->hallState == motor->commutationOrder[5]) {
		SetPhases(motor, PHASE_B_IDX, PHASE_C_IDX, direction);
	}
}

void SetPhases(Motor *motor, uint8_t highPhase, uint8_t lowPhase, bool direction) {
	if(highPhase > PHASE_C_IDX || lowPhase > PHASE_C_IDX) return;
	if(highPhase == lowPhase) return;

	uint8_t extraPhase = PHASE_A_IDX + PHASE_B_IDX + PHASE_C_IDX - highPhase - lowPhase;

	SetPhase(motor, extraPhase, false, false);

	SetPhase(motor, highPhase, true, direction);
	SetPhase(motor, lowPhase, true, !direction);

}

void SetPhase(Motor *motor, uint8_t phase,  bool OnorOff, bool HighOrLow) {
	if(OnorOff) {
		if(HighOrLow) {
			HAL_TIMEx_PWMN_Stop(motor->Timer, motor->phaseChannel[phase]);
			HAL_TIM_PWM_Start(motor->Timer, motor->phaseChannel[phase]);
		} else {
			HAL_TIM_PWM_Stop(motor->Timer, motor->phaseChannel[phase]);
			HAL_TIMEx_PWMN_Start(motor->Timer, motor->phaseChannel[phase]);
		}
	} else {
		HAL_TIM_PWM_Stop(motor->Timer, motor->phaseChannel[phase]);
		HAL_TIMEx_PWMN_Stop(motor->Timer, motor->phaseChannel[phase]);
	}
}

void ReadHallSensors(Motor *motor) {
	motor->hallState = (HAL_GPIO_ReadPin(motor->Hall1_Port, motor->Hall1_Pin))
					| (HAL_GPIO_ReadPin(motor->Hall2_Port, motor->Hall2_Pin)) << 1
					| (HAL_GPIO_ReadPin(motor->Hall3_Port, motor->Hall3_Pin) << 2);
}


void CalculateHallTiming(Motor *motor) {
	uint32_t currentMicroTime = 0;

	if(motor->HallTimer->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		currentMicroTime = HAL_TIM_ReadCapturedValue(motor->HallTimer, motor->Hall1_Channel);
	} else if(motor->HallTimer->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		currentMicroTime = HAL_TIM_ReadCapturedValue(motor->HallTimer, motor->Hall2_Channel);
	} else if (motor->HallTimer->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		currentMicroTime = HAL_TIM_ReadCapturedValue(motor->HallTimer, motor->Hall3_Channel);
	}

	motor->pid.totalHallTime += currentMicroTime - motor->pid.lastHallTime;

	motor->pid.lastHallTime = currentMicroTime;

	if(motor->direction) motor->pid.hallCount++;
	else motor->pid.hallCount--;
}

void CalculateDistance(Motor *motor) {
	motor->distance = (int16_t)(motor->pid.hallCount & 0xFFFF); // * RPM_INPUT_DIVISOR / HALL_TICKS_PER_REV);
}

void Motor_GetDistance(Motor *motor, int16_t *distance) {
	*distance = motor->distance;
}

void Motor_GetSpeed(Motor *motor, int16_t *speed) {
	*speed = motor->current_speed;
}

bool Motor_GetFaultStatus(Motor *motor) {
	motor->isFault = !HAL_GPIO_ReadPin(motor->Fault_Port, motor->Fault_Pin);
	if(motor->isFault) motor->faultRecovery = true;
	return motor->isFault;
}

void Motor_CurrFaultHandler(Motor *motor) {
	if(motor->faultRecovery) {
		//do some stuff. right now. going to just reset and continue normally.
		motor->faultRecovery = false;
	}
}


