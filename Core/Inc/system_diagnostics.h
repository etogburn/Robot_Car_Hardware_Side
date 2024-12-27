/*
 * system_diagnostics.h
 *
 *  Created on: Dec 27, 2024
 *      Author: etogb
 */

#ifndef INC_SYSTEM_DIAGNOSTICS_H_
#define INC_SYSTEM_DIAGNOSTICS_H_

#include "stm32g4xx_hal.h"

#define VBAT_R1 470 //ohms times 10
#define VBAT_R2 82 //ohms times 10
#define VBAT_R (VBAT_R1 + VBAT_R2)
#define VBAT_ADC_REF 330 //3.3 V
#define VBAT_ADC_MAX 4095

#define VBAT_ADC_SLOPE 108
#define VBAT_ADC_INTERSECT -2130
#define VBAT_ADC_SCALE 100


typedef struct {
	GPIO_TypeDef *Mcm_Kill_Port;  // GPIO Port for current fault
	uint16_t Mcm_Kill_Pin;
	ADC_HandleTypeDef *batAdc;
	uint32_t batAdcChannel;
	uint32_t lastSleepReset;
	uint32_t sleepTimeout;
} SystemDiagnostics_t;

void SysDiag_Init(SystemDiagnostics_t *sys);
void SysDiag_SleepWatchdog(SystemDiagnostics_t *sys);
void SysDiag_Kill(SystemDiagnostics_t *sys);
void SysDiag_ResetWatchDog(SystemDiagnostics_t *sys);
void SysDiag_GetBatVoltage(SystemDiagnostics_t *sys, uint16_t *batVoltage);


#endif /* INC_SYSTEM_DIAGNOSTICS_H_ */
