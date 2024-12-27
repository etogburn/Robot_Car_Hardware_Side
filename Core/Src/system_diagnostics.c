/*
 * system_diagnostics.c
 *
 *  Created on: Dec 27, 2024
 *      Author: etogb
 */

#include "system_diagnostics.h"

void SysDiag_Init(SystemDiagnostics_t *sys) {
	sys->lastSleepReset = 0;
}

void SysDiag_SleepWatchdog(SystemDiagnostics_t *sys) {

	if(sys->sleepTimeout < 1000) return;

	uint32_t currentTime = HAL_GetTick();

	if(currentTime - sys->lastSleepReset > sys->sleepTimeout) {
		SysDiag_Kill(sys);
	}
}


void SysDiag_Kill(SystemDiagnostics_t *sys) {
	HAL_Delay(250);
	HAL_GPIO_WritePin(sys->Mcm_Kill_Port, sys->Mcm_Kill_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
}

void SysDiag_ResetWatchDog(SystemDiagnostics_t *sys) {
	sys->lastSleepReset = HAL_GetTick();
}

void SysDiag_GetBatVoltage(SystemDiagnostics_t *sys, uint16_t *batVoltage) {

    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure the ADC channel
    sConfig.Channel = sys->batAdcChannel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;

    HAL_ADC_ConfigChannel(sys->batAdc, &sConfig);

	HAL_ADC_Start(sys->batAdc);

	HAL_ADC_PollForConversion(sys->batAdc, HAL_MAX_DELAY);

	uint32_t adcVal = HAL_ADC_GetValue(sys->batAdc);

	HAL_ADC_Stop(sys->batAdc);

	uint32_t tempVolt = adcVal * VBAT_ADC_SLOPE / VBAT_ADC_SCALE + VBAT_ADC_INTERSECT;

	*batVoltage = tempVolt;
}


