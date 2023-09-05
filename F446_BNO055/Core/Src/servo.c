/*
 * servo.c
 *
 *  Created on: Jul 14, 2023
 *      Author: baske
 */

#include "main.h"
#include "servo.h"

static uint8_t freqError = 0;

#define getFreq(System_CLK, Timer_Prescaler, Timer_Period)		(System_CLK / (Timer_Prescaler+1)) / (Timer_Period+1)

static inline uint32_t map(int8_t x, int8_t x_min, int8_t x_max, uint32_t out_min, uint32_t out_max){

	if(x <= x_min)
		return x_min;

	if(x >= x_max)
		return x_max;

	return (x - x_min) * (out_max - out_min) / (x_max - x_min) + out_min;

}

SERVO_ErrorTypeDef SERVO_INIT(SERVO_Instance_t* Servo_Instance){

	uint16_t timFreq;

	/* Check if this timer is setup correctly for SERVO Control */
	timFreq = getFreq(HAL_RCC_GetSysClockFreq(), Servo_Instance->htim->Init.Prescaler, __HAL_TIM_GET_AUTORELOAD(Servo_Instance->htim));

	if(timFreq != DESIRE_SERVO_FREQ){
		freqError = 1;
		return SERVO_FREQ_ERROR;
	}

	/* Configure Min and Max count based on duty cycle given */
	Servo_Instance->minCnt = Servo_Instance->config->minDuty * __HAL_TIM_GET_AUTORELOAD(Servo_Instance->htim);
	Servo_Instance->maxCnt = Servo_Instance->config->maxDuty * __HAL_TIM_GET_AUTORELOAD(Servo_Instance->htim);

	/* Zero Out SERVO */
	DRIVE_SERVO(Servo_Instance, 0);

	return SERVO_OK;

}


SERVO_ErrorTypeDef DRIVE_SERVO(const SERVO_Instance_t* servo, int8_t angle){

	/* Servo Protection */
	if(freqError)
		return SERVO_FREQ_ERROR;

	/* Asserting Params */
	if(servo == NULL)
		return SERVO_INSTANCE_ERROR;
	else if(angle < servo->config->minAngle)
		return SERVO_RANGE_ERROR_MIN;
	else if(angle > servo->config->maxAngle)
		return SERVO_RANGE_ERROR_MAX;

	/* Set New Compare Value */
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, map(angle, servo->config->minAngle, servo->config->maxAngle, servo->minCnt, servo->maxCnt));

	return SERVO_OK;
}
