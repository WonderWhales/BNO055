/*
 * servo.h
 *
 *  Created on: Jul 14, 2023
 *      Author: baske
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define DESIRE_SERVO_FREQ		(50U)

typedef enum{

	SERVO_OK 		 		= 0x00U,
	SERVO_FREQ_ERROR 		= 0x01U,
	SERVO_RANGE_ERROR_MIN	= 0x02U,
	SERVO_RANGE_ERROR_MAX	= 0x03U,
	SERVO_INSTANCE_ERROR	= 0x04U

} SERVO_ErrorTypeDef;

typedef struct{

	float minDuty;
	float maxDuty;
	int8_t minAngle;
	int8_t maxAngle;

} SERVO_Config_t;

typedef struct{

	TIM_HandleTypeDef*	htim;
	uint8_t				channel;
	SERVO_Config_t*		config;

	/* These will be set in the INIT function */
	uint32_t 			minCnt;
	uint32_t 			maxCnt;

} SERVO_Instance_t;

SERVO_ErrorTypeDef SERVO_INIT(SERVO_Instance_t* Servo_Instance);
SERVO_ErrorTypeDef DRIVE_SERVO(const SERVO_Instance_t* servo, int8_t angle);

#endif /* INC_SERVO_H_ */
