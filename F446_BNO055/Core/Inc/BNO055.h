/*
 * BNO055.h
 *
 *  Created on: Sep 4, 2023
 *      Author: baske
 */

#ifndef INC_BNO055_H_
#define INC_BNO055_H_

#include "main.h"

typedef enum{
	BNO055_SUCCESS        = 0U,
	BNO055_NOT_DETECTED   = 1U,
} BNO055_ERROR;

typedef enum{
	CONFIG,
	ACC_ONLY,
	MAG_ONLY,
	GYRO_ONLY,
	ACC_MAG,
	ACC_GYRO,
	MAG_GYRO,
	AMG,
	IMU,
	COMPASS,
	M4G,
	NDOF_FMC_OFF,
	NDOF
} BNO055_OPERATION_MODE;

typedef enum{
	METER_PER_SEC_SQUARD,
	MILLI_G
} BNO055_ACCEL_GRAV_UNIT;

typedef enum{
	Dps,
	Rps
} BNO055_ANGULAR_RATE_UNIT;

typedef enum{
	DEGREES,
	RADIANS
} BNO055_EULER_UNIT;

typedef enum{
	CELCIUS,
	FAHRENHEIT
} BNO055_TEMP_UNIT;

typedef struct{
	float x;
	float y;
	float z;
} BNO055_Euler_Vec_t;

typedef struct{
	float w;
	float x;
	float y;
	float z;
} BNO055_Quad_Vec_t;

//#define BNO055_HARDWARE_RESET

void BNO055_I2C_Mount(I2C_HandleTypeDef* i2c);

#ifdef BNO055_HARDWARE_RESET
	void BNO055_HW_Reset_Mount(uint32_t GPIO_Port, uint16_t GPIO_Pin);
#endif

BNO055_ERROR BNO055_Init(void);
BNO055_ERROR BNO055_Set_Unit(BNO055_ACCEL_GRAV_UNIT accUnit, BNO055_ANGULAR_RATE_UNIT angRateUnit, BNO055_EULER_UNIT eulerUnit, BNO055_TEMP_UNIT tempUnit);
BNO055_ERROR BNO055_Set_OP_Mode(BNO055_OPERATION_MODE op);
BNO055_ERROR BNO055_Get_Euler_Vec(BNO055_Euler_Vec_t* vec);

#endif /* INC_BNO055_H_ */
