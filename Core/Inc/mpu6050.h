
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include <math.h>


void MPU6050_Calibrate_Gyro(void);
// 초기화 함수
void MPU6050_Init(void);

// 가속도 읽기
void MPU6050_Read_Accel(float* ax, float* ay, float* az);

// 자이로 읽기
void MPU6050_Read_Gyro(float* gx, float* gy, float* gz);

// Pitch/Roll 계산 함수 (가속도 기반)
void MPU6050_Calculate_PitchRoll(float ax, float ay, float az, float* pitch, float* roll);



#endif /* INC_MPU6050_H_ */
