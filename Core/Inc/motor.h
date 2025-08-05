

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "mpu6050.h"
#include "pid.h"
#include <math.h>
#include "i2c.h"
#include "tim.h"

#define MPU6050_ADDR (0x68 << 1) // HAL 라이브러리에서는 주소 << 1




void Set_Motor_PWM(float pid_output);
void Stop_Motors(void);
void Left_Motor_Forward(int pwm);
void Left_Motor_Backward(int pwm);
void Right_Motor_Forward(int pwm);
void Right_Motor_Backward(int pwm);





#endif  /* INC_MOTOR_H_*/

