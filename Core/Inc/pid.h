


#ifndef INC_PID_H_
#define INC_PID_H_


#include "main.h"
#include "motor.h"
#include "i2c.h"
#include "tim.h"
#include "mpu6050.h"


extern float dt;
extern float pitch, acc_pitch, gyro_y;
extern float alpha;

extern float set_point;
extern float Kp, Ki, Kd;
extern float error, prev_error, integral;






float Complementary_PID(float ax, float ay, float az, float gyro_y);
void PID_Reset(void);




#endif  /* INC_PID_H_ */

