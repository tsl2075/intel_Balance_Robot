#include "pid.h"

// 필터 , PID 상태 변수
float dt = 0.005f;
float pitch = 0.0f;
float acc_pitch = 0.0f;
float gyro_y = 0.0f;

float alpha = 0.95f;  // Complementary Filter 계수

// PID 제어기용
float set_point = 0.0f; //목표 0도로 설정
float Kp = 33.0f, Ki = 0.5f, Kd = 0.1f;
float error = 0.0f, prev_error = 0.0f, integral = 0.0f;
//17-0-0


float deadband = 0.2f;  // 데드밴드 범위 (도 단위, 조정 가능)


float Complementary_PID(float ax, float ay, float az, float gyro_y)
{
    float pid_output = 0.0f;

    // 가속도 기반 Pitch 계산
    acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;

/*    float pitch_offset = -0.3f;  // 튜닝 필요
    acc_pitch += pitch_offset;*/


    // 자이로 적분 (각속도 * 시간)
    pitch = alpha * (pitch + gyro_y * dt) + (1 - alpha) * acc_pitch;


    // PID 제어
    error = set_point - pitch;

    // 데드밴드 적용
    if (fabs(error) < deadband)
      {
	  pid_output = 0.0f;
      }
    else
      {
	  // PID 계산 (기존 로직 유지)
	  integral += error * dt;
	  float derivative = (error - prev_error) / dt;
	  prev_error = error;
	  pid_output = Kp * error + Ki * integral + Kd * derivative;

	  // ±1000 클램핑은 유지
	  if (pid_output > 1000.0f) pid_output = 1000.0f;
	  if (pid_output < -1000.0f) pid_output = -1000.0f;
      }

    return pid_output;
}

void PID_Reset(void)
{
    integral = 0.0f;
    prev_error = 0.0f;
}










