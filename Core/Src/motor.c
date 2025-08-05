
#include <motor.h>

extern TIM_HandleTypeDef htim3;  // PWM 타이머
#define MIN_PWM 100  // 모터가 움직이기 시작하는 최소 PWM 값 (실험으로 확인 필요)
#define MAX_PWM 999            // PWM 최대값 (타이머 Period 기준)

void Set_Motor_PWM(float pid_output)
{
    int pwm = (int)fabsf(pid_output);
    // PID 출력이 0이 아닌 경우, PWM을 최소값 이상으로 보정
    if (pid_output != 0 && pwm < MIN_PWM)
      {
	pwm = MIN_PWM;
      }
    // 최대 PWM 값 초과 방지
    if (pwm > MAX_PWM)
      {
	pwm = MAX_PWM;
      }

    if (pid_output > 0)
      {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
      }

    else if (pid_output < 0)
      {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // AIN1 (왼쪽 전진)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // AIN2
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // BIN1 (오른쪽 전진)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // BIN2
      }

    else
      {
        // 정지 (PWM = 0)
	// Short Brake
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        pwm = 0;
      }

    // PWM 듀티 설정 (왼쪽/오른쪽 모터)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);  // 왼쪽 모터 (PA6)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);  // 오른쪽 모터 (PA7)
}











