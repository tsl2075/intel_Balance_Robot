/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include <stdio.h>
#include "nrf24.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */
/* Definitions for BalanceTask */
osThreadId_t BalanceTaskHandle;
const osThreadAttr_t BalanceTask_attributes = {
  .name = "BalanceTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for nRF24Task */
osThreadId_t nRF24TaskHandle;
const osThreadAttr_t nRF24Task_attributes = {
  .name = "nRF24Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBalanceTask(void *argument);
void StartnRF24Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BalanceTask */
  BalanceTaskHandle = osThreadNew(StartBalanceTask, NULL, &BalanceTask_attributes);

  /* creation of nRF24Task */
  nRF24TaskHandle = osThreadNew(StartnRF24Task, NULL, &nRF24Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBalanceTask */
/**
  * @brief  Function implementing the BalanceTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBalanceTask */
void StartBalanceTask(void *argument)
{
  /* USER CODE BEGIN StartBalanceTask */

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(5);

  float ax, ay, az, gx, gy, gz;
  float output;
  float roll;

  /* Infinite loop */
  for(;;)
  {
      // 1. 센서 데이터 읽기
      MPU6050_Read_Accel(&ax, &ay, &az);
      MPU6050_Read_Gyro(&gx, &gy, &gz);

      // 2. 필터 + PID 계산
      output = Complementary_PID(ax, ay, az, gy);


      //  15도 이상이면 정지
      if (fabs(pitch) > 70.5f)
	{
          Set_Motor_PWM(0);  // 넘어진 것으로 판단 → 정지
          PID_Reset();
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
          continue;  // 다음 루프로
	}

      // 3. 모터 제어
      Set_Motor_PWM(output);

      // 5. 주기 유지
      vTaskDelayUntil(&xLastWakeTime, xFrequency);


      MPU6050_Calculate_PitchRoll(ax, ay, az, &pitch, &roll);

      //printf("%.2f,%.2f\n", pitch, roll);  // 쉼표로 구분해서 출력 (Processing에서 읽기 좋게)
      printf("pitch: %.2f, output: %.2f\n", pitch, output);
      //printf("%.2f,%.2f\n", pitch, output); //processing용 프린트
  }
  /* USER CODE END StartBalanceTask */
}

/* USER CODE BEGIN Header_StartnRF24Task */
/**
* @brief Function implementing the nRF24Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartnRF24Task */
void StartnRF24Task(void *argument)
{
  /* USER CODE BEGIN StartnRF24Task */

  /* Infinite loop */
  for(;;)
  {

  }
  /* USER CODE END StartnRF24Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

