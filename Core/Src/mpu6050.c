
#include "mpu6050.h"


#define MPU6050_ADDR     (0x68 << 1)  // AD0 = GND → 0x68 (HAL은 8비트 주소 사용)
float gyro_bias_y = 0.0f;
extern I2C_HandleTypeDef hi2c1;  // I2C1 핸들러 사용


void MPU6050_Calibrate_Gyro(void)
{
    float sum_gy = 0.0f;
    int num_samples = 100;
    for (int i = 0; i < num_samples; i++)
      {
        float gx, gy, gz;
        MPU6050_Read_Gyro(&gx, &gy, &gz);
        sum_gy += gy;
        HAL_Delay(10); // 10ms 간격으로 샘플링
      }
    gyro_bias_y = sum_gy / num_samples;
}

void MPU6050_Init(void)
{
    uint8_t data;

    // 1. Power Management 1 (레지스터 0x6B) - 슬립 해제
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);

    // 2. Sample Rate Divider (레지스터 0x19) - 샘플링 주기 설정 (1kHz / (1 + 7) = 125Hz)
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &data, 1, 100);

    // 3. Configuration (레지스터 0x1A) - 저역통과필터 (DLPF_CFG = 3 → 약 44Hz 대역폭)
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &data, 1, 100);

    // 4. Gyro Configuration (레지스터 0x1B) - ±250°/s 범위 (값 = 0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);

    // 5. Accel Configuration (레지스터 0x1C) - ±2g 범위 (값 = 0x00)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);
}


void MPU6050_Read_Accel(float* ax, float* ay, float* az)
{
  float last_ax = 0.0f, last_ay = 0.0f, last_az = 0.0f;
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buf, 6, 100) == HAL_OK)
      {
        int16_t raw_ax = (int16_t)(buf[0] << 8 | buf[1]);
        int16_t raw_ay = (int16_t)(buf[2] << 8 | buf[3]);
        int16_t raw_az = (int16_t)(buf[4] << 8 | buf[5]);
        *ax = raw_ax / 16384.0f;
        *ay = raw_ay / 16384.0f;
        *az = raw_az / 16384.0f;
        last_ax = *ax;
        last_ay = *ay;
        last_az = *az;
      }
    else
      {
        *ax = last_ax;
        *ay = last_ay;
        *az = last_az;
      }
}



void MPU6050_Read_Gyro(float* gx, float* gy, float* gz)
{
  float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
    uint8_t buf[6];
    int16_t raw_gx, raw_gy, raw_gz;

    // GYRO_XOUT_H = 0x43
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buf, 6, 100) == HAL_OK)
    {
        raw_gx = (int16_t)(buf[0] << 8 | buf[1]);
        raw_gy = (int16_t)(buf[2] << 8 | buf[3]);
        raw_gz = (int16_t)(buf[4] << 8 | buf[5]);

        // ±250°/s 범위일 때 정규화: 131 LSB = 1°/s
        *gx = raw_gx / 131.0f;
        *gy = (raw_gy / 131.0f) - gyro_bias_y; // 바이어스 보정
        *gz = raw_gz / 131.0f;
        last_gx = *gx;
	last_gy = *gy;
	last_gz = *gz;
      }
    else
      {
	  *gx = last_gx;
	  *gy = last_gy;
	  *gz = last_gz;
      }
}


// Pitch/Roll 계산 함수 (가속도 기반)
void MPU6050_Calculate_PitchRoll(float ax, float ay, float az, float* pitch, float* roll)
{
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    *roll  = atan2f(ay, az) * 180.0f / M_PI;
}



























