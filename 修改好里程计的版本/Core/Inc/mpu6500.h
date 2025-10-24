#ifndef __MPU6500_H
#define __MPU6500_H

#include "stm32f4xx_hal.h"

#define MPU6500_ADDR         (0x68 << 1)  // HAL库I2C地址需要左移1位
#define MPU6500_ACCEL_REG    0x3B

// 原始数据+物理单位数据结构体
typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    float   Accel_X_mps2;
    float   Accel_Y_mps2;
    float   Accel_Z_mps2;
    float   Gyro_X_dps;
    float   Gyro_Y_dps;
    float   Gyro_Z_dps;
} MPU6500_Data;

typedef struct {
    int16_t ax_offset, ay_offset, az_offset;
    int16_t gx_offset, gy_offset, gz_offset;
    float velocity_x;
    float distance_x;
    float dt;
    uint8_t calibrated;
    // 滤波器状态
    float accel_x_filt, accel_y_filt, accel_z_filt;
    float gyro_x_filt, gyro_y_filt, gyro_z_filt;
} MPU6500_Calib;

uint8_t MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_Calibrate(I2C_HandleTypeDef *hi2c, MPU6500_Calib *calib);
uint8_t MPU6500_Read_All(I2C_HandleTypeDef *hi2c, MPU6500_Data *data);
void MPU6500_GetCalibratedData(MPU6500_Data *raw, MPU6500_Calib *calib, MPU6500_Data *out);
void MPU6500_Convert_Unit(MPU6500_Data *data);
void MPU6500_LowPassFilter(MPU6500_Data *data, MPU6500_Calib *calib, float alpha);
void MPU6500_Update_XMotion(MPU6500_Data *calib_data, MPU6500_Calib *calib);

#endif 