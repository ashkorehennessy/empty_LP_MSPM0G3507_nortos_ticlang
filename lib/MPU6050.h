#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "ti_msp_dl_config.h"
#include "I2C.h"
#include "MPU6050_Reg.h"

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;
    double Old_Gz;

    float Temperature;

    double KalmanAngleX_Old;
    double KalmanAngleY_Old;
    double KalmanAngleX;
    double KalmanAngleY;
    double AngleZ;
} MPU6050_t;

extern MPU6050_t mpu6050;
void MPU6050_Init(void);

uint8_t MPU6050_GetID(void);

void MPU6050_Read_All(MPU6050_t *DataStruct);

#endif

