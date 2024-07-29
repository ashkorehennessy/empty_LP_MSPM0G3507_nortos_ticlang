#ifndef __IMU_H__
#define __IMU_H__

#include "ti_msp_dl_config.h"
#include "math.h"
#include "MPU6050.h"
		
typedef struct
{
	float AX;
	float AY;
	float AZ;
	float GX;
	float GY;
	float GZ;
}param_imu;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}param_Angle;

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

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;
//extern MPU6050_t mpu;
// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

extern param_Angle imu_Angle;	
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt,int n);
void MPU6050_Read_All(MPU6050_t *DataStruct,int n);						
void IMU_getEuleranAngles(void);
							
#endif