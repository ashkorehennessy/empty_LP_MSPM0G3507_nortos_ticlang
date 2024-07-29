#include "IMU.h"
#include "extern.h"

#define kp 				20.00f
#define ki 				0.001f
#define cycle_T 		0.005f//200hz
#define half_T 			0.0025f

int16_t Ax,Ay,Az,Gx,Gy,Gz;
param_imu imu_data;
param_Angle imu_Angle;
float q[4] = {1.0,0.0,0.0,0.0};
float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;
//////////////////////////////////////////////////////////////////////////////////
float fast_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);  //0x5f3759df��?��?????��??����1��y?��??����
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
//////////////////////////////////////////////////////////////////////////////////

void IMU_GetValues(void)
{
	MPU6050_GetData(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
	
	imu_data.AX = ((float)Ax)/2048;
	imu_data.AY = ((float)Ay)/2048;
	imu_data.AZ = ((float)Az)/2048;

	imu_data.GX = ((float)Gx)*0.001064;
	imu_data.GY = ((float)Gy)*0.001064;
	imu_data.GZ = ((float)Gz)*0.001064;
}
void IMU_AHRSupdate(param_imu* imu_temp)
{
	float ax,ay,az;
	float gx,gy,gz;
	ax = imu_temp->AX;
	ay = imu_temp->AY;
	az = imu_temp->AZ;
	gx = imu_temp->GX;
	gy = imu_temp->GY;
	gz = imu_temp->GZ;
	float vx, vy, vz; 
	float ex, ey, ez; 
	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];
	float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
	
	vx = 2 * (q1*q3 - q0*q2);
	vy = 2 * (q2*q3 + q0*q1);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
    ex = (ay * vz - az * vy);//  |A|*|B|*sin<A,B>
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
	
	exInt += ki * ex;
	eyInt += ki * ey;
	ezInt += ki * ez;
	gx += kp * ex + exInt;
	gy += kp * ey + eyInt;
	gz += kp * ez + ezInt;
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy)  * half_T;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx)  * half_T;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx)  * half_T;
	norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q[0] = q0 * norm;
	q[1] = q1 * norm;
	q[2] = q2 * norm;
	q[3] = q3 * norm;
	
}

void IMU_getEuleranAngles(void)
{
	IMU_GetValues();
	IMU_AHRSupdate(&imu_data);
	
	imu_Angle.Pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.2957;
	imu_Angle.Roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.2957;
	
	imu_Angle.Yaw  += imu_data.GZ* 57.2957* cycle_T* 4;
	
}   

#define RAD_TO_DEG 57.295779513082320876798154814105
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
double dt[10],roll[10],roll_sqrt[10],pitch[10],rate[10],S[10],K[10][2],P00_temp[10],P01_temp[10],y[10];
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void MPU6050_Read_All(MPU6050_t *DataStruct,int n)
{
    uint8_t Rec_Data[14];
    // int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register
	    MPU6050_GetData(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
    // HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = Ax;
    DataStruct->Accel_Y_RAW = Ay;
    DataStruct->Accel_Z_RAW = Az;
    // temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = Gx;
    DataStruct->Gyro_Y_RAW = Gy;
    DataStruct->Gyro_Z_RAW = Gz;

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    // DataStruct->Ax = DataStruct->Accel_X_RAW /100;
    // DataStruct->Ay = DataStruct->Accel_Y_RAW/100;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    // DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
     dt[n] = (double)(time_system - timer)/10;
    timer = time_system;
//    double roll;
     roll_sqrt[n] = sqrt(pow(DataStruct->Accel_X_RAW, 2) + pow(DataStruct->Accel_Z_RAW, 2));
    if (roll_sqrt[n] != 0.0)
    {
        roll[n] = atan(DataStruct->Accel_Y_RAW / roll_sqrt[n]) * RAD_TO_DEG;
    }
    else
    {
        roll[n] = 0.0;
    }
     pitch[n] = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch[n] < -90 && DataStruct->KalmanAngleY > 90) || (pitch[n] > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch[n];
        DataStruct->KalmanAngleY = pitch[n];
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch[n], DataStruct->Gy, dt[n],n)-2.3;
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll[n], DataStruct->Gx, dt[n],n)+0.85;
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt,int n)
{
     rate[n] = newRate - Kalman->bias;
    Kalman->angle += dt * rate[n];

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

     S[n] = Kalman->P[0][0] + Kalman->R_measure;
//    double K[2];
    K[n][0] = Kalman->P[0][0] / S[n];
    K[n][1] = Kalman->P[1][0] / S[n];

     y[n] = newAngle - Kalman->angle;
    Kalman->angle += K[n][0] * y[n];
    Kalman->bias += K[n][1] * y[n];

     P00_temp[n] = Kalman->P[0][0];
     P01_temp[n] = Kalman->P[0][1];

    Kalman->P[0][0] -= K[n][0] * P00_temp[n];
    Kalman->P[0][1] -= K[n][0] * P01_temp[n];
    Kalman->P[1][0] -= K[n][1] * P00_temp[n];
    Kalman->P[1][1] -= K[n][1] * P01_temp[n];

    return Kalman->angle;
};