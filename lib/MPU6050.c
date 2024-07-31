#include "MPU6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105
extern uint32_t uptime;
double timer = 0;
MPU6050_t mpu6050;
// Setup MPU6050
#define MPU6050_ADDRESS 0xD0
#define abs(x) ((x) > 0 ? (x) : -(x))
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
double Gyro_Z_corrector = 0.43;

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	I2C_Start();
	I2C_SendByte(MPU6050_ADDRESS);
	I2C_ReceiveAck();
	I2C_SendByte(RegAddress);
	I2C_ReceiveAck();
	I2C_SendByte(Data);
	I2C_ReceiveAck();
	I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	I2C_Start();
	I2C_SendByte(MPU6050_ADDRESS);
	I2C_ReceiveAck();
	I2C_SendByte(RegAddress);
	I2C_ReceiveAck();

	I2C_Start();
	I2C_SendByte(MPU6050_ADDRESS | 0x01);
	I2C_ReceiveAck();
	Data = I2C_ReceiveByte();
	I2C_SendAck(1);
	I2C_Stop();

	return Data;
}
void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_Read_All(MPU6050_t *DataStruct)
{
    double dt = (uptime - timer) / 1000.0;
    timer = uptime;
	uint8_t DataH, DataL;
	
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
//	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
//    DataStruct->Accel_X_RAW = (int16_t)(DataH << 8 | DataL);
//
//	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
//    DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
//    DataStruct->Accel_Y_RAW = (int16_t)(DataH << 8 | DataL);
//
//    DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
//    DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
//    DataStruct->Accel_Z_RAW = (int16_t)(DataH << 8 | DataL);
//
//    DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
//    DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
//    DataStruct->Gyro_X_RAW = (int16_t)(DataH << 8 | DataL);
//
//    DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
//    DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
//    DataStruct->Gyro_Y_RAW = (int16_t)(DataH << 8 | DataL);

    DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    DataStruct->Gyro_Z_RAW = (int16_t)(DataH << 8 | DataL);

//    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
//    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
//    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
//
//    DataStruct->Gx = DataStruct->Gyro_X_RAW / 65.5;
//    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 65.5;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0 + Gyro_Z_corrector;

    double AngleZ = DataStruct->Gz * dt;
    if(abs((int)AngleZ) > 1000) AngleZ = 0;
    DataStruct->AngleZ += AngleZ;
}
