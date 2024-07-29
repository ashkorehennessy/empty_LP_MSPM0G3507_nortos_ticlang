#include "I2C.h"
#include "ti_msp_dl_config.h"

void SDA_OUT(void)   
{
    DL_GPIO_initDigitalOutput(MPU6050_SDA_IOMUX);     
	DL_GPIO_setPins(MPU6050_PORT, MPU6050_SDA_PIN);	   
    DL_GPIO_enableOutput(MPU6050_PORT, MPU6050_SDA_PIN); 
}
void SDA_IN(void)
{
    DL_GPIO_initDigitalInputFeatures(MPU6050_SDA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}

void SCL_OUT(void)   
{
    DL_GPIO_initDigitalOutput(MPU6050_SCL_IOMUX);     
	DL_GPIO_setPins(MPU6050_PORT, MPU6050_SCL_PIN);	   
    DL_GPIO_enableOutput(MPU6050_PORT, MPU6050_SCL_PIN); 
}
void I2C_W_SCL (uint8_t BitValue)
{
	if(BitValue == 0)
	{
		DL_GPIO_clearPins(MPU6050_PORT,MPU6050_SCL_PIN);
	}
	else
	{
		DL_GPIO_setPins(MPU6050_PORT,MPU6050_SCL_PIN);
	}
	delay_us(8);
}

void I2C_W_SDA(uint8_t BitValue)
{
	SDA_OUT();
	if(BitValue == 0)
	{
		DL_GPIO_clearPins(MPU6050_PORT,MPU6050_SDA_PIN);
	}
	else
	{
		DL_GPIO_setPins(MPU6050_PORT,MPU6050_SDA_PIN);
	}
	delay_us(8);
}

uint8_t I2C_R_SDA(void)
{
	uint8_t BitValue;
	SDA_IN();
	BitValue = DL_GPIO_readPins(MPU6050_PORT,MPU6050_SDA_PIN) == 0 ? 0 : 1;
	delay_us(8);
	return BitValue;
}
void I2C_Init(void)
{
    SYSCFG_DL_GPIO_init();
	/*����Ĭ�ϵ�ƽ*/
	DL_GPIO_setPins(MPU6050_PORT, MPU6050_SDA_PIN | MPU6050_SCL_PIN);//����PA8��PA9���ų�ʼ����Ĭ��Ϊ�ߵ�ƽ���ͷ�����״̬��
}
void I2C_Start(void)
{
	SDA_OUT();
	I2C_W_SDA(1);
	I2C_W_SCL(1);
	I2C_W_SDA(0);
	I2C_W_SCL(0);
}

void I2C_Stop(void)
{
	SDA_OUT();
	I2C_W_SDA(0);
	I2C_W_SCL(1);
	I2C_W_SDA(1);
}
void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	SDA_OUT();
	for(i = 0;i<8;i++)
	{
		I2C_W_SDA(Byte & (0x80 >> i));
		I2C_W_SCL(1);
		I2C_W_SCL(0);
	}
}

uint8_t I2C_ReceiveByte(void)
{
	uint8_t i,Byte = 0x00;
	SDA_OUT();
	I2C_W_SDA(1);
	for(i = 0;i<8;i++)
	{
		SDA_IN();
		I2C_W_SCL(1);
		if(I2C_R_SDA() == 1)  Byte |= (0x80>>i);
		I2C_W_SCL(0);
	}
	return Byte;
}

void I2C_SendAck(uint8_t AckBit)
{
	SDA_OUT();
	I2C_W_SDA(AckBit);
	I2C_W_SCL(1);
	I2C_W_SCL(0);
}
uint8_t I2C_ReceiveAck(void)
{
	uint8_t AckBit;
	SDA_OUT();
	I2C_W_SDA(1);
	I2C_W_SCL(1);
	SDA_IN();
	AckBit = I2C_R_SDA();
	I2C_W_SCL(0);
	return AckBit;
}
	