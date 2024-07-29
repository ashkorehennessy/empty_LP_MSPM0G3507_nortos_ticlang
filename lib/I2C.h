#ifndef __I2C_H__
#define __I2C_H__

#include "ti_msp_dl_config.h"
#include "Delay.h"

void I2C_W_SCL (uint8_t BitValue);

void I2C_W_SDA(uint8_t BitValue);

uint8_t I2C_R_SDA(void);

void I2C_Init(void);

void I2C_Start(void);

void I2C_Stop(void);

void I2C_SendByte(uint8_t Byte);

uint8_t I2C_ReceiveByte(void);

void I2C_SendAck(uint8_t AckBit);

uint8_t I2C_ReceiveAck(void);
void SDA_OUT(void);   

void SDA_IN(void);

void SCL_OUT(void);   


#endif

