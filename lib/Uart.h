#ifndef __UART_H__
#define __UART_H__

#include "ti_msp_dl_config.h"


void UART_0_INST_IRQHandler(void);
void UART_trans_string(UART_Regs *uart, uint8_t *str,int size);
void UART_receiveData(UART_Regs *uart,uint8_t *Data,int size);
void UART_Receive(void);
void UART_Receive_mspmo(void);


#endif 