#include "Uart.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_uart.h"
#include "ti/driverlib/dl_uart.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "Delay.h"
#include "extern.h"
#include "string.h"
#include "OLED.h"
#include "string.h"
#include "stdio.h"

uint8_t uart0_rx_buf[5];
//PA11 RX PA10 TX ok
uint8_t gEchoData[10];
uint8_t i_data=0;
uint8_t data;
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) 
	{
        case DL_UART_MAIN_IIDX_RX:
            DL_GPIO_togglePins(LED_PORT,LED_led1_PIN);
            UART_Receive_mspmo();
            // UART_trans_string(UART_0_INST, uart0_rx_buf,strlen((const char*)uart0_rx_buf));
            break;
        default:
            break;
    }
}
void UART_Receive_mspmo(void)
{
    data=DL_UART_Main_receiveData(UART_0_INST);
    if(i_data==0)
    {
    uart0_rx_buf[0] = data;
    }
		if((int)uart0_rx_buf[0]==0xa3)
		{
            if(i_data==1)
            {
            uart0_rx_buf[1] =data;
            }
            if(i_data>1)
            {
			if((int)uart0_rx_buf[1]==0xb3)
			{
                uart0_rx_buf[i_data] =data;
			}
            else 
            {
            	i_data=-1;
                uart0_rx_buf[0]=0;
            }
            }
            i_data++;
		}
		else
        {
            i_data=0;
            uart0_rx_buf[0]=0;
        }
        if(i_data>=5)
        {
            if(uart0_rx_buf[4]==0xc3)
            {
            int symbol=(int)uart0_rx_buf[2];
            if(symbol==0x01)
            angel_error=(int)uart0_rx_buf[3];
            else
            angel_error=(int)(-uart0_rx_buf[3]);
            }
            i_data=0;
            uart0_rx_buf[0]=0;

        }
        // uint8_t s[10]="1212";
        // sprintf((char*)s,"%c",data);
        // OLED_Fill(0X00);
		// OLED_ShowStr(0,3,s,1);

}

void UART_Receive(void)
{
    uart0_rx_buf[0] = DL_UART_Main_receiveData(UART_0_INST);
		if((int)uart0_rx_buf[0]==0xa3)
		{


            uart0_rx_buf[1] = DL_UART_Main_receiveData(UART_0_INST);
			if((int)uart0_rx_buf[1]==0xb3)
			{
                    UART_receiveData(UART_0_INST,uart0_rx_buf+2,2);
					if((int)uart0_rx_buf[3]==(0xc3))
					{
                        angel_error=(int)uart0_rx_buf[2];
					}
			}
		}
			else
				uart0_rx_buf[0]=0;

}

void UART_receiveData(UART_Regs *uart,uint8_t *Data,int size)
{
    int i=0;
    for(i=0;i<size;i++)
    {
    Data[i]=DL_UART_Main_receiveData(uart);
    delay_us(100);
    }
}

void UART_trans_string(UART_Regs *uart, uint8_t *str,int size)
{
    int i=0;
    for(i=0;i<size;i++)
    {
    DL_UART_transmitData(uart,str[i]);
    delay_us(100);
    }

}