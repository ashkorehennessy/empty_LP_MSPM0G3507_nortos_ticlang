/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS CT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES BE LIABLE FOR ANY DIRE(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
PB4 PB1 R
PB6 PB7 L
mpu6050 scl PA12 sda PA13
oled scl PB2 sda PB31
uart TX PA28 RX PA31
encoder R PA24 PA17
encoder L PA8 PA26

*/



#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "Servo.h"
#include "Sonic.h"
#include "Delay.h"
#include "stdio.h"
#include "Uart.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "string.h"
#include "IMU.h"
#include "UI.h"
#include "switch.h"
#include "ssd1306.h"
//�����δ���ʱ��ʵ�ֵľ�ȷms��ʱ
volatile unsigned int delay_times = 0;
uint32_t time_system; 
MPU6050_t mpu;
void Delay_Systick_ms(unsigned int ms) 
{
    delay_times = ms;
    while( delay_times != 0 );
}      
 
//�δ���ʱ���ж�
void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}


uint8_t str[50],str2[50];
int speed_L=0,speed_R=0,speed=1000,angel_error=0;
extern param_imu imu_data;
extern param_Angle imu_Angle;

int main(void)
{
    SYSCFG_DL_init();
	ssd1306_Init();
    UI_init();
//    MPU6050_Init();
	DL_TimerG_startCounter(PWM_Motor_L_INST);//B6 B7
    DL_TimerG_startCounter(PWM_Motor_R_INST);//B4 B1
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
//    NVIC_EnableIRQ(TIMER_system_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	DL_Timer_startCounter(TIMER_Encoder_Read_INST);      
    NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN );
    Motor motor_L = motor_init(PWM_Motor_L_INST, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX);
    Motor motor_R = motor_init(PWM_Motor_R_INST, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX);


    while (1) 
	{

         UI_show();
         UI_key_process();
        delay_ms(20);




    }
}

void INT_timeG8()
{           
    time_system++;
    static uint16_t cnt = 0;
    cnt++;
    if(cnt%10==0)
    {
        MPU6050_Read_All(&mpu,0);

    }
    if(cnt>=1000)
    {
        cnt=0;
        // DL_GPIO_togglePins(LED_PORT,LED_led1_PIN);
    }

}

//void TIMER_system_INST_IRQHandler(void)
//{
//    switch (DL_TimerG_getPendingInterrupt(TIMER_system_INST)) {
//        case DL_TIMER_IIDX_ZERO:
//         INT_timeG8();
//            break;
//        default:
//            break;
//    }
//}
