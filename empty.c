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
#include "Delay.h"
#include "stdio.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "string.h"
#include "UI.h"
#include "switch.h"
#include "ssd1306.h"
#include "IR.h"
#include "counter.h"
#include "beep.h"
#include "PID.h"
#include "tasks.h"
volatile unsigned int delay_times = 0;
uint32_t time_system; 
IR_t Left_IR;
IR_t Right_IR;
int ir_pos;
PID_Base track_pid;
PID_Base angle_pid;
PID_Base roaming_pid;
PID_Base left_pid;
PID_Base right_pid;
Motor motor_L;
Motor motor_R;
float base_setpoint = 140;
float left_setpoint = 0;
float right_setpoint = 0;
int ir_not_found = 0;
int need_calibrate = 0;
int first_detect = 0;
extern uint8_t task_index;
void Delay_Systick_ms(unsigned int ms) 
{
    delay_times = ms;
    while( delay_times != 0 );
}      

void SysTick_Handler(void)
{
    if( delay_times != 0 )
    {
        delay_times--;
    }
}


uint8_t str[50],str2[50];
int speed_L=0,speed_R=0,speed=1000,angel_error=0;
uint32_t uptime = 0;

#define LOWPASS_FILTER_FACTOR 0.33f
static int lowpass_filter(int new_sample) {
    static int last_sample = 0;
    int filtered_sample = (last_sample * LOWPASS_FILTER_FACTOR) + new_sample * (1 - LOWPASS_FILTER_FACTOR);
    last_sample = filtered_sample;
    return filtered_sample;
}

int main(void)
{
    SYSCFG_DL_init();
    track_pid = PID_Base_Init(-6, 0, -8, 800, -800, 1, 0, 0, 0);
    angle_pid = PID_Base_Init(4, 0, 0, 900, -900, 1, 0, 0, 0);
    roaming_pid = PID_Base_Init(0, 0, 0, 900, -900, 0, 0, 0, 0);
    left_pid = PID_Base_Init(10, 0, 1, 900, -900, 0, 0, 0, 0);
    right_pid = PID_Base_Init(10, 0, 1, 900, -900, 0, 0, 0, 0);
	ssd1306_Init();
    UI_init();
    MPU6050_Init();
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
	DL_Timer_startCounter(TIMER_Encoder_Read_INST);
    NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN);
    motor_L = motor_init(PWM_Motor_R_INST, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX);//B4 B1
    motor_R = motor_init(PWM_Motor_L_INST, DL_TIMER_CC_0_INDEX, DL_TIMER_CC_1_INDEX);//B6 B7

    IR_Init(&Left_IR, IRfront_SF1_PORT, IRfront_SF1_PIN, IRfront_SF2_PORT, IRfront_SF2_PIN, IRfront_SF3_PORT, IRfront_SF3_PIN, IRfront_SF4_PORT, IRfront_SF4_PIN);
    IR_Init(&Right_IR, IRback_SB1_PORT, IRback_SB1_PIN, IRback_SB2_PORT, IRback_SB2_PIN, IRback_SB3_PORT, IRback_SB3_PIN, IRback_SB4_PORT, IRback_SB4_PIN);

    while (1) 
	{
        UI_show();
        UI_key_process();
        MPU6050_Read_All(&mpu6050);
        if(task_running == 1) {
            switch (task_index) {
                case 1:
                    task1();
                    break;
                case 2:
                    task2();
                    break;
                case 3:
                    task3();
                    break;
                case 4:
                    task4();
                    break;
                case 5:
                    task5();
                    break;
                case 6:
                    task6();
                    break;
            }
        }
//        delay_ms(20);


    }
}

int IR_not_found(){
    static int time_limit = 0;
    if(Left_IR.S1 + Left_IR.S2 + Left_IR.S3 + Left_IR.S4 + Right_IR.S1 + Right_IR.S2 + Right_IR.S3 + Right_IR.S4 == 8){
        time_limit += 5;
        if(time_limit > 1000){
            return 1;
        }
    } else {
        time_limit = 0;
        if(need_calibrate == 1){
            if(Left_IR.S1 == 0){
                mpu6050.AngleZ += 4.5;
                first_detect = 1;
            }
            if(Left_IR.S2 == 0){
                mpu6050.AngleZ += 1.6;
                first_detect = 2;
            }
            if(Left_IR.S3 == 0){
                mpu6050.AngleZ += 1.4;
                first_detect = 3;
            }
            if(Left_IR.S4 == 0){
                mpu6050.AngleZ += 0.1;
                first_detect = 4;
            }
            if(Right_IR.S1 == 0){
                mpu6050.AngleZ -= 0.1;
                first_detect = 5;
            }
            if(Right_IR.S2 == 0){
                mpu6050.AngleZ -= 1.4;
                first_detect = 6;
            }
            if(Right_IR.S3 == 0){
                mpu6050.AngleZ -= 1.6;
                first_detect = 7;
            }
            if(Right_IR.S4 == 0){
                mpu6050.AngleZ -= 4.5;
                first_detect = 8;
            }
            need_calibrate = 0;
        }
        return 0;
    }
    return 0;
}

void counter_process(){
    if(counter.beep_ms>0){
        counter.beep_ms-=5;
        beep_on();
    }else{
        beep_off();
    }

    if(counter.led_ms > 0) {
        if (counter.led_ms % 501 > 250) {
            DL_GPIO_setPins(LED_PORT, LED_led1_PIN);
            DL_GPIO_clearPins(LED_PORT, LED_led2_PIN);
        } else {
            DL_GPIO_clearPins(LED_PORT, LED_led1_PIN);
            DL_GPIO_setPins(LED_PORT, LED_led2_PIN);
        }
        beep_on();
    } else {
        DL_GPIO_clearPins(LED_PORT, LED_led1_PIN);
        DL_GPIO_clearPins(LED_PORT, LED_led2_PIN);
        beep_off();
    }
    if(counter.led_ms > 0){
        counter.led_ms-=5;
    }
}

void TIMER_Encoder_Read_INST_IRQHandler(void){
    float turn_out;
    uptime += 5;
    speed_L=left_count;
    speed_R=right_count;
    left_count_sum+=left_count;
    right_count_sum+=right_count;
    left_count=0;
    right_count=0;
    IR_Read(&Left_IR);
    IR_Read(&Right_IR);
    ir_not_found = IR_not_found();
    ir_pos = lowpass_filter(IR_get_pos(&Left_IR, &Right_IR));
    if(task_running == 1 && task_index < 7) {
        if (tracking_mode == 1) {
            if(ir_not_found == 0) {
                turn_out = PID_Base_Calc(&track_pid, ir_pos, 0);
            } else {
                turn_out = PID_Base_Calc(&roaming_pid, mpu6050.AngleZ, target_angle);
            }
        } else {
            turn_out = PID_Base_Calc(&angle_pid, mpu6050.AngleZ, target_angle);
        }
        if (turn_out > 0) {
            left_setpoint = base_setpoint;
            right_setpoint = base_setpoint - turn_out;
        } else {
            right_setpoint = base_setpoint;
            left_setpoint = base_setpoint + turn_out;
        }
        motor_set_speed(&motor_L, (int) PID_Base_Calc(&left_pid, speed_L, left_setpoint));
        motor_set_speed(&motor_R, (int) PID_Base_Calc(&right_pid, speed_R, right_setpoint));
    } else {
        motor_set_speed(&motor_L, 0);
        motor_set_speed(&motor_R, 0);
    }
    counter_process();
}
