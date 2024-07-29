#include "ti_msp_dl_config.h"
#include "Motor.h"
Motor motor_init(GPTIMER_Regs * pwm_inst, DL_TIMER_CC_INDEX cc0_index, DL_TIMER_CC_INDEX cc1_index){
    Motor motor;
    motor.pwm_inst = pwm_inst;
    motor.cc0_index = cc0_index;
    motor.cc1_index = cc1_index;
    return motor;
}

void motor_set_speed(Motor *motor, int speed){
    if (speed >= 0) {
        DL_TimerG_setCaptureCompareValue(motor->pwm_inst, speed, motor->cc0_index);
        DL_TimerG_setCaptureCompareValue(motor->pwm_inst, 0, motor->cc1_index);
    } else {
        DL_TimerG_setCaptureCompareValue(motor->pwm_inst, 0, motor->cc0_index);
        DL_TimerG_setCaptureCompareValue(motor->pwm_inst, -speed, motor->cc1_index);
    }
}
//void Motor(int speed, int out)
//{
//
//if(speed>=0)
//{
//
//    if(speed+out>=0)
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,speed+out, DL_TIMER_CC_0_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,0, DL_TIMER_CC_1_INDEX);
//    }
//    else
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,0, DL_TIMER_CC_0_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,-speed-out, DL_TIMER_CC_1_INDEX);
//    }
//    if(speed-out>=0)
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,speed-out, DL_TIMER_CC_0_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,0, DL_TIMER_CC_1_INDEX);
//    }
//    else
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,0, DL_TIMER_CC_0_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,out-speed, DL_TIMER_CC_1_INDEX);
//
//    }
//}
//else
//{
//    speed=-speed;
//    if(speed+out>=0)
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,speed+out, DL_TIMER_CC_1_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,0, DL_TIMER_CC_0_INDEX);
//    }
//    else
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,0, DL_TIMER_CC_1_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_L_INST,-speed-out, DL_TIMER_CC_0_INDEX);
//    }
//    if(speed-out>=0)
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,speed-out, DL_TIMER_CC_1_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,0, DL_TIMER_CC_0_INDEX);
//    }
//    else
//    {
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,0, DL_TIMER_CC_1_INDEX);
//    DL_TimerG_setCaptureCompareValue(PWM_Motor_R_INST,out-speed, DL_TIMER_CC_0_INDEX);
//    }
//}
//
//}
//
//void speed_roll(int speed,int speed_L ,int speed_R ,int angel_error)
//{
//    int error_speed=speed-(speed_L+speed_R)/2;
//    int speed_out=Pid_Count(0,0,0 , error_speed,0);
//    int angle_out=Pid_Count(7,1.5,1 , angel_error,1);
//    Motor(speed+speed_out,angle_out);
//}


// void Motorleft(int Compare)//Compare��ֵ3200
// {
// 	if(Compare>0)
// 	{
// 		//������ ��ת
// 		DL_GPIO_setPins(Motor_Left_PORT, Motor_Left_ML1_PIN);
// 		DL_GPIO_clearPins(Motor_Left_PORT, Motor_Left_ML2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_0_INDEX);
// 	}
// 	if(Compare<0)
// 	{
// 		//������ ��ת
// 		DL_GPIO_clearPins(Motor_Left_PORT, Motor_Left_ML1_PIN);
// 		DL_GPIO_setPins(Motor_Left_PORT, Motor_Left_ML2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_0_INDEX);
// 	}
// }

// void Motorright(int Compare)
// {
// 	if(Compare>0)
// 	{
// 		//�ҵ��� ��ת
// 		DL_GPIO_setPins(Motor_Right_PORT, Motor_Right_MR1_PIN);
// 		DL_GPIO_clearPins(Motor_Right_PORT, Motor_Right_MR2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_1_INDEX);
// 	}
// 	if(Compare<0)
// 	{
// 		//�ҵ��� ��ת
// 		DL_GPIO_clearPins(Motor_Right_PORT, Motor_Right_MR1_PIN);
// 		DL_GPIO_setPins(Motor_Right_PORT, Motor_Right_MR2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_1_INDEX);
// 	}
// }

// void GO_Ahead(int Compare)
// {
// 	if(Compare>0)
// 	{
// 		DL_GPIO_setPins(Motor_Left_PORT, Motor_Left_ML1_PIN);
// 		DL_GPIO_clearPins(Motor_Left_PORT, Motor_Left_ML2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_0_INDEX);
// 		DL_GPIO_setPins(Motor_Right_PORT, Motor_Right_MR1_PIN);
// 		DL_GPIO_clearPins(Motor_Right_PORT, Motor_Right_MR2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_1_INDEX);
// 	}
// 	if(Compare<0)
// 	{

// 		DL_GPIO_clearPins(Motor_Left_PORT, Motor_Left_ML1_PIN);
// 		DL_GPIO_setPins(Motor_Left_PORT, Motor_Left_ML2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_0_INDEX);

// 		DL_GPIO_clearPins(Motor_Right_PORT, Motor_Right_MR1_PIN);
// 		DL_GPIO_setPins(Motor_Right_PORT, Motor_Right_MR2_PIN);
// 		DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,Compare, DL_TIMER_CC_1_INDEX);
// 	}
// }
