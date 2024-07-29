#ifndef __MOTOR_H__
#define	__MOTOR_H__

#include "ti_msp_dl_config.h"

#define MAX_SPEED 900

// Motor struct
typedef struct {
    GPTIMER_Regs * pwm_inst;   // PWM实例
    DL_TIMER_CC_INDEX cc0_index;  // 捕获比较值索引0
    DL_TIMER_CC_INDEX cc1_index;  // 捕获比较值索引1
} Motor;

Motor motor_init(GPTIMER_Regs * pwm_inst, DL_TIMER_CC_INDEX cc0_index, DL_TIMER_CC_INDEX cc1_index);

void motor_set_speed(Motor *motor, int speed);

#endif