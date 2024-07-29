/*
 * PID_Base.c
 *
 *  Created on: 2023年9月13日
 *      Author: ashkore
 */

#include "PID.h"

PID_Base PID_Base_Init(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_integral,
                       uint8_t use_lowpass_filter, float lowpass_filter_factor, float deadzone) {
    PID_Base pid;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.last_error = 0;
    pid.last_out = 0;
    pid.integral = 0;
    pid.outmax = outmax;
    pid.outmin = outmin;
    pid.use_integral = use_integral;
    pid.use_lowpass_filter = use_lowpass_filter;
    pid.lowpass_filter_factor = lowpass_filter_factor;
    pid.deadzone = deadzone;
    return pid;
}

float PID_Base_Calc(PID_Base *pid, float input_value, float setpoint){
    float error = setpoint - input_value;
    float derivative = error - pid->last_error;
    if(pid->use_integral) {
        pid->integral += error;
    }
    pid->last_error = error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->last_out = output;
    output += pid->deadzone;

//    // Integral limit
//    if(pid->integral > pid->outmax/9){
//        pid->integral = pid->outmax/9;
//    } else if(pid->integral < pid->outmin/9){
//        pid->integral = pid->outmin/9;
//    }

    // Output limit
    if(output > pid->outmax){
        output = pid->outmax;
    } else if(output < pid->outmin){
        output = pid->outmin;
    }

    // Low pass filter
    if(pid->use_lowpass_filter){
        output = pid->last_out * pid->lowpass_filter_factor + output * (1 - pid->lowpass_filter_factor);
    }


    return output;
}

PID_Incremental PID_Incremental_Init(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_lowpass_filter, float lowpass_filter_factor) {
    PID_Incremental pid;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.error = 0;
    pid.last_error = 0;
    pid.last_last_error = 0;
    pid.last_out = 0;
    pid.out = 0;
    pid.outmax = outmax;
    pid.outmin = outmin;
    pid.use_lowpass_filter = use_lowpass_filter;
    pid.lowpass_filter_factor = lowpass_filter_factor;
    return pid;
}

float PID_Incremental_Calc(PID_Incremental *pid, float input_value, float setpoint){
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = setpoint - input_value;
    float derivative = (pid->error - 2 * pid->last_last_error + pid->last_last_error);
    float output_increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error + pid->Kd * derivative;

    pid->out += output_increment;

    // Output limit
    if(pid->out > pid->outmax){
        pid->out = pid->outmax;
    } else if(pid->out < pid->outmin){
        pid->out = pid->outmin;
    }

    // Low pass filter
    if(pid->use_lowpass_filter){
        pid->out = pid->last_out * pid->lowpass_filter_factor + pid->out * (1 - pid->lowpass_filter_factor);
    }

    pid->last_out = pid->out;

    return pid->out;
}