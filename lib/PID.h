#ifndef __PID_H
#define __PID_H
#include "ti_msp_dl_config.h"
typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float last_error;
    float last_out;
    float integral;
    float outmax;
    float outmin;
    uint8_t use_lowpass_filter;
    float lowpass_filter_factor;
} PID_Base;

typedef struct{
    int Kp;
    int Ki;
    int Kd;
    int error;
    int last_error;
    int last_last_error;
    int last_out;
    int out;
    int outmax;
    int outmin;
    uint8_t use_lowpass_filter;
    int lowpass_filter_factor;
} PID_Incremental;
extern PID_Incremental PID0;
extern PID_Incremental PID1;

int pid1(int aim,int current,float p,float d);                         //最简PID
int pid_flow(int aim,int current,float p,float d,float fp,float fd);   //动态PD
int PID_location(int aim,int current,float Kp,float Ki,float Kd);      //位置式PID
int PID_increment(int practical,float Kp,float Ki,float Kd,int x);  //增量式PID
int Pid_Count(float proportion,float integral,float differential,int practical,int x);//目标，比例系数，积分系数，微分系数，当前实际值,第几个pid
#endif
