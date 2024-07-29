#include "PID.h"
int Erro=0;            //当前偏差
int LErro=0;           //上次偏差
int Differential=0;    //微分误差变化率
int Intedral=0;        //积分误差累计
float T;               //PID输出周期，应与采样周期一致
uint16_t Tdata;          //判断输出周期标志位
float Ti;              //积分时间常数
float Td;              //微分系数

float Ek[10];              //当前偏差
float Ek_1[10];            //上次偏差
float Ek_2[10];
float SEk;             //历史偏差之和
float DelEk;           //本次和上次的偏差

float Pout;
float Iout;
float Dout;
float out;             //电机最终输出值

uint16_t pwmcycle;       //pwm占空比
float I_Error[10]={0},Accumulate[10]={0},D_Error[10]={0},Last_Error[10]={0};

//pid结构体初始化
PID_Incremental PID0={
    .Kp=0,
    .Ki=0,
    .Kd=0
};
PID_Incremental PID1={
    .Kp=0,
    .Ki=0,
    .Kd=0
};

/*
 * *****最简PID函数，PD控制*****
 *      aim目标参数
 *      current当前
 *      p比例系数值
 *      d微分系数值
 */
int pid1(int aim,int current,float p,float d)
{
    Erro=current-aim;       //当前误差

    Differential=Erro-LErro;//当次误差与上次误差之差，算出误差变化

    LErro=Erro;             //存储此次误差准备和下次比较

    return (int)(p*Erro+d*Differential+0*Intedral);//PD控制输出值

}


/*
 * *****动态PID函数，动态Pd*****
 *
 * fp为前瞻偏差乘适当系数，应小于p的数量级
 * fd同理
 *
 */
int pid_flow(int aim,int current,float p,float d,float fp,float fd)
{
    Erro=current-aim;       //当前误差

    Differential=Erro-LErro;//当次误差与上次误差之差，算出误差变化

    LErro=Erro;             //存储此次误差准备和下次比较

    return (int)(fp*p*Erro+fd*d*Differential+0*Intedral);

}


///*
// * *****位置式PID*****
// *
// *
// */
// int PID_location(int aim,int current,float Kp,float Ki,float Kd)
//{
//    T=12;                       //PID输出周期
//
//    Ti=180;
//    Td=1;
//    pwmcycle=330;              //pwm实际输出周期
//
//    float ti;                 //积分时间系数
//    float td;                 //微分时间系数
//
//    //if(Tdata<T)               //最小计算周期未到
//    //{
//    //   return 0;
//    //}
//
//    //Tdata=0;                 //判断置0
//
//
//    Ek=(float)(aim-current);           //本次偏差
//
//    SEk+=Ek;                  //历史偏差总和
//
//    DelEk=Ek-Ek_1;            //本次和上次的偏差
//
//
//    /**比例输出**/
//    Pout=Kp*Ek;
//
//    /**积分输出**/
//    ti=T/Ti;
//    Ki=ti*Kp;
//    Iout=Ki*SEk;
//
//    /**微分输出**/
//    td = Td/T;
//    Kd=td*Kp;
//    Dout=Kd*DelEk;
//
//    Ek_1=Ek;                  //更新偏差
//
//    return out=Pout+Iout+Dout;
//
//}


 /*
  * *****增量式PID****
  *
  */
int PID_increment(int practical,float Kp,float Ki,float Kd,int x)
{
        T=12;                       //PID输出周期

        Ti=120;
        Td=1;
        pwmcycle=10;              //pwm实际输出周期

        float dk1;
        //float dk2;

        Ek[x]=(float)(practical);

        dk1=Ek[x]-Ek_1[x];
       // dk2=Ek-2*Ek_1+Ek_2;

        Pout=Kp*dk1;

        Iout=Kp*T/Ti;
        Iout=Iout*Ki;

        Dout=Kp*Td/T;
        Dout=Dout*Kd;

        Ek_2[x]=Ek_1[x];
        Ek_1[x]=Ek[x];

        return out+=Pout+Iout+Dout;

}

int Pid_Count(float proportion,float integral,float differential,int practical,int x)//目标，比例系数，积分系数，微分系数，当前误差
  {

      I_Error[x] = practical;           //偏差
      Accumulate[x] += I_Error[x];               //偏差累计
      if(Accumulate[x]>1000)Accumulate[x]=1000;  //积分限幅
      else if(Accumulate[x]<-1000)Accumulate[x]=-1000;
      D_Error[x] = I_Error[x] - Last_Error[x];
//      D_Error[x] = -func_abs(XK.gyroz);
      Last_Error[x] = I_Error[x];
      return (int)(proportion*I_Error[x] + integral*Accumulate[x] + differential*D_Error[x]);
  }

int PID_Incremental_Calc(PID_Incremental *pid, int proportion){
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = proportion;
    int derivative = (pid->error - 2 * pid->last_last_error + pid->last_last_error);
    int output_increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error + pid->Kd * derivative;

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
