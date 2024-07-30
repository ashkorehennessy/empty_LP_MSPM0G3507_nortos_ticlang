//
// Created by ashkore on 24-7-30.
//

#ifndef EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_TASKS_H
#define EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_TASKS_H
#include "ti_msp_dl_config.h"
extern int tracking_mode;
extern float target_angle;
extern int target_distance;
extern uint8_t task_running;
extern uint8_t task_index;
extern float input_pos;
extern int need_calibrate;
int task1_prepare();
int task1();
int task2_prepare();
int task2();
int task3_prepare();
int task3();
int task4_prepare();
int task4();



#endif //EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_TASKS_H
