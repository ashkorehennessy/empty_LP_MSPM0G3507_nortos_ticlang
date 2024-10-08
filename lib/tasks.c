//
// Created by ashkore on 24-7-30.
//

#include "tasks.h"
#include "beep.h"
#include "counter.h"
#include "MPU6050.h"
#include "PID.h"
#include "IR.h"

extern uint32_t uptime;
extern float base_setpoint;
int turn1_angle = 35;
int turn2_angle = 53;
int turn3_angle = 194;
int distance1 = 30000;
int distance2 = 26000;
int tracking_mode = 0;
float target_angle = 0;
int target_distance = 0;
uint8_t task_running = 0;
uint8_t task_index = 6;
extern int left_count_sum;
extern int right_count_sum;
extern int need_calibrate;
extern PID_Base track_pid;
extern IR_t Left_IR;
extern IR_t Right_IR;

int task1_prepare(){
    target_distance = 25000;
    target_angle = mpu6050.AngleZ;
    tracking_mode = 0;
    need_calibrate = 0;
}
int task1(){
    if(left_count_sum + right_count_sum > target_distance){
        task_running = 0;
        counter.led_ms = 2000;
        return 1;
    } else {
        return 0;
    }
}


int task2_prepare(){
    target_distance = distance2;
    target_angle = mpu6050.AngleZ;
    tracking_mode = 0;
    need_calibrate = 0;
}
int task2(){
    static int task_state = 0;
    switch (task_state) {
        case 0:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 1000;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ + 160;
            }
            break;
        case 1:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 2;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ + 14;
                target_distance = distance2 - 1000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 1000;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ + 160;
            }
            break;
        case 3:
            if(mpu6050.AngleZ > target_angle) {
                task_running = 0;
                counter.led_ms = 2000;
            }
            break;
    }
}

int task3_prepare(){
    target_distance = distance1;
    target_angle = mpu6050.AngleZ + turn1_angle;
    tracking_mode = 0;
}

int task3(){
    static int task_state = 0;
    switch (task_state) {
        case 0:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 1:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 2;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 3:
            if (mpu6050.AngleZ > target_angle) {
                task_running = 0;
                counter.led_ms = 2000;
            }
            break;
    }
}

int task4_prepare(){
    target_distance = distance1;
    target_angle = mpu6050.AngleZ + turn1_angle;
    tracking_mode = 0;
}

int task4(){
    static int task_state = 0;
    switch (task_state) {
        case 0:
            if (left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 1:
            if (mpu6050.AngleZ < target_angle) {
                task_state = 2;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if (left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 3:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 4;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ + turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 4:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 5;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 5:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 6;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 6:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 7;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 7:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 8;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ + turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 8:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 9;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 9:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 10;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 10:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 11;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 11:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 12;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ + turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 12:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 13;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 13:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 14;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 14:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 15;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 15:
            if(mpu6050.AngleZ > target_angle) {
                task_running = 0;
                counter.led_ms = 2000;
            }
            break;
    }
}

int task5_prepare(){
    target_distance = distance1;
    target_angle = mpu6050.AngleZ + turn1_angle;
    tracking_mode = 0;
}

int task5(){
    static int task_state = 0;
    static int round = 0;
    switch (task_state) {
        case 0:
            if (left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - turn3_angle;
            }
            break;
        case 1:
            if (mpu6050.AngleZ < target_angle) {
                task_state = 2;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ - turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if (left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 1000;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + turn3_angle;
            }
            break;
        case 3:
            if (mpu6050.AngleZ > target_angle) {
                task_state = 0;
                counter.led_ms = 1000;
                target_angle = mpu6050.AngleZ + turn2_angle;
                target_distance = distance1;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                round++;
                if (round == 4) {
                    task_running = 0;
                    counter.led_ms = 2000;
                }
            }
            break;
    }
}

int task6_prepare(){
    tracking_mode = 1;
    need_calibrate = 0;
}

int task6(){
    if(Left_IR.S1 + Left_IR.S2 + Left_IR.S3 + Left_IR.S4 + Right_IR.S1 + Right_IR.S2 + Right_IR.S3 + Right_IR.S4 < 4){
        task_running = 0;
        counter.led_ms = 2000;
        return 1;
    } else {
        return 0;
    }
}