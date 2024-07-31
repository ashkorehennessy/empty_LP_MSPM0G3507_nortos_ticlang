//
// Created by ashkore on 24-7-30.
//

#include "tasks.h"
#include "beep.h"
#include "counter.h"
#include "MPU6050.h"

#define BASE_MAX 105
#define BASE_MIN 90

extern uint32_t uptime;
extern float base_setpoint;
int tracking_mode = 0;
float target_angle = 0;
int target_distance = 0;
uint8_t task_running = 0;
uint8_t task_index = 4;
extern int left_count_sum;
extern int right_count_sum;
extern int need_calibrate;

int task1_prepare(){
    target_distance = 22000;
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
    target_distance = 22000;
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
                counter.led_ms = 400;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ + 160;
            }
            break;
        case 1:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 2;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ + 17;
                target_distance = 22000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 400;
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
    target_distance = 28000;
    target_angle = mpu6050.AngleZ;
    tracking_mode = 0;
    need_calibrate = 1;
}

int task3(){
    static int task_state = 0;
    switch (task_state) {
        case 0:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 400;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ - 190;
            }
            break;
        case 1:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 2;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ - 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
            }
            break;
        case 2:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 400;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ + 190;
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

int task4_prepare(){
    target_distance = 28000;
    target_angle = mpu6050.AngleZ;
    tracking_mode = 0;
    need_calibrate = 1;
    base_setpoint = BASE_MAX;
}

int task4(){
    static int task_state = 0;
    switch (task_state) {
        case 0:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 1;
                counter.led_ms = 400;
                tracking_mode = 1;
                target_angle = mpu6050.AngleZ - 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 1:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 2;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ - 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 2:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 3;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 3:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 4;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ + 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 4:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 5;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 5:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 6;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ - 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 6:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 7;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 7:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 8;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ + 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 8:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 9;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 9:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 10;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ - 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 10:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 11;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 11:
            if(mpu6050.AngleZ > target_angle) {
                task_state = 12;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ + 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 12:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 13;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ - 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 13:
            if(mpu6050.AngleZ < target_angle) {
                task_state = 14;
                counter.led_ms = 400;
                target_angle = mpu6050.AngleZ - 53;
                target_distance = 28000;
                tracking_mode = 0;
                left_count_sum = 0;
                right_count_sum = 0;
                base_setpoint = BASE_MAX;
            }
            break;
        case 14:
            if(left_count_sum + right_count_sum > target_distance) {
                task_state = 15;
                counter.led_ms = 400;
                tracking_mode = 1;
                need_calibrate = 1;
                target_angle = mpu6050.AngleZ + 190;
                base_setpoint = BASE_MIN;
            }
            break;
        case 15:
            if(mpu6050.AngleZ > target_angle) {
                task_running = 0;
                counter.led_ms = 1000;
            }
            break;
    }
}
