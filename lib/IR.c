//
// Created by ashkore on 24-7-29.
//

#include "IR.h"
#define WINDOW_SIZE 5
static int moving_average_filter(int new_sample) {
    static int samples[WINDOW_SIZE] = {0};
    static int index = 0;
    static int sum = 0;

    // 更新滑动窗口
    sum -= samples[index];
    samples[index] = new_sample;
    sum += new_sample;
    index = (index + 1) % WINDOW_SIZE;

    // 计算平均值
    return (sum / WINDOW_SIZE);
}

void IR_Init(IR_t* ir, GPIO_Regs* S1_port, uint32_t S1_pin, GPIO_Regs* S2_port, uint32_t S2_pin, GPIO_Regs* S3_port, uint32_t S3_pin, GPIO_Regs* S4_port, uint32_t S4_pin){
    ir->S1_port = S1_port;
    ir->S1_pin = S1_pin;
    ir->S2_port = S2_port;
    ir->S2_pin = S2_pin;
    ir->S3_port = S3_port;
    ir->S3_pin = S3_pin;
    ir->S4_port = S4_port;
    ir->S4_pin = S4_pin;
}

void IR_Read(IR_t* ir){
    ir->S1 = !DL_GPIO_readPins(ir->S1_port, ir->S1_pin);
    ir->S2 = !DL_GPIO_readPins(ir->S2_port, ir->S2_pin);
    ir->S3 = !DL_GPIO_readPins(ir->S3_port, ir->S3_pin);
    ir->S4 = !DL_GPIO_readPins(ir->S4_port, ir->S4_pin);
}

//     ir1         ir2
// s1 s2 s3 s4 s1 s2 s3 s4
int IR_get_pos(IR_t *ir1, IR_t *ir2) {
    static int last_pos = 0;
    int pos = 0;
    if(ir1->S1 + ir1->S2 + ir1->S3 + ir1->S4 + ir2->S1 + ir2->S2 + ir2->S3 + ir2->S4 == 8){
        pos = last_pos;
    }
    if(ir1->S4 == 0) {
        pos = -8;
    }
    if(ir2->S1 == 0){
        pos = 8;
    }
    if(ir1->S1 == 0){
        pos = -25;
        last_pos = -25;
    }
    if(ir2->S4 == 0){
        pos = 25;
        last_pos = 25;
    }
    if(ir1->S2 == 0){
        pos = -20;
        last_pos = -20;
    }
    if(ir2->S3 == 0){
        pos = 20;
        last_pos = 20;
    }
    if(ir1->S3 == 0){
        pos = -18;
        last_pos = -18;
    }
    if(ir2->S2 == 0){
        pos = 18;
        last_pos = 18;
    }

    pos = moving_average_filter(pos);

    return pos;
}