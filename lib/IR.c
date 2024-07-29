//
// Created by ashkore on 24-7-29.
//

#include "IR.h"

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

float IR_get_pos(IR_t* ir) {
    static float last_pos = 0;
    if(ir->S1 == 1 && ir->S2 == 1 && ir->S3 == 1 && ir->S4 == 1){
        return last_pos;
    }
    if(ir->S2 == 0 && ir->S3 == 0){
        return 0;
    }
    if(ir->S1 == 0){
        return -4.5f;
    }
    if(ir->S4 == 0){
        return 4.5f;
    }
    if(ir->S2 == 0){
        return -1.5f;
    }
    if(ir->S3 == 0){
        return 1.5f;
    }
    return last_pos;
}