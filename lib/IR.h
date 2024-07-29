//
// Created by ashkore on 24-7-29.
//

#ifndef EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_IR_H
#define EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_IR_H

#include "ti_msp_dl_config.h"

typedef struct {
    uint8_t S1;
    GPIO_Regs* S1_port;
    uint32_t S1_pin;
    uint8_t S2;
    GPIO_Regs* S2_port;
    uint32_t S2_pin;
    uint8_t S3;
    GPIO_Regs* S3_port;
    uint32_t S3_pin;
    uint8_t S4;
    GPIO_Regs* S4_port;
    uint32_t S4_pin;
}IR_t;

void IR_Init(IR_t* ir, GPIO_Regs* S1_port, uint32_t S1_pin, GPIO_Regs* S2_port, uint32_t S2_pin, GPIO_Regs* S3_port, uint32_t S3_pin, GPIO_Regs* S4_port, uint32_t S4_pin);

void IR_Read(IR_t* ir);

float IR_get_pos(IR_t* ir);

#endif //EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_IR_H
