//
// Created by ashkore on 24-7-29.
//

#ifndef EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_COUNTER_H
#define EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_COUNTER_H
#include "ti_msp_dl_config.h"
typedef struct {
    uint16_t beep_ms;
    uint16_t led_ms;
}Counter;
extern Counter counter;
#endif //EMPTY_LP_MSPM0G3507_NORTOS_TICLANG_COUNTER_H
