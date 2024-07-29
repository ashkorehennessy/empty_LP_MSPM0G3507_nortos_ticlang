//
// Created by ashkore on 24-7-29.
//

#include "beep.h"
#include "counter.h"

void beep_on(){
    DL_GPIO_clearPins(BEEP_PORT, BEEP_beep_PIN);
}

void beep_off(){
    DL_GPIO_setPins(BEEP_PORT, BEEP_beep_PIN);
}

void beep_ms(uint16_t ms){
    counter.beep_ms = ms;
}
