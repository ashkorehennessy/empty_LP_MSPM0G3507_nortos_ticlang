//
// Created by ashkore on 2024/1/24.
//

#ifndef SMART_CAR_CAMERA_SWITCH_H
#define SMART_CAR_CAMERA_SWITCH_H

#include "ti_msp_dl_config.h"

#define DIP_SWITCH_1 !DL_GPIO_readPins(KEY_dip1_PORT, KEY_dip1_PIN)
#define DIP_SWITCH_2 !DL_GPIO_readPins(KEY_dip2_PORT, KEY_dip2_PIN)
#define DIP_SWITCH  ((DIP_SWITCH_1) | (DIP_SWITCH_2) << 1)

#define SWITCH_1 !DL_GPIO_readPins(KEY_switch1_PORT, KEY_switch1_PIN)
#define SWITCH_2 !DL_GPIO_readPins(KEY_switch2_PORT, KEY_switch2_PIN)
#define SWITCH_3 !DL_GPIO_readPins(KEY_switch3_PORT, KEY_switch3_PIN)
#define SWITCH_4 !DL_GPIO_readPins(KEY_switch4_PORT, KEY_switch4_PIN)

int dip_switch();

int switch1();

int switch2();

int switch3();

int switch4();

#endif //SMART_CAR_CAMERA_SWITCH_H
