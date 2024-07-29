//
// Created by ashkore on 2024/2/1.
//

#ifndef SMART_CAR_CAMERA_UI_H
#define SMART_CAR_CAMERA_UI_H


#include "ssd1306_fonts.h"

#define FONT_W 6
#define FONT_H 8
#define SCREEN_W 128
#define SCREEN_H 64

#define KEY_FORWARD SWITCH_1
#define KEY_UP SWITCH_2
#define KEY_DOWN SWITCH_3
#define KEY_BACK SWITCH_4

typedef enum {
    INT32 = 0,
    INT16 = 1,
    INT8 = 2,
    UINT32 = 3,
    UINT16 = 4,
    UINT8 = 5,
    DOUBLE = 6,
    FLOAT = 7,
    EMPTY = 8,
    FUNC = 9,
    CHAR = 10
} Type;

typedef union {
    int32_t *int32_p;
    int16_t *int16_p;
    int8_t *int8_p;
    uint32_t *uint32_p;
    uint16_t *uint16_p;
    uint8_t *uint8_p;
    double *double_p;
    float *float_p;
    int (*func_p)(void);  // ֧����ʾ�ղ���int�����ķ���ֵ
    char *char_p;
} Var_p;

typedef struct {
    char name[7];
    uint8_t type;
    Var_p var_p;
} UI_item;

void UI_item_init(UI_item *item, const char *name, int type, void *var_ptr);

void UI_item_set_value(UI_item *item, double value);

double UI_item_get_value(UI_item *item);

void UI_item_show_name(UI_item *item, uint16_t x, uint16_t y, FontDef font);

void UI_item_show_value(UI_item *item, uint16_t x, uint16_t y, FontDef font);

void UI_init();

void UI_show();

void UI_show_custom_part();

void UI_key_process();

#endif //SMART_CAR_CAMERA_UI_H
