#ifndef __ENCODER_H__
#define	__ENCODER_H__

#include "ti_msp_dl_config.h"

void GROUP1_IRQHandler(void);

void TIMER_Encoder_Read_INST_IRQHandler(void);

extern int left_count;
extern int right_count;
extern int left_count_sum;
extern int right_count_sum;

#endif