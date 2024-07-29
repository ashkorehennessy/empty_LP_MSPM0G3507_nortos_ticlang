#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include "MPU6050.h"


int left_count=0;
int right_count=0;
int left_count_sum=0;
int right_count_sum=0;
extern int speed_L;
extern int speed_R;
void GROUP1_IRQHandler(void){
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA, Encoder_Left_EL2_PIN | Encoder_Right_ER1_PIN );
    if (gpioA & Encoder_Left_EL2_PIN) {
        if (DL_GPIO_readPins(GPIOB, Encoder_Left_EL1_PIN)){
            left_count--;
        }else {
            left_count++;
        }
        DL_GPIO_clearInterruptStatus(GPIOA,Encoder_Left_EL2_PIN);
    }

    if (gpioA & Encoder_Right_ER1_PIN){
        if(DL_GPIO_readPins(GPIOA,Encoder_Right_ER2_PIN)){
            right_count++;
        } else {
            right_count--;
        }
        DL_GPIO_clearInterruptStatus(GPIOA , Encoder_Right_ER1_PIN);
    }
}

