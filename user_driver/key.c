#include "key.h"
extern int status;

uint32_t counter_1_A = 0;
uint32_t counter_2_A = 0;

void GROUP1_IRQHandler()
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB))
    {
    case DC_MOTOR_BA_IIDX:
        /* code */
        counter_2_A ++;
        break;
    case DC_MOTOR_AA_IIDX:
        counter_1_A ++;
        break;
    
    default:
        break;
    }
}



