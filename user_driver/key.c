#include "key.h"

/* 通道 1/A=右轮，通道 2/B=左轮；沿用原工程的正向累加语义。 */
volatile uint32_t counter_1_A = 0U;
volatile uint32_t counter_2_A = 0U;

/**
 * GPIOB GROUP1 中断只统计两路编码器 A 相上升沿。
 *
 * 中断中不做速度换算、浮点运算、串口打印或控制算法。
 */
void GROUP1_IRQHandler(void)
{
    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
    case DC_MOTOR_BA_IIDX:
        counter_2_A++;
        break;

    case DC_MOTOR_AA_IIDX:
        counter_1_A++;
        break;

    default:
        break;
    }
}
