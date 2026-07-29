#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 原工程编码器 A 相上升沿计数底层。当前仅保留硬件计数能力，
 * 不包含速度换算或闭环控制。
 */
extern volatile uint32_t counter_1_A;
extern volatile uint32_t counter_2_A;

#endif /* KEY_H */
