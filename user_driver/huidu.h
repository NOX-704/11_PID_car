#ifndef HUIDU_H
#define HUIDU_H

// 八路循迹模块 GPIO 引脚定义
// L1=PB6 L2=PB7 L3=PA17 L4=PA18 R1=PA21 R2=PA22 R3=PA24 R4=PA25

#include "ti_msp_dl_config.h"
#include "motor.h"

#define MIN_SPEED 150

void huidu_get_value();
void adjust_motor();

#endif
