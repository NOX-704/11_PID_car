#ifndef HUIDU_H
#define HUIDU_H

// 八路循迹模块 I2C 地址
#define HUIDU8_I2C_ADDR  0x12

// 接线
// 八路循迹模块        MSPM0G3507(I2C1)
// VCC                 5V(根据说明书确定具体电压)
// GND                 GND
// SCL                 PB2 (GPIOB.2, I2C1_SCL)
// SDA                 PB3 (GPIOB.3, I2C1_SDA)

#include "ti_msp_dl_config.h"
#include "motor.h"

void huidu_get_value();
void adjust_motor();

#endif
