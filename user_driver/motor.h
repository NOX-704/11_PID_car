#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 260
// 轮胎直径 mm
#define MOTOR_WHEEL_D 48

// G3507      TB6612
// PB24 <--> STBY
// PA9 <--> AIN1
// PA8 <--> AIN2
// PA12 <--> PWMA
// GND <--> GND
// 3V3 <--> VCC

// TB6612    电源模块
// VM          7.4V
// GND         GND

// TB6612    直流电机1
// AO1<--> M+
// AO2<--> M-

// G3507    直流电机1
// PB8 <--> A
// PB9 <--> B
// 3V3 <--> VCC
// GND <--> GND

// 直流电机接线：
// BO1<--> M+
// BO2<--> M-
// PB19 <--> A
// PB20 <--> B
// 3V3 <--> VCC
// GND <--> GND

// G3507    TB6612
// PA13 <--> PWMB
// PA7 <--> BIN1
// PB18 <--> BIN2

// 所有的GND都需要连接在一起

#include "ti_msp_dl_config.h"

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

/*
 * 由主循环轮询。返回 true 时立即执行一次 mpu6050_update_10ms()，
 * 从而让阻塞式 I2C 始终位于中断之外。
 */
bool motor_take_imu_sample_request(void);

#endif // MOTOR_H
