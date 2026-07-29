#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * 复制自 11_PID_car 的 TB6612FNG 底层接口。
 * motor_id=1 对应 A 通道/右轮，motor_id=2 对应 B 通道/左轮，
 * motor_id=3 表示同时初始化两路。
 */
void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif /* MOTOR_H */
