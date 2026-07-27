#ifndef HUIDU_H
#define HUIDU_H

#include <stdint.h>

/*
 * 八路循迹探头从车体左到右排列：
 * L1=PB6, L2=PB7, L3=PA17, L4=PA18,
 * R1=PA21, R2=PA22, R3=PA24, R4=PA25。
 */
#define HUIDU_SENSOR_COUNT (8U)

/*
 * 传感器逻辑值与板载指示灯保持一致：
 * 1 表示探头亮灯、检测到白色；0 表示不亮、检测到黑色胶带。
 * 该数组会同时在 TIMA0 控制中断和主循环串口输出中访问，因此使用 volatile。
 */
extern volatile uint8_t huidu_value[HUIDU_SENSOR_COUNT];

/* 读取八路 GPIO，并更新 huidu_value[] 的亮灯状态。 */
void huidu_get_value(void);

/* 根据不亮探头的加权位置更新两路电机目标速度。 */
void adjust_motor(void);

#endif
