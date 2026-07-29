#ifndef HUIDU_H
#define HUIDU_H

#include <stdint.h>

/*
 * 八路循迹探头从车体左到右排列：
 * L1=PA18, L2=PA16, L3=PB7, L4=PA17,
 * R1=PA21, R2=PA22, R3=PA24, R4=PA2。
 */
#define HUIDU_SENSOR_COUNT (8U)

/*
 * 传感器逻辑采用亚博智能八路循迹模块的数字输出语义：
 * 1 表示检测到黑色胶带，0 表示白色背景。
 * 该数组会同时在 TIMA0 控制中断和主循环串口输出中访问，因此使用 volatile。
 */
extern volatile uint8_t huidu_value[HUIDU_SENSOR_COUNT];

/* CCS 调参观察量：滤波后的黑线横向误差和当前左右轮差速修正。 */
extern volatile float huidu_line_error;
extern volatile float huidu_steer_correction;

/* 镜像读取八路 GPIO，并按车体从左到右更新黑线检测状态。 */
void huidu_get_value(void);

/* 根据八路黑线重心执行连续转向 PID，并更新两路电机目标速度。 */
void adjust_motor(void);

#endif
