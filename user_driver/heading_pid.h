#ifndef HEADING_PID_H
#define HEADING_PID_H

#include <stdint.h>

/**
 * 初始化新的 MPU6050 航向 PID 状态。
 */
void heading_pid_init(void);

/**
 * 根据当前循迹档位和基础轮速计算 MPU6050 差速修正。
 *
 * @param tracking_gear 负数表示向左修正，正数表示向右修正，0 表示居中。
 * @param center_speed_mm_s 当前两轮基础目标速度的平均值。
 * @return 正数表示左轮加速、右轮减速，即车辆顺时针/向右修正。
 */
float heading_pid_update(
    int8_t tracking_gear, float center_speed_mm_s);

/**
 * 丢线保持原运动状态时暂停 PID，并以当前航向消除恢复时的误差突跳。
 */
void heading_pid_pause(void);

/**
 * 停车或异常时清空积分、目标航向和输出。
 */
void heading_pid_reset(void);

/** 返回 0=未工作、1=直线航向保持、2=顺时针半圆跟踪。 */
uint8_t heading_pid_get_mode(void);

/** 返回当前平滑后的差速修正，供串口调试。 */
float heading_pid_get_correction(void);

#endif /* HEADING_PID_H */
