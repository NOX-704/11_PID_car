#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MPU6050 固定接在 PCB H1/H6/H7 的共享 I2C1 总线上：
 * PB2=O_SCL，PB3=O_SDA，AD0 接地时器件地址为 0x68。
 */

/**
 * 初始化 MPU6050，并在车辆静止时完成 Z 轴陀螺仪零偏校准。
 *
 * @return true 表示 WHO_AM_I、寄存器配置和零偏校准全部成功。
 */
bool mpu6050_init(void);

/**
 * 读取一次 Z 轴角速度并积分得到短时间相对航向角。
 *
 * 本工程控制周期固定为 10 ms，因此该函数应由主循环每个控制节拍
 * 调用一次，禁止放入定时器中断中执行阻塞式 I2C 传输。
 *
 * @return true 表示本次采样成功。
 */
bool mpu6050_update_10ms(void);

/** 返回 MPU6050 是否已经成功初始化并完成零偏校准。 */
bool mpu6050_is_ready(void);

/** 返回以顺时针为正方向的 Z 轴角速度，单位为度每秒。 */
float mpu6050_get_gyro_z_dps(void);

/** 返回以顺时针为正方向的短时间相对航向角，范围为 [-180, 180] 度。 */
float mpu6050_get_yaw_deg(void);

/** 把当前车头方向重新定义为 0 度，不改变陀螺仪零偏。 */
void mpu6050_reset_yaw(void);

/** 返回最近一次驱动错误码，0 表示正常。 */
int mpu6050_get_last_error(void);

#endif /* MPU6050_H */
