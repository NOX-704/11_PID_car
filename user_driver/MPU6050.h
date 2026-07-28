#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* I2C 地址: ADO 接 GND = 0x68, ADO 接 VCC = 0x69 */
#define MPU6050_I2C_ADDR             (0x68U)

/* 陀螺仪满量程选择 */
#define MPU6050_GYRO_FS_250          (0x00U)
#define MPU6050_GYRO_FS_500          (0x01U)
#define MPU6050_GYRO_FS_1000         (0x02U)
#define MPU6050_GYRO_FS_2000         (0x03U)

/* DLPF 低通滤波带宽 */
#define MPU6050_DLPF_256HZ           (0x00U)
#define MPU6050_DLPF_188HZ           (0x01U)
#define MPU6050_DLPF_98HZ            (0x02U)
#define MPU6050_DLPF_42HZ            (0x03U)
#define MPU6050_DLPF_20HZ            (0x04U)
#define MPU6050_DLPF_10HZ            (0x05U)
#define MPU6050_DLPF_5HZ             (0x06U)

/* 内部寄存器地址 */
#define MPU6050_REG_SMPLRT_DIV       (0x19U)
#define MPU6050_REG_CONFIG           (0x1AU)
#define MPU6050_REG_GYRO_CONFIG      (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG     (0x1CU)
#define MPU6050_REG_PWR_MGMT_1       (0x6BU)
#define MPU6050_REG_WHO_AM_I         (0x75U)
#define MPU6050_REG_GYRO_XOUT_H      (0x43U)
#define MPU6050_REG_GYRO_ZOUT_H      (0x47U)

/* 循迹控制的工作模式 */
typedef enum {
    MPU6050_MODE_PASSIVE = 0,   /* 跟踪模块主导, 陀螺仪辅助直行保持 */
    MPU6050_MODE_ACTIVE  = 1    /* 陀螺仪主导角度控制, 覆盖跟踪模块输出 */
} mpu6050_mode_t;

/* MPU6050 内部状态 */
typedef struct {
    float    yaw_angle_deg;          /* 累计偏航角, 单位度 */
    float    gyro_z_bias;            /* Z 轴零偏, 静止时采集 */
    float    target_yaw_deg;         /* 角度环目标值 */
    float    angle_kp;               /* 角度环 P 系数 */
    float    angle_ki;               /* 角度环 I 系数 */
    float    angle_kd;               /* 角度环 D 系数 */
    float    angle_error_integral;   /* 角度误差积分 */
    float    angle_last_error;       /* 上一次角度误差 */
    float    base_speed;             /* 转向时内侧轮基础速度 */
    float    max_diff_speed;         /* 转向时最大差速 */
    uint32_t last_update_tick;       /* 上次更新计数 */
    uint8_t  ready;                  /* 传感器初始化完成标志 */
    mpu6050_mode_t mode;
} mpu6050_state_t;

extern mpu6050_state_t g_mpu6050;

/* 初始化 MPU6050, 必须在使用前调用一次 */
void mpu6050_init(void);

/* 每 10 ms 调用一次, 读取陀螺仪并更新偏航角 */
void mpu6050_update_yaw(void);

/* 获取当前偏航角 (度) */
float mpu6050_get_yaw(void);

/* 设置当前偏航角参考值 (如重置为 0) */
void mpu6050_set_yaw(float angle_deg);

/* 设置角度控制的目标角度 */
void mpu6050_set_target_yaw(float angle_deg);

/* 将 MPU6050 设定为主动模式, 目标为当前角度 ± 指定转角 */
void mpu6050_start_turn(float delta_deg);

/* 主动模式下执行角度 PID 控制, 直接修改 target_speed_1/2 */
void mpu6050_angle_control(void);

/* 检查 MPU6050 当前是否处于主动控制模式 */
uint8_t mpu6050_is_active(void);

/* 从主动模式切换回被动模式 */
void mpu6050_release_control(void);

#endif /* MPU6050_H */
