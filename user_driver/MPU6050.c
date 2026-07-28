#include "MPU6050.h"
#include "ti_msp_dl_config.h"
#include "motor.h"

/* 陀螺仪量程对应的灵敏度 (LSB / °/s) */
#define MPU6050_GYRO_SENS_250     (131.0f)
#define MPU6050_GYRO_SENS_500     (65.5f)
#define MPU6050_GYRO_SENS_1000    (32.8f)
#define MPU6050_GYRO_SENS_2000    (16.4f)

/* 实际使用的量程 */
#define MPU6050_CURRENT_GYRO_FS  MPU6050_GYRO_FS_2000
#define MPU6050_CURRENT_SENS     MPU6050_GYRO_SENS_2000

/* 零偏校准采样次数 */
#define MPU6050_CALIB_SAMPLES    (200U)

/* 角度环收敛阈值 (度) */
#define MPU6050_TURN_TOLERANCE   (3.0f)

/* 主动转向时内侧轮基础速度 */
#define MPU6050_TURN_BASE_SPEED  (150.0f)
/* 角度环最大差速 */
#define MPU6050_MAX_DIFF_SPEED   (300.0f)

/* 角度标准化到 [-180, 180] */
#define NORMALIZE_ANGLE(a)                                                 \
    do {                                                                   \
        while ((a) > 180.0f)  { (a) -= 360.0f; }                          \
        while ((a) < -180.0f) { (a) += 360.0f; }                          \
    } while (0)

mpu6050_state_t g_mpu6050;

extern float target_speed_1;
extern float target_speed_2;

/* ---- 底层 I2C 操作 ---- */

static uint8_t mpu6050_read_byte(uint8_t reg)
{
    uint8_t data = 0;
    uint32_t timeout;

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU6050_INST,
        MPU6050_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    timeout = 10000U;
    while ((DL_I2C_getControllerStatus(MPU6050_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}

    DL_I2C_startControllerTransfer(MPU6050_INST,
        MPU6050_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    timeout = 10000U;
    while ((DL_I2C_getControllerStatus(MPU6050_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}
    data = (uint8_t) DL_I2C_receiveControllerData(MPU6050_INST);

    return data;
}

static void mpu6050_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t timeout;
    uint8_t i;

    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU6050_INST,
        MPU6050_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    timeout = 10000U;
    while ((DL_I2C_getControllerStatus(MPU6050_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}

    DL_I2C_startControllerTransfer(MPU6050_INST,
        MPU6050_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    timeout = 10000U;
    while ((DL_I2C_getControllerStatus(MPU6050_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}

    for (i = 0U; i < len; i++) {
        buf[i] = (uint8_t) DL_I2C_receiveControllerData(MPU6050_INST);
    }
}

static void mpu6050_write_byte(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    uint32_t timeout;

    buf[0] = reg;
    buf[1] = data;
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, buf, 2);
    DL_I2C_startControllerTransfer(MPU6050_INST,
        MPU6050_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    timeout = 10000U;
    while ((DL_I2C_getControllerStatus(MPU6050_INST)
            & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}
}

/* ---- 传感器初始化 ---- */

void mpu6050_init(void)
{
    uint8_t who_am_i;
    float bias_sum = 0.0f;
    int16_t raw_z;
    uint8_t buf[2];
    uint16_t i;

    g_mpu6050.yaw_angle_deg        = 0.0f;
    g_mpu6050.gyro_z_bias          = 0.0f;
    g_mpu6050.target_yaw_deg       = 0.0f;
    g_mpu6050.angle_kp             = 2.5f;
    g_mpu6050.angle_ki             = 0.02f;
    g_mpu6050.angle_kd             = 0.5f;
    g_mpu6050.angle_error_integral = 0.0f;
    g_mpu6050.angle_last_error     = 0.0f;
    g_mpu6050.base_speed           = MPU6050_TURN_BASE_SPEED;
    g_mpu6050.max_diff_speed       = MPU6050_MAX_DIFF_SPEED;
    g_mpu6050.last_update_tick     = 0U;
    g_mpu6050.ready                = 0U;
    g_mpu6050.mode                 = MPU6050_MODE_PASSIVE;

    /* 唤醒 MPU6050, 清除睡眠位 */
    mpu6050_write_byte(MPU6050_REG_PWR_MGMT_1, 0x00U);

    /* 简单延时等待稳定 */
    for (i = 0U; i < 60000U; i++) { __asm(" nop"); }

    /* 检查 WHO_AM_I */
    who_am_i = mpu6050_read_byte(MPU6050_REG_WHO_AM_I);
    if (who_am_i != 0x68U) {
        return;
    }

    /* 采样率分频: SMPLRT_DIV = 0 → 1kHz 内部采样率 */
    mpu6050_write_byte(MPU6050_REG_SMPLRT_DIV, 0x00U);

    /* DLPF 配置: 带宽 20Hz, 适合小车滤除高频振动 */
    mpu6050_write_byte(MPU6050_REG_CONFIG, MPU6050_DLPF_20HZ);

    /* 陀螺仪量程 ±2000 °/s */
    mpu6050_write_byte(MPU6050_REG_GYRO_CONFIG,
        (MPU6050_CURRENT_GYRO_FS << 3));

    /* 加速度计量程 ±2g (默认值, 可不设) */
    mpu6050_write_byte(MPU6050_REG_ACCEL_CONFIG, 0x00U);

    /* 延时等内部滤波器稳定 */
    for (i = 0U; i < 60000U; i++) { __asm(" nop"); }

    /* Z 轴零偏标定: 静止时采样取平均 */
    for (i = 0U; i < MPU6050_CALIB_SAMPLES; i++) {
        mpu6050_read_bytes(MPU6050_REG_GYRO_ZOUT_H, buf, 2);
        raw_z = (int16_t)(((uint16_t) buf[0] << 8) | (uint16_t) buf[1]);
        bias_sum += (float) raw_z;
    }
    g_mpu6050.gyro_z_bias = bias_sum / (float) MPU6050_CALIB_SAMPLES;

    g_mpu6050.ready = 1U;
}

/* ---- 读取 Z 轴角速度 ---- */

static float mpu6050_read_gyro_z(void)
{
    int16_t raw_z;
    uint8_t buf[2];

    mpu6050_read_bytes(MPU6050_REG_GYRO_ZOUT_H, buf, 2);
    raw_z = (int16_t)(((uint16_t) buf[0] << 8) | (uint16_t) buf[1]);

    return ((float) raw_z - g_mpu6050.gyro_z_bias) / MPU6050_CURRENT_SENS;
}

/* ---- 偏航角更新 (每 10 ms 调用一次) ---- */

void mpu6050_update_yaw(void)
{
    float gyro_z_dps;   /* 角速度, 单位 °/s */

    if (g_mpu6050.ready == 0U) {
        return;
    }

    gyro_z_dps = mpu6050_read_gyro_z();

    /*
     * 死区过滤: 角速度绝对值小于 0.5 °/s 视为静止噪声,
     * 直接清零, 避免零偏漂移累积。
     */
    if (gyro_z_dps > -0.5f && gyro_z_dps < 0.5f) {
        gyro_z_dps = 0.0f;
    }

    /* 欧拉积分: dt = 0.01 s */
    g_mpu6050.yaw_angle_deg += gyro_z_dps * 0.01f;

    /* 保持角度在 [-180, 180] 范围 */
    NORMALIZE_ANGLE(g_mpu6050.yaw_angle_deg);
}

/* ---- 公共接口 ---- */

float mpu6050_get_yaw(void)
{
    return g_mpu6050.yaw_angle_deg;
}

void mpu6050_set_yaw(float angle_deg)
{
    g_mpu6050.yaw_angle_deg = angle_deg;
    NORMALIZE_ANGLE(g_mpu6050.yaw_angle_deg);
}

void mpu6050_set_target_yaw(float angle_deg)
{
    g_mpu6050.target_yaw_deg = angle_deg;
    NORMALIZE_ANGLE(g_mpu6050.target_yaw_deg);
    g_mpu6050.angle_error_integral = 0.0f;
    g_mpu6050.angle_last_error     = 0.0f;
}

void mpu6050_start_turn(float delta_deg)
{
    float target;

    target = g_mpu6050.yaw_angle_deg + delta_deg;
    NORMALIZE_ANGLE(target);
    mpu6050_set_target_yaw(target);

    g_mpu6050.mode = MPU6050_MODE_ACTIVE;
}

void mpu6050_release_control(void)
{
    g_mpu6050.mode             = MPU6050_MODE_PASSIVE;
    g_mpu6050.angle_error_integral = 0.0f;
    g_mpu6050.angle_last_error     = 0.0f;
}

uint8_t mpu6050_is_active(void)
{
    return (g_mpu6050.mode == MPU6050_MODE_ACTIVE) ? 1U : 0U;
}

/* ---- 角度 PID 控制 ---- */

void mpu6050_angle_control(void)
{
    float error;
    float p_term, i_term, d_term;
    float diff;

    if (g_mpu6050.mode != MPU6050_MODE_ACTIVE) {
        return;
    }

    error = g_mpu6050.target_yaw_deg - g_mpu6050.yaw_angle_deg;
    NORMALIZE_ANGLE(error);

    /* 进入死区则切换回跟踪模式 */
    if (error > -MPU6050_TURN_TOLERANCE && error < MPU6050_TURN_TOLERANCE) {
        target_speed_1 = g_mpu6050.base_speed;
        target_speed_2 = g_mpu6050.base_speed;
        mpu6050_release_control();
        return;
    }

    /* 增量式 PID */
    p_term = g_mpu6050.angle_kp * error;

    g_mpu6050.angle_error_integral += error * 0.01f;
    if (g_mpu6050.angle_error_integral >  200.0f) {
        g_mpu6050.angle_error_integral =  200.0f;
    }
    if (g_mpu6050.angle_error_integral < -200.0f) {
        g_mpu6050.angle_error_integral = -200.0f;
    }
    i_term = g_mpu6050.angle_ki * g_mpu6050.angle_error_integral;

    d_term = g_mpu6050.angle_kd * (error - g_mpu6050.angle_last_error) / 0.01f;
    g_mpu6050.angle_last_error = error;

    diff = p_term + i_term + d_term;

    /* 限幅 */
    if (diff > g_mpu6050.max_diff_speed) {
        diff = g_mpu6050.max_diff_speed;
    }
    if (diff < -g_mpu6050.max_diff_speed) {
        diff = -g_mpu6050.max_diff_speed;
    }

    /*
     * 通道 1 = 右轮, 通道 2 = 左轮
     * error > 0 → 需要右转 → 左轮快、右轮慢
     * diff 正 → 右轮 = base - diff (慢), 左轮 = base + diff (快) → 右转
     */
    target_speed_1 = g_mpu6050.base_speed - diff;
    target_speed_2 = g_mpu6050.base_speed + diff;

    /* 保证电机正向转动 */
    if (target_speed_1 < 0.0f) { target_speed_1 = 0.0f; }
    if (target_speed_2 < 0.0f) { target_speed_2 = 0.0f; }

    motor_set_direction(1U, 1U);
    motor_set_direction(2U, 1U);
}
