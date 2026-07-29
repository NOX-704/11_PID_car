#include "mpu6050.h"

#include "delay.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

/*
 * ======================== PCB 与器件固定参数 ========================
 */
#define MPU6050_I2C_INST                 EXPANSION_I2C_INST
#define MPU6050_ADDRESS                  (0x68U)
#define MPU6050_REG_SMPLRT_DIV           (0x19U)
#define MPU6050_REG_CONFIG               (0x1AU)
#define MPU6050_REG_GYRO_CONFIG          (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG         (0x1CU)
#define MPU6050_REG_GYRO_ZOUT_H          (0x47U)
#define MPU6050_REG_PWR_MGMT_1           (0x6BU)
#define MPU6050_REG_WHO_AM_I             (0x75U)

/*
 * ±500 dps 对应 65.5 LSB/(deg/s)。胶囊轨迹半径 0.5 m，车辆正常速度下
 * 的偏航角速度远低于 500 dps，这一量程比 ±2000 dps 有更好的分辨率。
 */
#define MPU6050_GYRO_SCALE_LSB_PER_DPS    (65.5f)
#define MPU6050_CONTROL_DT_S              (0.01f)
#define MPU6050_I2C_TIMEOUT               (100000U)
#define MPU6050_CALIBRATION_SAMPLES       (200U)
#define MPU6050_CALIBRATION_INTERVAL_MS   (10U)
#define MPU6050_GYRO_FILTER_ALPHA         (0.25f)

/*
 * MPU6050 芯片正面朝上并平放时，右手坐标系的顺时针旋转通常为负值。
 * 控制层统一规定“顺时针为正”，因此默认乘 -1。若实车串口观察到手动
 * 顺时针旋转时 GZ 为负，只需把这里改成 +1.0f。
 */
#define MPU6050_CLOCKWISE_SIGN            (-1.0f)

static bool s_ready = false;
static float s_gyro_z_bias_raw = 0.0f;
static float s_gyro_z_dps = 0.0f;
static float s_yaw_deg = 0.0f;
static int s_last_error = 0;

/**
 * 等待 I2C 控制器空闲。
 */
static int mpu6050_i2c_wait_idle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(MPU6050_I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            return -1;
        }
    }
    return 0;
}

/**
 * 等待一次 I2C 传输结束，同时检查硬件错误标志。
 */
static int mpu6050_i2c_wait_done(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;
    uint32_t status;

    do {
        status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return -2;
        }
        if (--timeout == 0U) {
            return -1;
        }
    } while ((status & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U);

    return 0;
}

/**
 * 向 MPU6050 写一个寄存器。
 */
static int mpu6050_write_register(uint8_t register_address, uint8_t value)
{
    uint8_t tx_data[2] = {register_address, value};
    int result;

    result = mpu6050_i2c_wait_idle();
    if (result != 0) {
        return result;
    }

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(
        MPU6050_I2C_INST, tx_data, sizeof(tx_data));
    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        sizeof(tx_data));

    return mpu6050_i2c_wait_done();
}

/**
 * 从指定寄存器开始连续读取数据。
 */
static int mpu6050_read_registers(
    uint8_t register_address, uint8_t *data, uint8_t length)
{
    uint8_t index;
    int result;

    if ((data == NULL) || (length == 0U)) {
        return -3;
    }

    result = mpu6050_i2c_wait_idle();
    if (result != 0) {
        return result;
    }

    DL_I2C_flushControllerTXFIFO(MPU6050_I2C_INST);
    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_fillControllerTXFIFO(
        MPU6050_I2C_INST, &register_address, 1U);
    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1U);
    result = mpu6050_i2c_wait_done();
    if (result != 0) {
        return result;
    }

    result = mpu6050_i2c_wait_idle();
    if (result != 0) {
        return result;
    }

    DL_I2C_flushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_startControllerTransfer(
        MPU6050_I2C_INST,
        MPU6050_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);

    for (index = 0U; index < length; index++) {
        uint32_t timeout = MPU6050_I2C_TIMEOUT;

        while (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST)) {
            if ((DL_I2C_getControllerStatus(MPU6050_I2C_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
                return -2;
            }
            if (--timeout == 0U) {
                return -1;
            }
        }
        data[index] =
            (uint8_t) DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    }

    return mpu6050_i2c_wait_done();
}

/**
 * 读取未经零偏修正的 Z 轴陀螺仪原始值。
 */
static int mpu6050_read_gyro_z_raw(int16_t *gyro_z_raw)
{
    uint8_t data[2];
    int result;

    if (gyro_z_raw == NULL) {
        return -3;
    }

    result = mpu6050_read_registers(
        MPU6050_REG_GYRO_ZOUT_H, data, sizeof(data));
    if (result != 0) {
        return result;
    }

    *gyro_z_raw =
        (int16_t) (((uint16_t) data[0] << 8U) | data[1]);
    return 0;
}

/**
 * 把航向角限制在 [-180, 180]，避免长时间运行后数值无限增大。
 */
static float mpu6050_wrap_yaw(float yaw_deg)
{
    while (yaw_deg > 180.0f) {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg < -180.0f) {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}

bool mpu6050_init(void)
{
    uint8_t who_am_i = 0U;
    uint16_t sample_index;
    int32_t bias_sum = 0;

    s_ready = false;
    s_last_error = 0;
    s_gyro_z_bias_raw = 0.0f;
    s_gyro_z_dps = 0.0f;
    s_yaw_deg = 0.0f;

    /* 先软复位，再使用陀螺仪 X 轴 PLL 作为时钟源。 */
    if (mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x80U) != 0) {
        s_last_error = -10;
        return false;
    }
    delay_ms(100U);
    if (mpu6050_write_register(MPU6050_REG_PWR_MGMT_1, 0x01U) != 0) {
        s_last_error = -11;
        return false;
    }
    delay_ms(50U);

    if (mpu6050_read_registers(
            MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != 0) {
        s_last_error = -12;
        return false;
    }
    if (who_am_i != MPU6050_ADDRESS) {
        s_last_error = -13;
        return false;
    }

    /*
     * DLPF=3，陀螺仪内部采样基准为 1 kHz；SMPLRT_DIV=9 后输出 100 Hz，
     * 与工程 TIMA0 的 10 ms 控制周期完全一致。
     */
    if (mpu6050_write_register(MPU6050_REG_CONFIG, 0x03U) != 0) {
        s_last_error = -14;
        return false;
    }
    if (mpu6050_write_register(MPU6050_REG_SMPLRT_DIV, 0x09U) != 0) {
        s_last_error = -15;
        return false;
    }
    if (mpu6050_write_register(MPU6050_REG_GYRO_CONFIG, 0x08U) != 0) {
        s_last_error = -16;
        return false;
    }
    if (mpu6050_write_register(MPU6050_REG_ACCEL_CONFIG, 0x00U) != 0) {
        s_last_error = -17;
        return false;
    }

    /*
     * 必须在电机尚未启动且车体完全静止时校准。200 个 100 Hz 样本覆盖
     * 约 2 秒，可显著减小短直线段内的 yaw 漂移。
     */
    for (sample_index = 0U;
         sample_index < MPU6050_CALIBRATION_SAMPLES;
         sample_index++) {
        int16_t raw_value;

        if (mpu6050_read_gyro_z_raw(&raw_value) != 0) {
            s_last_error = -18;
            return false;
        }
        bias_sum += raw_value;
        delay_ms(MPU6050_CALIBRATION_INTERVAL_MS);
    }

    s_gyro_z_bias_raw =
        (float) bias_sum / (float) MPU6050_CALIBRATION_SAMPLES;
    s_ready = true;
    s_last_error = 0;
    return true;
}

bool mpu6050_update_10ms(void)
{
    int16_t raw_value;
    float measured_dps;

    if (!s_ready) {
        return false;
    }

    if (mpu6050_read_gyro_z_raw(&raw_value) != 0) {
        s_last_error = -20;
        s_ready = false;
        return false;
    }

    measured_dps =
        ((float) raw_value - s_gyro_z_bias_raw) /
        MPU6050_GYRO_SCALE_LSB_PER_DPS;
    measured_dps *= MPU6050_CLOCKWISE_SIGN;

    /*
     * 一阶低通只滤除车体和电机产生的高频振动；航向角直接积分滤波后的
     * 角速度。没有磁力计时 yaw 只能作为每段直线/半圆的短时相对量。
     */
    s_gyro_z_dps += MPU6050_GYRO_FILTER_ALPHA *
        (measured_dps - s_gyro_z_dps);
    s_yaw_deg = mpu6050_wrap_yaw(
        s_yaw_deg + s_gyro_z_dps * MPU6050_CONTROL_DT_S);
    s_last_error = 0;
    return true;
}

bool mpu6050_is_ready(void)
{
    return s_ready;
}

float mpu6050_get_gyro_z_dps(void)
{
    return s_gyro_z_dps;
}

float mpu6050_get_yaw_deg(void)
{
    return s_yaw_deg;
}

void mpu6050_reset_yaw(void)
{
    s_yaw_deg = 0.0f;
}

int mpu6050_get_last_error(void)
{
    return s_last_error;
}
