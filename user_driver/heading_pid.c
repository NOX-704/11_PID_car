#include "heading_pid.h"

#include "mpu6050.h"
#include <stdbool.h>

/*
 * ======================== 新 MPU6050 航向 PID 参数 ========================
 *
 * PID 输出的单位是 mm/s，直接叠加到两轮目标速度：
 *   右轮 = 基础右轮 - correction
 *   左轮 = 基础左轮 + correction
 *
 * 调参顺序：
 *   1. 先保持 KI=0，只增加 KD，直到快速左右摆动明显减弱；
 *   2. 再增加 KP，让偏离方向后能够回正；
 *   3. 只有确认零偏很小但仍存在固定方向误差时，才把 KI 从 0.01 开始加。
 */
#define HEADING_PID_KP                   (2.0f)
#define HEADING_PID_KI                   (0.0f)
#define HEADING_PID_KD                   (1.0f)
#define HEADING_PID_DT_S                 (0.01f)
#define HEADING_PID_INTEGRAL_LIMIT       (20.0f)
#define HEADING_PID_OUTPUT_LIMIT         (80.0f)
#define HEADING_PID_OUTPUT_STEP          (6.0f)

/*
 * 胶囊轨迹两侧半圆半径为 0.5 m。曲线目标角速度根据车辆中心速度实时
 * 计算：omega(deg/s)=v(mm/s)/500(mm)*57.2958。
 */
#define CAPSULE_RADIUS_MM                (500.0f)
#define RAD_TO_DEG                       (57.29578f)
#define CURVE_TARGET_RATE_MIN_DPS        (12.0f)
#define CURVE_TARGET_RATE_MAX_DPS        (45.0f)

/*
 * 进入半圆：循迹达到 2 档，或实测角速度超过 8 dps。
 * 离开半圆：循迹居中且角速度低于 4 dps，连续 20 个周期（200 ms）。
 */
#define CURVE_ENTER_GEAR_LEVEL           (2)
#define CURVE_ENTER_RATE_DPS             (8.0f)
#define CURVE_EXIT_RATE_DPS              (4.0f)
#define CURVE_EXIT_CONFIRM_TICKS         (20U)

enum {
    HEADING_MODE_DISABLED = 0U,
    HEADING_MODE_STRAIGHT = 1U,
    HEADING_MODE_CURVE = 2U
};

static uint8_t s_mode = HEADING_MODE_DISABLED;
static uint8_t s_curve_exit_ticks = 0U;
static float s_target_yaw_deg = 0.0f;
static float s_integral = 0.0f;
static float s_applied_correction = 0.0f;

/** 返回浮点绝对值，避免为简单运算引入额外数学库依赖。 */
static float heading_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 把 value 限制到指定范围。 */
static float heading_clampf(
    float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/** 限制每个 10 ms 周期的差速变化量，防止 PID 输出形成硬阶跃。 */
static float heading_move_toward(
    float current, float target, float maximum_step)
{
    if (target > current + maximum_step) {
        return current + maximum_step;
    }
    if (target < current - maximum_step) {
        return current - maximum_step;
    }
    return target;
}

/** 计算跨越 ±180 度边界后的最短航向误差。 */
static float heading_wrap_error(float error_deg)
{
    while (error_deg > 180.0f) {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f) {
        error_deg += 360.0f;
    }
    return error_deg;
}

void heading_pid_init(void)
{
    heading_pid_reset();
}

float heading_pid_update(
    int8_t tracking_gear, float center_speed_mm_s)
{
    float current_yaw;
    float gyro_z_dps;
    float target_rate_dps = 0.0f;
    float heading_error;
    float requested_correction;
    bool enter_curve;

    if ((!mpu6050_is_ready()) || (center_speed_mm_s <= 0.0f)) {
        heading_pid_reset();
        return 0.0f;
    }

    current_yaw = mpu6050_get_yaw_deg();
    gyro_z_dps = mpu6050_get_gyro_z_dps();

    if (s_mode == HEADING_MODE_DISABLED) {
        s_mode = HEADING_MODE_STRAIGHT;
        s_target_yaw_deg = current_yaw;
        s_integral = 0.0f;
    }

    /*
     * 用户轨迹固定为顺时针，因此只有向右的高档循迹请求或顺时针
     * 实测角速度才能进入半圆模式；直线上的向左纠偏不会误触发半圆。
     */
    enter_curve =
        ((int) tracking_gear >= CURVE_ENTER_GEAR_LEVEL) ||
        (gyro_z_dps >= CURVE_ENTER_RATE_DPS);

    if ((s_mode == HEADING_MODE_STRAIGHT) && enter_curve) {
        /*
         * 原有阶梯差速负责把车带入弯道；检测到入弯后，目标航向开始按
         * 半径 0.5 m 的顺时针角速度连续前进，不再锁死直线航向。
         */
        s_mode = HEADING_MODE_CURVE;
        s_target_yaw_deg = current_yaw;
        s_integral = 0.0f;
        s_curve_exit_ticks = 0U;
    }

    if (s_mode == HEADING_MODE_CURVE) {
        target_rate_dps =
            center_speed_mm_s / CAPSULE_RADIUS_MM * RAD_TO_DEG;
        target_rate_dps = heading_clampf(
            target_rate_dps,
            CURVE_TARGET_RATE_MIN_DPS,
            CURVE_TARGET_RATE_MAX_DPS);
        s_target_yaw_deg = heading_wrap_error(
            s_target_yaw_deg + target_rate_dps * HEADING_PID_DT_S);

        /*
         * 只有真正恢复直行并稳定 200 ms 才退出曲线模式，避免半圆中
         * 传感器短暂居中时错误锁定当前航向。
         */
        if ((tracking_gear == 0) &&
            (heading_absf(gyro_z_dps) < CURVE_EXIT_RATE_DPS)) {
            if (s_curve_exit_ticks < CURVE_EXIT_CONFIRM_TICKS) {
                s_curve_exit_ticks++;
            }
        } else {
            s_curve_exit_ticks = 0U;
        }

        if (s_curve_exit_ticks >= CURVE_EXIT_CONFIRM_TICKS) {
            s_mode = HEADING_MODE_STRAIGHT;
            s_integral = 0.0f;
            s_curve_exit_ticks = 0U;
            target_rate_dps = 0.0f;
        }
    }

    heading_error = heading_wrap_error(
        s_target_yaw_deg - current_yaw);

    s_integral += heading_error * HEADING_PID_DT_S;
    s_integral = heading_clampf(
        s_integral,
        -HEADING_PID_INTEGRAL_LIMIT,
        HEADING_PID_INTEGRAL_LIMIT);

    /*
     * D 项直接使用“目标角速度-实测角速度”，比对含噪声的航向角做
     * 数值微分稳定。顺时针统一为正，因此正输出会让左轮更快、右轮更慢。
     */
    requested_correction =
        HEADING_PID_KP * heading_error +
        HEADING_PID_KI * s_integral +
        HEADING_PID_KD * (target_rate_dps - gyro_z_dps);
    if (s_mode == HEADING_MODE_STRAIGHT) {
        requested_correction *= 0.6f;
    }
    requested_correction = heading_clampf(
        requested_correction,
        -HEADING_PID_OUTPUT_LIMIT,
        HEADING_PID_OUTPUT_LIMIT);

    s_applied_correction = heading_move_toward(
        s_applied_correction,
        requested_correction,
        HEADING_PID_OUTPUT_STEP);
    return s_applied_correction;
}

void heading_pid_pause(void)
{
    if (mpu6050_is_ready()) {
        s_target_yaw_deg = mpu6050_get_yaw_deg();
    }
    s_integral = 0.0f;
    s_curve_exit_ticks = 0U;
}

void heading_pid_reset(void)
{
    s_mode = HEADING_MODE_DISABLED;
    s_curve_exit_ticks = 0U;
    s_target_yaw_deg = 0.0f;
    s_integral = 0.0f;
    s_applied_correction = 0.0f;
}

uint8_t heading_pid_get_mode(void)
{
    return s_mode;
}

float heading_pid_get_correction(void)
{
    return s_applied_correction;
}
