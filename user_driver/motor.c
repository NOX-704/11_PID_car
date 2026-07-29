#include "motor.h"

/* TIMG0 的周期来自原工程 empty.syscfg，PWM 合法范围为 0~4000。 */
#define MOTOR_PWM_MAX_DUTY (4000U)

/**
 * 初始化 TB6612FNG 底层输出。
 *
 * 初始化时两路方向脚均置高、PWM 置零，保持原工程的安全制动状态。
 * 本函数只启动 PWM 定时器，不启动任何业务控制循环。
 */
void motor_init(uint8_t motor_id)
{
    if ((motor_id == 1U) || (motor_id == 3U)) {
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, 0U, GPIO_PWMAB_C0_IDX);
    }

    if ((motor_id == 2U) || (motor_id == 3U)) {
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, 0U, GPIO_PWMAB_C1_IDX);
    }

    DL_Timer_startCounter(PWMAB_INST);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
}

/** 把 PWM 请求限制到 SysConfig 的 TIMG0 周期范围。 */
static uint32_t motor_limit_duty(uint32_t duty)
{
    return (duty > MOTOR_PWM_MAX_DUTY) ?
        MOTOR_PWM_MAX_DUTY : duty;
}

/** 写入指定 TB6612 通道的 PWM 比较值。 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    uint32_t limited_duty = motor_limit_duty(duty);

    if (motor_id == 1U) {
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, limited_duty, GPIO_PWMAB_C0_IDX);
    } else if (motor_id == 2U) {
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, limited_duty, GPIO_PWMAB_C1_IDX);
    }
}

/**
 * 设置指定 TB6612 通道的方向。
 *
 * direction=0：原工程制动电平；direction=1：车辆前进；
 * direction=2：车辆后退。两轮实际方向电平完全沿用 11_PID_car。
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if (motor_id == 1U) {
        if (direction == 0U) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == 1U) {
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        } else if (direction == 2U) {
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    } else if (motor_id == 2U) {
        if (direction == 0U) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == 1U) {
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        } else if (direction == 2U) {
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}
