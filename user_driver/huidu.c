#include "huidu.h"

#include "heading_pid.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

/*
 * ======================== 连续循迹 PD 参数 ========================
 *
 * 用浮点重心偏移量驱动差速，输出连续值替代原来的 4 级阶梯档位。
 * 差速 (mm/s) = KP × 重心偏移 + KD × 偏移变化率。
 *
 * huidu_line_error 范围约 [-4, 4]，KP=85 → 最大差速 ±340 mm/s。
 */
#define TRACK_CENTER_SPEED       (300.0f)
#define TRACK_MAX_DIFF           (340.0f)
#define TRACK_PD_KP              (85.0f)
#define TRACK_PD_KD              (15.0f)
#define TRACK_LPF_ALPHA          (0.3f)
#define TRACK_TARGET_SPEED_MAX   (640.0f)

/*
 * 虚拟档位阈值，用于 heading_pid_update 的入弯/出弯判断。
 * 只对航向 PID 可见，不影响实际差速输出。
 */
#define TRACK_VGEAR_DEADBAND     (25.0f)
#define TRACK_VGEAR_LEVEL_2      (100.0f)
#define TRACK_VGEAR_LEVEL_3      (160.0f)
#define TRACK_VGEAR_LEVEL_4      (260.0f)

/*
 * 八路全白丢线保持：上一周期差速绝对值超过此阈值时保持原运动状态。
 */
#define TRACK_HOLD_THRESHOLD     (200.0f)

/*
 * 权重以 L4/R1 之间的车体中心为零点。
 * 通道顺序固定为 L1、L2、L3、L4、R1、R2、R3、R4。
 */
static const int8_t s_black_position_weight[HUIDU_SENSOR_COUNT] = {
    -4, -3, -2, -1, 1, 2, 3, 4
};

/*
 * 亚博智能八路循迹模块的 GPIO 数字输出为低电平有效：
 * huidu_value[]=1 表示黑线，0 表示白色背景。
 */
volatile uint8_t huidu_value[HUIDU_SENSOR_COUNT] = {0U};

/*
 * CCS Expressions 观察量：原始黑线重心，以及输出的半差速。
 */
volatile float huidu_line_error = 0.0f;
volatile float huidu_steer_correction = 0.0f;

extern float target_speed_1;
extern float target_speed_2;

/*
 * 连续 PD 状态：
 * - s_line_error_filtered：低通滤波后的重心偏移，用于 P 项和 D 项。
 * - s_prev_line_error：上一周期的滤波值，用于 D 项（微分）。
 * - s_prev_track_diff：上一周期 PD 输出的差速，用于全白丢线保持判断。
 */
static float s_line_error_filtered = 0.0f;
static float s_prev_line_error = 0.0f;
static float s_prev_track_diff = 0.0f;

/**
 * 读取亚博智能八路循迹模块的单路数字输出。
 *
 * @return 1 表示检测到黑色胶带，0 表示白色背景。
 */
static uint8_t huidu_read_black_state(GPIO_Regs *port, uint32_t pin)
{
    uint32_t raw_level = DL_GPIO_readPins(port, pin) & pin;

    /*
     * 亚博模块白底灭灯时 IO=1，黑线亮灯时 IO=0。读取层统一转换成
     * “黑=1、白=0”，避免控制层重复反相。
     */
    return (raw_level == 0U) ? 1U : 0U;
}

/**
 * 连续循迹 PD，替代原来基于档位的阶梯差速。
 *
 * 输入：传感器加权误差和黑点数。
 * 直接输出 target_speed_1（右轮）和 target_speed_2（左轮）。
 */
static void huidu_tracking_pd(
    int16_t weighted_error, uint8_t black_count)
{
    float track_diff;
    float heading_correction;
    float left_speed, right_speed;
    float abs_diff;
    int8_t virtual_gear;

    /* 八路全黑无法判断黑线中心，立即停车并清除全部控制历史。 */
    if (black_count == HUIDU_SENSOR_COUNT) {
        huidu_line_error = 0.0f;
        s_line_error_filtered = 0.0f;
        s_prev_line_error = 0.0f;
        s_prev_track_diff = 0.0f;
        heading_pid_reset();
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        huidu_steer_correction = 0.0f;
        return;
    }

    /*
     * 八路全白可能是黑线位于中心间隙，也可能是严重偏移后刚越线。
     * 若上一周期差速较大，保持原运动状态并暂停航向 PID，
     * 直到探头重新看到黑线。
     */
    if (black_count == 0U) {
        abs_diff = (s_prev_track_diff < 0.0f) ?
            -s_prev_track_diff : s_prev_track_diff;
        if (abs_diff > TRACK_HOLD_THRESHOLD) {
            heading_pid_pause();
            return;
        }

        huidu_line_error = 0.0f;
        s_line_error_filtered = 0.0f;
        s_prev_line_error = 0.0f;
        s_prev_track_diff = 0.0f;
        heading_correction = heading_pid_update(0, TRACK_CENTER_SPEED);
        target_speed_1 = TRACK_CENTER_SPEED - heading_correction;
        target_speed_2 = TRACK_CENTER_SPEED + heading_correction;
        huidu_steer_correction = heading_correction;
        return;
    }

    /* 计算黑线重心偏移量。 */
    huidu_line_error =
        (float) weighted_error / (float) black_count;

    /* 低通滤波抑制传感器瞬时抖动。 */
    s_line_error_filtered =
        TRACK_LPF_ALPHA * huidu_line_error +
        (1.0f - TRACK_LPF_ALPHA) * s_line_error_filtered;

    /* 连续 PD：差速 = P(比例) + D(微分阻尼)。 */
    {
        float line_delta = s_line_error_filtered - s_prev_line_error;
        s_prev_line_error = s_line_error_filtered;

        track_diff =
            TRACK_PD_KP * s_line_error_filtered +
            TRACK_PD_KD * line_delta;
    }

    /* 限幅到允许的最大差速范围。 */
    if (track_diff > TRACK_MAX_DIFF) {
        track_diff = TRACK_MAX_DIFF;
    } else if (track_diff < -TRACK_MAX_DIFF) {
        track_diff = -TRACK_MAX_DIFF;
    }

    s_prev_track_diff = track_diff;

    /*
     * 将连续差速映射为虚拟档位，供 heading_pid_update 进行
     * 入弯/出弯模式切换。该档位仅对航向 PID 可见，不影响实际差速。
     */
    abs_diff = (track_diff < 0.0f) ? -track_diff : track_diff;
    if (abs_diff < TRACK_VGEAR_DEADBAND) {
        virtual_gear = 0;
    } else if (abs_diff < TRACK_VGEAR_LEVEL_2) {
        virtual_gear = 1;
    } else if (abs_diff < TRACK_VGEAR_LEVEL_3) {
        virtual_gear = 2;
    } else if (abs_diff < TRACK_VGEAR_LEVEL_4) {
        virtual_gear = 3;
    } else {
        virtual_gear = 4;
    }
    if (track_diff < 0.0f) {
        virtual_gear = -virtual_gear;
    }

    /* MPU6050 航向 PID 修正。 */
    heading_correction = heading_pid_update(
        virtual_gear, TRACK_CENTER_SPEED);

    /*
     * 融合：中心速度 ± 循迹差速的一半 ± 航向修正。
     *
     *   - track_diff 正 = 线在右 = 需右转 = 左轮加速
     *   - heading_correction 正 = 顺时针 = 左轮加速
     *   - 右轮目标 = target_speed_1，左轮目标 = target_speed_2
     */
    left_speed  = TRACK_CENTER_SPEED + track_diff * 0.5f +
                  heading_correction;
    right_speed = TRACK_CENTER_SPEED - track_diff * 0.5f -
                  heading_correction;

    /* 安全限幅：禁止负速和超速。 */
    if (left_speed < 0.0f) {
        left_speed = 0.0f;
    } else if (left_speed > TRACK_TARGET_SPEED_MAX) {
        left_speed = TRACK_TARGET_SPEED_MAX;
    }
    if (right_speed < 0.0f) {
        right_speed = 0.0f;
    } else if (right_speed > TRACK_TARGET_SPEED_MAX) {
        right_speed = TRACK_TARGET_SPEED_MAX;
    }

    target_speed_1 = right_speed;
    target_speed_2 = left_speed;
    huidu_steer_correction = (left_speed - right_speed) * 0.5f;
}

/**
 * 按 PCB 网络名称直接读取八路传感器，并从左到右写入数组。
 */
void huidu_get_value(void)
{
    huidu_value[0] = huidu_read_black_state(
        XUNJI_L1_PORT, XUNJI_L1_PIN);
    huidu_value[1] = huidu_read_black_state(
        XUNJI_L2_PORT, XUNJI_L2_PIN);
    huidu_value[2] = huidu_read_black_state(
        XUNJI_L3_PORT, XUNJI_L3_PIN);
    huidu_value[3] = huidu_read_black_state(
        XUNJI_L4_PORT, XUNJI_L4_PIN);
    huidu_value[4] = huidu_read_black_state(
        XUNJI_R1_PORT, XUNJI_R1_PIN);
    huidu_value[5] = huidu_read_black_state(
        XUNJI_R2_PORT, XUNJI_R2_PIN);
    huidu_value[6] = huidu_read_black_state(
        XUNJI_R3_PORT, XUNJI_R3_PIN);
    huidu_value[7] = huidu_read_black_state(
        XUNJI_R4_PORT, XUNJI_R4_PIN);
}

/**
 * 读取传感器，计算黑线重心，执行连续循迹 PD + 航向 PID 融合。
 */
void adjust_motor(void)
{
    int16_t weighted_error = 0;
    uint8_t black_count = 0U;
    uint8_t sensor_index;

    huidu_get_value();
    motor_set_direction(1U, 1U);
    motor_set_direction(2U, 1U);

    for (sensor_index = 0U;
         sensor_index < HUIDU_SENSOR_COUNT;
         sensor_index++) {
        if (huidu_value[sensor_index] != 0U) {
            weighted_error += s_black_position_weight[sensor_index];
            black_count++;
        }
    }

    huidu_tracking_pd(weighted_error, black_count);
}
