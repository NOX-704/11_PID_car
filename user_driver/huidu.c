#include "huidu.h"

#include "motor.h"
#include "ti_msp_dl_config.h"

/*
 * 循迹转向环每 10 ms 执行一次。它只根据八路探头的黑线横向位置
 * 计算左右轮目标速度差；每个车轮仍由 motor.c 的编码器速度 PID 闭环。
 */
#define TRACK_CONTROL_PERIOD_S      (0.01f)
#define TRACK_BASE_SPEED            (260.0f)
#define TRACK_MIN_SPEED             (140.0f)
#define TRACK_MAX_SPEED             (380.0f)

/*
 * 横向误差 PID 初始参数。
 *
 * 误差范围约为 -4.0~+4.0：负值表示黑线偏左，正值表示黑线偏右。
 * Kp 决定转向力度；Ki 只消除长期机械偏置；Kd 用于提前抑制过冲。
 */
#define TRACK_PID_KP                (20.0f)
#define TRACK_PID_KI                (2.0f)
#define TRACK_PID_KD                (0.12f)
#define TRACK_PID_INTEGRAL_LIMIT    (2.0f)

/*
 * 数字探头会让误差出现离散跳变。先对误差做一阶低通，再限制每个
 * 控制周期的差速变化量，避免差速在左右两侧之间瞬间翻转。
 */
#define TRACK_ERROR_FILTER_ALPHA    (0.25f)
#define TRACK_CENTER_DEADBAND       (0.05f)
#define TRACK_CORRECTION_LIMIT      (100.0f)
#define TRACK_CORRECTION_STEP       (8.0f)

/*
 * 中心黑线位于 L4/R1 间隙时可能出现八路全白。若全白前黑线已经明显
 * 偏到一侧，先保持 150 ms 原方向寻找黑线，随后逐步衰减回直行，
 * 防止丢线后永久保持大差速。
 */
#define TRACK_LOST_TRIGGER_ERROR    (2.0f)
#define TRACK_LOST_HOLD_TICKS       (15U)
#define TRACK_LOST_ERROR_DECAY      (0.85f)

/*
 * 权重以 L4/R1 之间的车体中心为零点。
 * 通道顺序固定为 L1、L2、L3、L4、R1、R2、R3、R4。
 */
static const int8_t s_black_position_weight[HUIDU_SENSOR_COUNT] = {
    -4, -3, -2, -1, 1, 2, 3, 4
};

/*
 * 亚博智能八路循迹模块使用高电平表示检测到黑线：
 * huidu_value[]=1 表示黑线，0 表示白色背景。
 */
volatile uint8_t huidu_value[HUIDU_SENSOR_COUNT] = {0U};

/*
 * 暴露滤波后的横向误差和最终差速修正量，便于在 CCS Expressions
 * 中观察和整定。它们只由 10 ms 控制中断写入。
 */
volatile float huidu_line_error = 0.0f;
volatile float huidu_steer_correction = 0.0f;

extern float target_speed_1;
extern float target_speed_2;

/* 转向 PID 内部状态。 */
static float s_filtered_error = 0.0f;
static float s_previous_error = 0.0f;
static float s_error_integral = 0.0f;
static float s_last_visible_error = 0.0f;
static float s_applied_correction = 0.0f;
static uint8_t s_has_line_history = 0U;
static uint8_t s_lost_line_ticks = 0U;

/**
 * 返回浮点数绝对值，避免为简单运算引入额外数学库依赖。
 */
static float huidu_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * 把 value 限制在 [minimum, maximum] 内。
 */
static float huidu_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/**
 * 限制一个控制周期内的变化量，防止差速修正突然反向。
 */
static float huidu_move_toward(
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

/**
 * 清除转向 PID 历史状态。
 *
 * 八路全黑等异常状态停车时必须清积分，否则重新找到黑线后会继承
 * 停车前的转向偏置。
 */
static void huidu_reset_steering_pid(void)
{
    s_filtered_error = 0.0f;
    s_previous_error = 0.0f;
    s_error_integral = 0.0f;
    s_last_visible_error = 0.0f;
    s_applied_correction = 0.0f;
    s_has_line_history = 0U;
    s_lost_line_ticks = 0U;
    huidu_line_error = 0.0f;
    huidu_steer_correction = 0.0f;
}

/**
 * 读取亚博智能八路循迹模块的单路数字输出。
 *
 * @return 1 表示检测到黑色胶带，0 表示白色背景。
 */
static uint8_t huidu_read_black_state(GPIO_Regs *port, uint32_t pin)
{
    uint32_t raw_level = DL_GPIO_readPins(port, pin) & pin;
    return (raw_level != 0U) ? 1U : 0U;
}

/**
 * 根据横向误差计算连续差速修正量。
 *
 * 计算顺序：误差低通 → 积分限幅 → PID → 输出限幅 → 变化率限制。
 * 返回值为正时黑线偏右，需要左轮加速、右轮减速；负值时相反。
 */
static float huidu_steering_pid_update(float raw_error)
{
    float derivative;
    float requested_correction;

    s_filtered_error += TRACK_ERROR_FILTER_ALPHA *
        (raw_error - s_filtered_error);

    if (huidu_absf(s_filtered_error) <= TRACK_CENTER_DEADBAND) {
        /*
         * 已经接近中心时快速释放积分，避免一个很小的历史偏置让车辆
         * 穿过中心后仍继续向原方向转动。
         */
        s_filtered_error = 0.0f;
        s_error_integral *= 0.8f;
    } else {
        s_error_integral +=
            s_filtered_error * TRACK_CONTROL_PERIOD_S;
        s_error_integral = huidu_clampf(
            s_error_integral,
            -TRACK_PID_INTEGRAL_LIMIT,
            TRACK_PID_INTEGRAL_LIMIT);
    }

    derivative = (s_filtered_error - s_previous_error) /
        TRACK_CONTROL_PERIOD_S;
    s_previous_error = s_filtered_error;

    requested_correction =
        TRACK_PID_KP * s_filtered_error +
        TRACK_PID_KI * s_error_integral +
        TRACK_PID_KD * derivative;
    requested_correction = huidu_clampf(
        requested_correction,
        -TRACK_CORRECTION_LIMIT,
        TRACK_CORRECTION_LIMIT);

    s_applied_correction = huidu_move_toward(
        s_applied_correction,
        requested_correction,
        TRACK_CORRECTION_STEP);

    huidu_line_error = s_filtered_error;
    huidu_steer_correction = s_applied_correction;
    return s_applied_correction;
}

/**
 * 读取从 L1 到 R4 的八路传感器。
 *
 * 实车确认模块安装方向与原 PCB 网络命名左右相反，因此在读取层做
 * 空间镜像：L1↔R4、L2↔R3、L3↔R2、L4↔R1。这样后续权重数组、
 * 串口输出和 PID 始终保持“索引 0 最左、索引 7 最右”的统一语义。
 */
void huidu_get_value(void)
{
    huidu_value[0] = huidu_read_black_state(XUNJI_R4_PORT, XUNJI_R4_PIN);
    huidu_value[1] = huidu_read_black_state(XUNJI_R3_PORT, XUNJI_R3_PIN);
    huidu_value[2] = huidu_read_black_state(XUNJI_R2_PORT, XUNJI_R2_PIN);
    huidu_value[3] = huidu_read_black_state(XUNJI_R1_PORT, XUNJI_R1_PIN);
    huidu_value[4] = huidu_read_black_state(XUNJI_L4_PORT, XUNJI_L4_PIN);
    huidu_value[5] = huidu_read_black_state(XUNJI_L3_PORT, XUNJI_L3_PIN);
    huidu_value[6] = huidu_read_black_state(XUNJI_L2_PORT, XUNJI_L2_PIN);
    huidu_value[7] = huidu_read_black_state(XUNJI_L1_PORT, XUNJI_L1_PIN);
}

/**
 * 用八路黑线重心 PID 连续更新两路电机目标速度。
 *
 * 通道 1/A 为右轮，通道 2/B 为左轮：
 *   correction < 0（黑线偏左）→ 右轮快、左轮慢；
 *   correction > 0（黑线偏右）→ 右轮慢、左轮快。
 */
void adjust_motor(void)
{
    int16_t weighted_error = 0;
    uint8_t black_count = 0U;
    uint8_t sensor_index;
    float raw_error;
    float correction;
    float right_target;
    float left_target;

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

    /*
     * 八路全黑无法计算可靠重心，立即把目标速度清零并清除 PID 历史，
     * 防止恢复时继承异常积分。
     */
    if (black_count == HUIDU_SENSOR_COUNT) {
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        huidu_reset_steering_pid();
        return;
    }

    if (black_count > 0U) {
        /* 多路同时压线时使用平均位置，得到 -4.0~+4.0 连续误差。 */
        raw_error = (float) weighted_error / (float) black_count;
        s_last_visible_error = raw_error;
        s_has_line_history = 1U;
        s_lost_line_ticks = 0U;
    } else if ((s_has_line_history != 0U) &&
               (huidu_absf(s_last_visible_error) >=
                TRACK_LOST_TRIGGER_ERROR)) {
        /*
         * 严重偏移后全白可能是刚刚越过黑线。短时间保持原方向寻找，
         * 超过 150 ms 后逐步衰减，避免旧差速无限锁住。
         */
        raw_error = s_last_visible_error;
        if (s_lost_line_ticks < TRACK_LOST_HOLD_TICKS) {
            s_lost_line_ticks++;
        } else {
            s_last_visible_error *= TRACK_LOST_ERROR_DECAY;
            raw_error = s_last_visible_error;
        }
    } else {
        /* 中心黑线位于 L4/R1 间隙时八路全白，目标误差为零。 */
        raw_error = 0.0f;
        s_lost_line_ticks = 0U;
    }

    correction = huidu_steering_pid_update(raw_error);

    right_target = huidu_clampf(
        TRACK_BASE_SPEED - correction,
        TRACK_MIN_SPEED,
        TRACK_MAX_SPEED);
    left_target = huidu_clampf(
        TRACK_BASE_SPEED + correction,
        TRACK_MIN_SPEED,
        TRACK_MAX_SPEED);

    target_speed_1 = right_target;
    target_speed_2 = left_target;
}
