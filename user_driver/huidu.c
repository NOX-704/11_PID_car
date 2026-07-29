#include "huidu.h"

#include "motor.h"
#include "ti_msp_dl_config.h"

/*
 * ======================== 循迹控制模式开关 ========================
 *
 * 0：使用四档阶梯式差速（当前默认）。
 * 1：使用下方保留的连续转向 PID。
 *
 * 本次按实车要求先禁用转向 PID，但没有删除 PID 参数和实现。以后需要
 * 恢复时只把 TRACK_USE_STEERING_PID 改为 1U，再重新编译即可。
 * 注意：motor.c 的双轮编码器速度 PID 仍然启用；这里只禁用循迹转向 PID。
 */
#define TRACK_USE_STEERING_PID      (0U)

/*
 * ======================== 四档阶梯差速参数 ========================
 *
 * 直行为 200，弯道内侧轮固定为 150；黑线离中心越远，外侧轮依次
 * 提高到 220、260、360、460。要加强某一档转向，只提高对应 OUTER；
 * 要整体减速，则同时降低 STRAIGHT、INNER 和四个 OUTER。
 */
#define TRACK_STRAIGHT_SPEED        (200.0f)
#define TRACK_INNER_SPEED           (150.0f)
#define TRACK_OUTER_SPEED_1         (220.0f)
#define TRACK_OUTER_SPEED_2         (260.0f)
#define TRACK_OUTER_SPEED_3         (360.0f)
#define TRACK_OUTER_SPEED_4         (460.0f)
#define TRACK_MAX_OFFSET_LEVEL      (4U)

#if TRACK_USE_STEERING_PID
/*
 * ======================== 已禁用的转向 PID 参数 ========================
 *
 * 这组代码仅在 TRACK_USE_STEERING_PID=1U 时参与编译。保留用户最近手动
 * 调整的 Kp=45，方便以后继续试验。
 */
#define TRACK_CONTROL_PERIOD_S      (0.01f)
#define TRACK_BASE_SPEED            (260.0f)
#define TRACK_MIN_SPEED             (120.0f)
#define TRACK_MAX_SPEED             (400.0f)
#define TRACK_PID_KP                (45.0f)
#define TRACK_PID_KI                (2.0f)
#define TRACK_PID_KD                (0.16f)
#define TRACK_PID_INTEGRAL_LIMIT    (2.0f)
#define TRACK_ERROR_FILTER_ALPHA    (0.25f)
#define TRACK_CENTER_DEADBAND       (0.05f)
#define TRACK_CORRECTION_LIMIT      (140.0f)
#define TRACK_CORRECTION_STEP       (12.0f)
#define TRACK_LOST_TRIGGER_ERROR    (2.0f)
#define TRACK_PID_LOST_HOLD_TICKS   (15U)
#define TRACK_LOST_ERROR_DECAY      (0.85f)
#endif

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
 * CCS Expressions 观察量。阶梯模式下分别表示原始黑线重心，以及
 * (左轮目标-右轮目标)/2；PID 模式下表示滤波误差和 PID 修正量。
 */
volatile float huidu_line_error = 0.0f;
volatile float huidu_steer_correction = 0.0f;

extern float target_speed_1;
extern float target_speed_2;

/* 阶梯模式状态：记录上次有效偏移档位，用于最高档丢线搜索。 */
static uint8_t s_last_offset_level = 0U;

#if TRACK_USE_STEERING_PID
/* 连续转向 PID 的内部状态；当前控制模式下不参与编译。 */
static float s_filtered_error = 0.0f;
static float s_previous_error = 0.0f;
static float s_error_integral = 0.0f;
static float s_last_visible_error = 0.0f;
static float s_applied_correction = 0.0f;
static uint8_t s_has_line_history = 0U;
static uint8_t s_lost_line_ticks = 0U;
#endif

/**
 * 读取亚博智能八路循迹模块的单路数字输出。
 *
 * @return 1 表示检测到黑色胶带，0 表示白色背景。
 */
static uint8_t huidu_read_black_state(GPIO_Regs *port, uint32_t pin)
{
    uint32_t raw_level = DL_GPIO_readPins(port, pin) & pin;

    /*
     * 亚博官方定义：白底灭灯时 IO=1，黑线亮灯时 IO=0。
     * 读取层统一转换成“黑=1、白=0”，避免控制层到处反相。
     */
    return (raw_level == 0U) ? 1U : 0U;
}

/**
 * 根据黑线重心的平均绝对偏移选择第 1~4 档差速。
 *
 * 使用整数交叉比较，避免在 10 ms 中断中进行不必要的浮点除法。
 */
static uint8_t huidu_select_offset_level(
    uint16_t error_magnitude, uint8_t black_count)
{
    if (error_magnitude <= (uint16_t) black_count) {
        return 1U;
    }
    if (error_magnitude <= (uint16_t) (2U * black_count)) {
        return 2U;
    }
    if (error_magnitude <= (uint16_t) (3U * black_count)) {
        return 3U;
    }
    return 4U;
}

/**
 * 返回指定偏移档位对应的弯道外侧轮目标速度。
 */
static float huidu_get_outer_speed(uint8_t offset_level)
{
    switch (offset_level) {
    case 1U:
        return TRACK_OUTER_SPEED_1;
    case 2U:
        return TRACK_OUTER_SPEED_2;
    case 3U:
        return TRACK_OUTER_SPEED_3;
    default:
        return TRACK_OUTER_SPEED_4;
    }
}

/**
 * 保存并下发一组阶梯差速目标。
 *
 * 通道 1/A 是右轮，通道 2/B 是左轮。诊断修正量保持与旧 PID 相同
 * 的符号约定：正值表示左轮更快、车辆向右修正。
 */
static void huidu_set_staircase_command(
    float right_speed, float left_speed, uint8_t offset_level)
{
    s_last_offset_level = offset_level;
    target_speed_1 = right_speed;
    target_speed_2 = left_speed;
    huidu_steer_correction = (left_speed - right_speed) * 0.5f;
}

#if TRACK_USE_STEERING_PID
/**
 * 返回浮点数绝对值。
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
 * 限制一个控制周期内的输出变化量。
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
 * 清除连续转向 PID 历史状态。
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
 * 保留的连续转向 PID 计算。
 */
static float huidu_steering_pid_update(float raw_error)
{
    float derivative;
    float requested_correction;

    s_filtered_error += TRACK_ERROR_FILTER_ALPHA *
        (raw_error - s_filtered_error);

    if (huidu_absf(s_filtered_error) <= TRACK_CENTER_DEADBAND) {
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
#endif

/**
 * 按 PCB 网络名称直接读取八路传感器，并从左到右写入数组。
 *
 * 左右不再额外镜像：L1~L4 对应数组前四项，R1~R4 对应后四项。
 */
void huidu_get_value(void)
{
    huidu_value[0] = huidu_read_black_state(XUNJI_L1_PORT, XUNJI_L1_PIN);
    huidu_value[1] = huidu_read_black_state(XUNJI_L2_PORT, XUNJI_L2_PIN);
    huidu_value[2] = huidu_read_black_state(XUNJI_L3_PORT, XUNJI_L3_PIN);
    huidu_value[3] = huidu_read_black_state(XUNJI_L4_PORT, XUNJI_L4_PIN);
    huidu_value[4] = huidu_read_black_state(XUNJI_R1_PORT, XUNJI_R1_PIN);
    huidu_value[5] = huidu_read_black_state(XUNJI_R2_PORT, XUNJI_R2_PIN);
    huidu_value[6] = huidu_read_black_state(XUNJI_R3_PORT, XUNJI_R3_PIN);
    huidu_value[7] = huidu_read_black_state(XUNJI_R4_PORT, XUNJI_R4_PIN);
}

/**
 * 当前默认的四档阶梯差速控制。
 */
static void huidu_adjust_staircase(
    int16_t weighted_error, uint8_t black_count)
{
    uint16_t error_magnitude;
    uint8_t offset_level;
    float outer_speed;

    /* 八路全黑无法判断黑线中心，停车并清除搜索历史。 */
    if (black_count == HUIDU_SENSOR_COUNT) {
        huidu_line_error = 0.0f;
        huidu_set_staircase_command(0.0f, 0.0f, 0U);
        return;
    }

    /*
     * 八路全白可能是黑线位于中心间隙，也可能是严重偏移后刚越线。
     * 若上一状态是第 4 档最高差速，则不写入新目标，原左右轮目标
     * 会一直保持，直到
     * 任意探头重新识别到黑线；第 1~3 档和直行状态遇到全白则直行。
     */
    if (black_count == 0U) {
        if (s_last_offset_level == TRACK_MAX_OFFSET_LEVEL) {
            return;
        } else {
            huidu_line_error = 0.0f;
            huidu_set_staircase_command(
                TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0U);
        }
        return;
    }

    huidu_line_error =
        (float) weighted_error / (float) black_count;

    /* 黑线重心对称时按直线速度运行。 */
    if (weighted_error == 0) {
        huidu_set_staircase_command(
            TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0U);
        return;
    }

    error_magnitude = (uint16_t)
        ((weighted_error < 0) ? -weighted_error : weighted_error);
    offset_level = huidu_select_offset_level(
        error_magnitude, black_count);
    outer_speed = huidu_get_outer_speed(offset_level);

    /*
     * 黑线偏左：右轮为外侧轮，右轮加速、左轮保持 150。
     * 黑线偏右：左轮为外侧轮，左轮加速、右轮保持 150。
     */
    if (weighted_error < 0) {
        huidu_set_staircase_command(
            outer_speed, TRACK_INNER_SPEED, offset_level);
    } else {
        huidu_set_staircase_command(
            TRACK_INNER_SPEED, outer_speed, offset_level);
    }
}

#if TRACK_USE_STEERING_PID
/**
 * 保留的连续转向 PID 控制；仅在模式开关设为 1U 时编译。
 */
static void huidu_adjust_pid(
    int16_t weighted_error, uint8_t black_count)
{
    float raw_error;
    float correction;

    if (black_count == HUIDU_SENSOR_COUNT) {
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        huidu_reset_steering_pid();
        return;
    }

    if (black_count > 0U) {
        raw_error = (float) weighted_error / (float) black_count;
        s_last_visible_error = raw_error;
        s_has_line_history = 1U;
        s_lost_line_ticks = 0U;
    } else if ((s_has_line_history != 0U) &&
               (huidu_absf(s_last_visible_error) >=
                TRACK_LOST_TRIGGER_ERROR)) {
        raw_error = s_last_visible_error;
        if (s_lost_line_ticks < TRACK_PID_LOST_HOLD_TICKS) {
            s_lost_line_ticks++;
        } else {
            s_last_visible_error *= TRACK_LOST_ERROR_DECAY;
            raw_error = s_last_visible_error;
        }
    } else {
        raw_error = 0.0f;
        s_lost_line_ticks = 0U;
    }

    correction = huidu_steering_pid_update(raw_error);
    target_speed_1 = huidu_clampf(
        TRACK_BASE_SPEED - correction,
        TRACK_MIN_SPEED,
        TRACK_MAX_SPEED);
    target_speed_2 = huidu_clampf(
        TRACK_BASE_SPEED + correction,
        TRACK_MIN_SPEED,
        TRACK_MAX_SPEED);
}
#endif

/**
 * 读取传感器并按当前编译模式更新两路目标速度。
 *
 * 这里只切换循迹外环；motor.c 中的两路编码器速度 PID 始终保留。
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

#if TRACK_USE_STEERING_PID
    huidu_adjust_pid(weighted_error, black_count);
#else
    huidu_adjust_staircase(weighted_error, black_count);
#endif
}
