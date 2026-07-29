#include "huidu.h"

#include "heading_pid.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

/*
 * ======================== 四档阶梯差速参数 ========================
 *
 * 阶梯差速继续负责根据黑线位置提供主要转弯量；MPU6050 航向 PID
 * 在此基础上只增加平滑修正，不再保留旧的纯循迹转向 PID。
 */
#define TRACK_STRAIGHT_SPEED        (200.0f)
#define TRACK_INNER_SPEED           (150.0f)
#define TRACK_OUTER_SPEED_1         (220.0f)
#define TRACK_OUTER_SPEED_2         (260.0f)
#define TRACK_OUTER_SPEED_3         (360.0f)
#define TRACK_OUTER_SPEED_4         (460.0f)
#define TRACK_MAX_OFFSET_LEVEL      (4U)
#define TRACK_TARGET_SPEED_MAX      (520.0f)

/*
 * 新档位必须连续出现 3 个 10 ms 控制周期才生效。
 * 增大可提高抗抖能力但会降低转向响应，减小则反应更快但更容易跳档。
 */
#define TRACK_GEAR_CONFIRM_TICKS    (3U)

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
 * CCS Expressions 观察量：原始黑线重心，以及阶梯差速与 MPU6050
 * 航向 PID 叠加后的 (左轮目标-右轮目标)/2。
 */
volatile float huidu_line_error = 0.0f;
volatile float huidu_steer_correction = 0.0f;

extern float target_speed_1;
extern float target_speed_2;

/*
 * 阶梯模式状态：
 * - s_applied_gear：当前实际档位，负数左偏、正数右偏、0 为直行。
 * - s_candidate_gear：等待确认的新档位。
 * - s_candidate_ticks：新档位连续出现的控制周期数。
 * - s_last_offset_level：当前实际档位绝对值，用于最高档丢线保持。
 */
static int8_t s_applied_gear = 0;
static int8_t s_candidate_gear = 0;
static uint8_t s_candidate_ticks = 0U;
static uint8_t s_last_offset_level = 0U;

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

/** 返回指定偏移档位对应的弯道外侧轮目标速度。 */
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

/** 把目标轮速限制到电机速度环允许的安全范围。 */
static float huidu_limit_target_speed(float speed)
{
    if (speed < 0.0f) {
        return 0.0f;
    }
    if (speed > TRACK_TARGET_SPEED_MAX) {
        return TRACK_TARGET_SPEED_MAX;
    }
    return speed;
}

/**
 * 保存并下发“阶梯差速 + MPU6050 航向 PID”的最终目标。
 *
 * 通道 1/A 是右轮，通道 2/B 是左轮。新的航向 PID 规定正修正表示
 * 左轮加速、右轮减速，即车辆顺时针/向右修正。
 */
static void huidu_set_staircase_command(
    float right_speed, float left_speed, int8_t applied_gear)
{
    float center_speed = (right_speed + left_speed) * 0.5f;
    float imu_correction = heading_pid_update(
        applied_gear, center_speed);
    float corrected_right = huidu_limit_target_speed(
        right_speed - imu_correction);
    float corrected_left = huidu_limit_target_speed(
        left_speed + imu_correction);
    uint8_t offset_level = (uint8_t)
        ((applied_gear < 0) ? -applied_gear : applied_gear);

    s_last_offset_level = offset_level;
    target_speed_1 = corrected_right;
    target_speed_2 = corrected_left;
    huidu_steer_correction =
        (corrected_left - corrected_right) * 0.5f;
}

/**
 * 清除待确认档位，并把当前实际档位强制设置为指定值。
 *
 * 全黑停车和非最高档全白直行属于安全/确定状态，应立即生效。
 */
static void huidu_reset_gear_debounce(int8_t applied_gear)
{
    s_applied_gear = applied_gear;
    s_candidate_gear = applied_gear;
    s_candidate_ticks = 0U;
}

/**
 * 取消尚未完成的换档确认，但保持当前实际档位不变。
 */
static void huidu_cancel_pending_gear(void)
{
    s_candidate_gear = s_applied_gear;
    s_candidate_ticks = 0U;
}

/**
 * 对请求档位执行连续样本确认，并返回当前允许下发的实际档位。
 */
static int8_t huidu_confirm_staircase_gear(int8_t requested_gear)
{
    if (requested_gear == s_applied_gear) {
        huidu_cancel_pending_gear();
        return s_applied_gear;
    }

    if (requested_gear != s_candidate_gear) {
        s_candidate_gear = requested_gear;
        s_candidate_ticks = 1U;
    } else if (s_candidate_ticks < TRACK_GEAR_CONFIRM_TICKS) {
        s_candidate_ticks++;
    }

    if (s_candidate_ticks >= TRACK_GEAR_CONFIRM_TICKS) {
        s_applied_gear = requested_gear;
        huidu_cancel_pending_gear();
    }

    return s_applied_gear;
}

/** 把带方向的实际档位转换为左右轮基础目标速度。 */
static void huidu_apply_staircase_gear(int8_t applied_gear)
{
    uint8_t offset_level;
    float outer_speed;

    if (applied_gear == 0) {
        huidu_set_staircase_command(
            TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0);
        return;
    }

    offset_level = (uint8_t)
        ((applied_gear < 0) ? -applied_gear : applied_gear);
    outer_speed = huidu_get_outer_speed(offset_level);

    if (applied_gear < 0) {
        /* 黑线偏左：右轮为外侧轮。 */
        huidu_set_staircase_command(
            outer_speed, TRACK_INNER_SPEED, applied_gear);
    } else {
        /* 黑线偏右：左轮为外侧轮。 */
        huidu_set_staircase_command(
            TRACK_INNER_SPEED, outer_speed, applied_gear);
    }
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
 * 执行四档阶梯差速，并在正常可见状态下叠加新的 MPU6050 航向 PID。
 */
static void huidu_adjust_staircase(
    int16_t weighted_error, uint8_t black_count)
{
    uint16_t error_magnitude;
    uint8_t offset_level;
    int8_t requested_gear;
    int8_t applied_gear;

    /* 八路全黑无法判断黑线中心，立即停车并清除全部控制历史。 */
    if (black_count == HUIDU_SENSOR_COUNT) {
        huidu_line_error = 0.0f;
        huidu_reset_gear_debounce(0);
        heading_pid_reset();
        huidu_set_staircase_command(0.0f, 0.0f, 0);
        return;
    }

    /*
     * 八路全白可能是黑线位于中心间隙，也可能是严重偏移后刚越线。
     * 若上一状态是第 4 档，则保持原左右轮目标，直到探头重新看到黑线。
     * 暂停航向 PID 只同步内部目标，不改变已经下发的电机运动状态。
     */
    if (black_count == 0U) {
        if (s_last_offset_level == TRACK_MAX_OFFSET_LEVEL) {
            huidu_cancel_pending_gear();
            heading_pid_pause();
            return;
        }

        huidu_line_error = 0.0f;
        huidu_reset_gear_debounce(0);
        huidu_set_staircase_command(
            TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0);
        return;
    }

    huidu_line_error =
        (float) weighted_error / (float) black_count;

    if (weighted_error == 0) {
        requested_gear = 0;
    } else {
        error_magnitude = (uint16_t)
            ((weighted_error < 0) ? -weighted_error : weighted_error);
        offset_level = huidu_select_offset_level(
            error_magnitude, black_count);
        requested_gear = (weighted_error < 0) ?
            -(int8_t) offset_level : (int8_t) offset_level;
    }

    /*
     * 只有同一带方向档位连续出现 TRACK_GEAR_CONFIRM_TICKS 次才切换；
     * 确认期间继续使用原档位，避免相邻探头抖动造成左右反复跳变。
     */
    applied_gear = huidu_confirm_staircase_gear(requested_gear);
    huidu_apply_staircase_gear(applied_gear);
}

/**
 * 读取传感器，先生成防抖阶梯差速，再叠加 MPU6050 航向 PID。
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

    huidu_adjust_staircase(weighted_error, black_count);
}
