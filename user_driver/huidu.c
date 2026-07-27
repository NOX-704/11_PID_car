#include "huidu.h"

#include "motor.h"
#include "ti_msp_dl_config.h"

/*
 * 继续使用当前已验证的差速参数：直行/内侧通道为 150，黑带越靠外，
 * 外侧通道依次提高到 250、300、350、400。
 */
#define TRACK_STRAIGHT_SPEED (150.0f)
#define TRACK_INNER_SPEED    (150.0f)
#define TRACK_OUTER_SPEED_1  (250.0f)
#define TRACK_OUTER_SPEED_2  (300.0f)
#define TRACK_OUTER_SPEED_3  (350.0f)
#define TRACK_OUTER_SPEED_4  (400.0f)

/*
 * TIMA0 每 10 ms 调用一次 adjust_motor()。目标轮速用 20 个控制周期
 * 完成一次线性过渡，使速度 PID 的 PWM 输入不再发生阶梯突变。
 */
#define TRACK_CONTROL_PERIOD_MS  (10U)
#define TRACK_RAMP_DURATION_MS   (200U)
#define TRACK_RAMP_TICKS         \
    (TRACK_RAMP_DURATION_MS / TRACK_CONTROL_PERIOD_MS)

/*
 * 第 3、4 档表示黑带已经明显偏到外侧。此时若下一帧八路全亮，
 * 更可能是车辆冲过胶带而不是胶带恰好居中，因此继续保持原转向。
 */
#define TRACK_LARGE_OFFSET_LEVEL (3U)

/*
 * 权重以 L4/R1 之间的车体中心为零点。
 * 负值表示黑带偏左，正值表示黑带偏右；绝对值越大，偏离中心越远。
 */
static const int8_t s_black_position_weight[HUIDU_SENSOR_COUNT] = {
    -4, -3, -2, -1, 1, 2, 3, 4
};

/*
 * huidu_value[] 保存的是“灯是否亮”，不是“是否压到黑带”。
 * 当前模块为低电平有效：白色物体使指示灯点亮，同时 GPIO 输出低电平。
 */
volatile uint8_t huidu_value[HUIDU_SENSOR_COUNT] = {0U};

extern float target_speed_1;
extern float target_speed_2;

/* 最近一次非全亮状态的差速决策，供严重偏移后的全亮状态继续保持。 */
static float s_last_command_speed_1 = TRACK_STRAIGHT_SPEED;
static float s_last_command_speed_2 = TRACK_STRAIGHT_SPEED;
static uint8_t s_last_offset_level = 0U;

/* 目标轮速斜坡的起点、终点和当前进度，由 10 ms 控制中断独占访问。 */
static float s_ramp_start_speed_1 = 0.0f;
static float s_ramp_start_speed_2 = 0.0f;
static float s_ramp_end_speed_1 = 0.0f;
static float s_ramp_end_speed_2 = 0.0f;
static uint8_t s_ramp_tick = TRACK_RAMP_TICKS;

/**
 * 读取单路循迹 GPIO，并转换为与物理指示灯一致的逻辑值。
 *
 * @return 1 表示亮灯/白色，0 表示不亮/黑色胶带。
 */
static uint8_t huidu_read_lit_state(GPIO_Regs *port, uint32_t pin)
{
    uint32_t raw_level = DL_GPIO_readPins(port, pin) & pin;
    return (raw_level == 0U) ? 1U : 0U;
}

/**
 * 根据黑带重心距离选择旧版差速档位。
 *
 * 使用“权重绝对值总和 / 不亮探头数量”的等价整数比较，避免在 10 ms
 * 中断中执行除法。平均偏移落在第 1~4 档时，返回对应的档位编号。
 */
static uint8_t huidu_select_offset_level(
    uint16_t error_magnitude, uint8_t unlit_count)
{
    if (error_magnitude <= (uint16_t) unlit_count) {
        return 1U;
    }
    if (error_magnitude <= (uint16_t) (2U * unlit_count)) {
        return 2U;
    }
    if (error_magnitude <= (uint16_t) (3U * unlit_count)) {
        return 3U;
    }
    return 4U;
}

/**
 * 把偏移档位转换为弯道外侧车轮的目标速度。
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
 * 在 200 ms 内把两路目标轮速从当前值线性过渡到新指令。
 *
 * 这里平滑的是速度 PID 的目标值，而不是绕过 PID 直接改 PWM。
 * 因此编码器闭环仍然有效，同时 PWM 不会因差速档位切换产生阶跃。
 */
static void huidu_apply_speed_ramp(
    float command_speed_1, float command_speed_2)
{
    float progress;

    /*
     * 指令变化时从当前已经输出的目标轮速重新起步。即使传感器连续
     * 跨越多个档位，也不会突然跳到新档位。
     */
    if ((command_speed_1 != s_ramp_end_speed_1) ||
        (command_speed_2 != s_ramp_end_speed_2)) {
        s_ramp_start_speed_1 = target_speed_1;
        s_ramp_start_speed_2 = target_speed_2;
        s_ramp_end_speed_1 = command_speed_1;
        s_ramp_end_speed_2 = command_speed_2;
        s_ramp_tick = 0U;
    }

    if (s_ramp_tick < TRACK_RAMP_TICKS) {
        s_ramp_tick++;
        progress = (float) s_ramp_tick / (float) TRACK_RAMP_TICKS;
        target_speed_1 = s_ramp_start_speed_1 +
            (s_ramp_end_speed_1 - s_ramp_start_speed_1) * progress;
        target_speed_2 = s_ramp_start_speed_2 +
            (s_ramp_end_speed_2 - s_ramp_start_speed_2) * progress;
    } else {
        /* 消除浮点插值尾差，完成后固定为准确的档位目标值。 */
        target_speed_1 = s_ramp_end_speed_1;
        target_speed_2 = s_ramp_end_speed_2;
    }
}

/**
 * 保存本次有效循迹决策，并通过 200 ms 斜坡更新 PID 目标轮速。
 */
static void huidu_set_command(
    float command_speed_1, float command_speed_2, uint8_t offset_level)
{
    s_last_command_speed_1 = command_speed_1;
    s_last_command_speed_2 = command_speed_2;
    s_last_offset_level = offset_level;
    huidu_apply_speed_ramp(command_speed_1, command_speed_2);
}

/**
 * 读取从 L1 到 R4 的八路传感器。
 *
 * 顺序固定为 L1、L2、L3、L4、R1、R2、R3、R4，便于串口输出和
 * 黑带位置权重一一对应。
 */
void huidu_get_value(void)
{
    huidu_value[0] = huidu_read_lit_state(XUNJI_L1_PORT, XUNJI_L1_PIN);
    huidu_value[1] = huidu_read_lit_state(XUNJI_L2_PORT, XUNJI_L2_PIN);
    huidu_value[2] = huidu_read_lit_state(XUNJI_L3_PORT, XUNJI_L3_PIN);
    huidu_value[3] = huidu_read_lit_state(XUNJI_L4_PORT, XUNJI_L4_PIN);
    huidu_value[4] = huidu_read_lit_state(XUNJI_R1_PORT, XUNJI_R1_PIN);
    huidu_value[5] = huidu_read_lit_state(XUNJI_R2_PORT, XUNJI_R2_PIN);
    huidu_value[6] = huidu_read_lit_state(XUNJI_R3_PORT, XUNJI_R3_PIN);
    huidu_value[7] = huidu_read_lit_state(XUNJI_R4_PORT, XUNJI_R4_PIN);
}

/**
 * 让不亮的探头尽量回到 L4/R1 之间的中心位置。
 *
 * 正常情况下，全亮表示黑带正好位于中心两路之间，应当直行。
 * 若全亮前处于第 3/4 档严重偏移，则保持原差速，直到再次出现不亮探头。
 * 若存在不亮探头，则使用其加权重心决定差速方向和强度。
 * 全灭时无法判断黑带中心，目标速度清零作为安全保护。
 */
void adjust_motor(void)
{
    int16_t weighted_error = 0;
    uint8_t unlit_count = 0U;
    uint8_t sensor_index;
    uint8_t offset_level;
    float outer_speed;

    huidu_get_value();

    /*
     * 保持旧版正转定义不变，只通过两路目标速度差进行转向，
     * 避免本次循迹逻辑重写改变 TB6612 的方向接线语义。
     */
    motor_set_direction(1U, 1U);
    motor_set_direction(2U, 1U);

    for (sensor_index = 0U;
         sensor_index < HUIDU_SENSOR_COUNT;
         sensor_index++) {
        if (huidu_value[sensor_index] == 0U) {
            weighted_error += s_black_position_weight[sensor_index];
            unlit_count++;
        }
    }

    /*
     * 严重偏移后突然全亮通常表示车身已经越过黑带。此时保持上一档
     * 转向，直到任意一路再次熄灭；其他全亮情况仍按居中直行处理。
     */
    if (unlit_count == 0U) {
        if (s_last_offset_level >= TRACK_LARGE_OFFSET_LEVEL) {
            huidu_apply_speed_ramp(
                s_last_command_speed_1, s_last_command_speed_2);
        } else {
            huidu_set_command(
                TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0U);
        }
        return;
    }

    /* 八路全灭无法得到有效横向位置，停车避免盲目冲出赛道。 */
    if (unlit_count == HUIDU_SENSOR_COUNT) {
        huidu_set_command(0.0f, 0.0f, 0U);
        return;
    }

    /*
     * 不亮探头相对中心对称时重心误差为零，说明黑带已经居中，
     * 两路使用相同目标速度直行。
     */
    if (weighted_error == 0) {
        huidu_set_command(
            TRACK_STRAIGHT_SPEED, TRACK_STRAIGHT_SPEED, 0U);
        return;
    }

    offset_level = huidu_select_offset_level(
        (uint16_t) ((weighted_error < 0) ? -weighted_error : weighted_error),
        unlit_count);
    outer_speed = huidu_get_outer_speed(offset_level);

    /*
     * 实车已确认：通道 1/A 是右轮，通道 2/B 是左轮。
     * 黑带偏左时左轮减速、右轮加速；黑带偏右时反向分配。
     */
    if (weighted_error < 0) {
        huidu_set_command(outer_speed, TRACK_INNER_SPEED, offset_level);
    } else {
        huidu_set_command(TRACK_INNER_SPEED, outer_speed, offset_level);
    }
}
