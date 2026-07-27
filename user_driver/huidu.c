#include "huidu.h"

#include "motor.h"
#include "ti_msp_dl_config.h"


#define TRACK_STRAIGHT_SPEED (280.0f)
#define TRACK_INNER_SPEED    (140.0f)
#define TRACK_OUTER_SPEED_1  (160.0f)
#define TRACK_OUTER_SPEED_2  (280.0f)
#define TRACK_OUTER_SPEED_3  (300.0f)
#define TRACK_OUTER_SPEED_4  (320.0f)

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
 * 中断中执行除法。平均偏移落在第 1~4 档时，分别返回 250~400。
 */
static float huidu_select_outer_speed(
    uint16_t error_magnitude, uint8_t unlit_count)
{
    if (error_magnitude <= (uint16_t) unlit_count) {
        return TRACK_OUTER_SPEED_1;
    }
    if (error_magnitude <= (uint16_t) (2U * unlit_count)) {
        return TRACK_OUTER_SPEED_2;
    }
    if (error_magnitude <= (uint16_t) (3U * unlit_count)) {
        return TRACK_OUTER_SPEED_3;
    }
    return TRACK_OUTER_SPEED_4;
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
 * 全亮表示黑带正好位于中心两路之间，按用户实测要求直行。
 * 若存在不亮探头，则使用其加权重心决定差速方向和强度。
 * 全灭时无法判断黑带中心，目标速度清零作为安全保护。
 */
void adjust_motor(void)
{
    int16_t weighted_error = 0;
    uint8_t unlit_count = 0U;
    uint8_t sensor_index;
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
     * 黑带位于 L4/R1 的间隙时八路都照到白色并全部亮灯，
     * 这是本车的正常直行状态，不应再按“丢线”停车。
     */
    if (unlit_count == 0U) {
        target_speed_1 = TRACK_STRAIGHT_SPEED;
        target_speed_2 = TRACK_STRAIGHT_SPEED;
        return;
    }

    /* 八路全灭无法得到有效横向位置，停车避免盲目冲出赛道。 */
    if (unlit_count == HUIDU_SENSOR_COUNT) {
        target_speed_1 = 0.0f;
        target_speed_2 = 0.0f;
        return;
    }

    /*
     * 不亮探头相对中心对称时重心误差为零，说明黑带已经居中，
     * 两路使用相同目标速度直行。
     */
    if (weighted_error == 0) {
        target_speed_1 = TRACK_STRAIGHT_SPEED;
        target_speed_2 = TRACK_STRAIGHT_SPEED;
        return;
    }

    outer_speed = huidu_select_outer_speed(
        (uint16_t) ((weighted_error < 0) ? -weighted_error : weighted_error),
        unlit_count);

    /*
     * 延续旧工程的电机通道差速方向：
     * 黑带偏左时通道 1 慢、通道 2 快；偏右时反向分配。
     */
    if (weighted_error < 0) {
        target_speed_1 = outer_speed;
        target_speed_2 = TRACK_INNER_SPEED;
    } else {
        target_speed_1 = TRACK_INNER_SPEED;
        target_speed_2 = outer_speed;
    }
}
