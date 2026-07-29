#include "motor.h"
#include "huidu.h"

/*
 * 电机 PWM 周期为 4000。灰度循迹中断周期为 5 ms，但编码器测速、
 * 速度 PI 和 PWM 斜坡仍每两个节拍运行一次，即保持原来的 10 ms。
 * 每次速度环最多改变 80；从 0 到 4000 约需 0.5 s。
 */
#define MOTOR_PWM_MAX_DUTY  (4000)
#define MOTOR_PWM_RAMP_STEP (80)

/*
 * 双轮编码器只负责让实际轮速跟随航向控制给出的左右目标。原来的
 * 通用增量式 PID 已删除；编码器计数的离散噪声容易被 D 项放大，
 * 因此新的底层速度环明确使用增量式 PI。
 */
#define MOTOR_SPEED_PI_KP   (0.5f)
#define MOTOR_SPEED_PI_KI   (0.4f)

/* 由 10 ms 中断置位，主循环取走后执行一次 MPU6050 I2C 采样。 */
static volatile uint8_t s_imu_sample_request = 0U;

/*
 * 5 ms 中断的二分频状态。灰度传感器和循迹外环每次都运行，编码器、
 * 速度 PI 与 MPU6050 请求只在每两个节拍中的第二个节拍运行。
 */
static uint8_t s_speed_loop_divider = 0U;

void motor_init(uint8_t motor_id)
{
    if(motor_id == 1 || motor_id == 3){
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, 0U, GPIO_PWMAB_C0_IDX);
    }
    if(motor_id == 2 || motor_id == 3){
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, 0U, GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(PWMAB_INST);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    DL_Timer_startCounter(CONTROL_LOOP_INST);
    NVIC_EnableIRQ(CONTROL_LOOP_INST_INT_IRQN);
}

// 限幅函数
int limit_duty(int duty)
{
    if(duty > MOTOR_PWM_MAX_DUTY){
        duty = MOTOR_PWM_MAX_DUTY;
    }
    if(duty < 0)
    {
        duty = 0;
    }
    return duty;
}

/**
 * 限制一次 10 ms 控制周期内的实际 PWM 变化量。
 *
 * 循迹外环更新目标轮速差，最终写入 TB6612 的 PWM 每次仍最多
 * 变化 80，避免档位切换和航向 PID 修正形成 PWM 硬阶跃。
 */
static int motor_ramp_duty(int current_duty, int requested_duty)
{
    if (requested_duty > current_duty + MOTOR_PWM_RAMP_STEP) {
        return current_duty + MOTOR_PWM_RAMP_STEP;
    }
    if (requested_duty < current_duty - MOTOR_PWM_RAMP_STEP) {
        return current_duty - MOTOR_PWM_RAMP_STEP;
    }
    return requested_duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    int limited_duty = limit_duty((int) duty);

    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, (uint32_t) limited_duty, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(
            PWMAB_INST, (uint32_t) limited_duty, GPIO_PWMAB_C1_IDX);
    }
}

/*
 * direction: 0=停止，1=车辆前进方向，2=车辆后退方向。
 * 电机 2 在实车上的安装极性与旧代码相反，因此其正转电平在此处对调。
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}


extern uint32_t counter_1_A;
float speed_1 = 0;

extern uint32_t counter_2_A;
float speed_2 = 0;

void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100; // 轮速 mm/s
        counter_1_A = 0; // 计算完速度后清零计数器
    }
    if (motor_id == 2) {
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 100; // 轮速 mm/s
        counter_2_A = 0; // 计算完速度后清零计数器
    }
}

int PWM_1_duty = 0;
float target_speed_1 = 0; // 目标速度 mm/s
static float s_last_speed_error_1 = 0.0f;

int PWM_2_duty = 0;
float target_speed_2 = 0; // 目标速度 mm/s
static float s_last_speed_error_2 = 0.0f;

/**
 * 新的双轮增量式速度 PI。
 *
 * MPU6050 航向 PID 产生左右目标轮速；本函数只消除两个电机和负载的
 * 个体差异。去掉 D 项可以避免 10 ms 编码器计数跳变放大为 PWM 抖动。
 */
static void motor_speed_pi_update(uint8_t motor_id)
{
    float error;
    int pid_delta;
    int requested_duty;

    if (motor_id == 1) {
        error = target_speed_1 - speed_1;
        pid_delta = (int)(
            MOTOR_SPEED_PI_KP * (error - s_last_speed_error_1) +
            MOTOR_SPEED_PI_KI * error);
        s_last_speed_error_1 = error;
        requested_duty = limit_duty(PWM_1_duty + pid_delta);
        PWM_1_duty = motor_ramp_duty(PWM_1_duty, requested_duty);
        motor_set_duty(motor_id, PWM_1_duty);
    }
    if (motor_id == 2) {
        error = target_speed_2 - speed_2;
        pid_delta = (int)(
            MOTOR_SPEED_PI_KP * (error - s_last_speed_error_2) +
            MOTOR_SPEED_PI_KI * error);
        s_last_speed_error_2 = error;
        requested_duty = limit_duty(PWM_2_duty + pid_delta);
        PWM_2_duty = motor_ramp_duty(PWM_2_duty, requested_duty);
        motor_set_duty(motor_id, PWM_2_duty);
    }
}

bool motor_take_imu_sample_request(void)
{
    if (s_imu_sample_request == 0U) {
        return false;
    }

    s_imu_sample_request = 0U;
    return true;
}

void CONTROL_LOOP_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(CONTROL_LOOP_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        /*
         * 灰度 GPIO 和循迹外环以 200 Hz 更新，让黑线位置变化最多
         * 经过 5 ms 就能反映到左右目标轮速。
         */
        adjust_motor();

        s_speed_loop_divider++;
        if (s_speed_loop_divider >= 2U) {
            s_speed_loop_divider = 0U;

            /*
             * 编码器和速度 PI 保持 100 Hz，避免 5 ms 内脉冲太少造成
             * 轮速量化跳动。阻塞式 MPU6050 I2C 仍交给主循环执行。
             */
            s_imu_sample_request = 1U;
            calculate_speed(1);
            motor_speed_pi_update(1);
            calculate_speed(2);
            motor_speed_pi_update(2);
        }
        break;

    default:
        break;
    }
}
