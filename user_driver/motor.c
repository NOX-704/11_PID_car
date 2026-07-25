#include "motor.h"

void motor_init(uint8_t motor_id)
{
    if(motor_id == 1 || motor_id == 3){
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C0_IDX);
    }
    if(motor_id == 2 || motor_id == 3){
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWMAB_INST, 0, GPIO_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(PWMAB_INST);
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

// 限幅函数
int limit_duty(int duty)
{
    if(duty > 4000){
        duty = 4000;
    }
    if(duty < 0)
    {
        duty = 0;
    }
    return duty;
}

void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(PWMAB_INST, duty, GPIO_PWMAB_C1_IDX);
    }
}

// direction: 0 停止，1 正转，2 反转
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
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

float kp = 0.5; // 比例系数
float ki = 0.4; // 积分系数
float kd = 0.1; // 微分系数

int PWM_1_duty = 0;
float target_speed_1 = 0; // 目标速度 mm/s
float last_error_1 = 0;
float prev_error_1 = 0;

int PWM_2_duty = 0;
float target_speed_2 = 0; // 目标速度 mm/s
float last_error_2 = 0;
float prev_error_2 = 0;

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == 1) {
        error = target_speed_1 - speed_1;
        PWM_1_duty += (int)(kp * (error - last_error_1)
                         + ki * error
                         + kd * (error - 2 * last_error_1 + prev_error_1));
        prev_error_1 = last_error_1;
        last_error_1 = error;
        PWM_1_duty = limit_duty(PWM_1_duty);
        motor_set_duty(motor_id, PWM_1_duty);
    }
    if (motor_id == 2) {
        error = target_speed_2 - speed_2;
        PWM_2_duty += (int)(kp * (error - last_error_2)
                         + ki * error
                         + kd * (error - 2 * last_error_2 + prev_error_2));
        prev_error_2 = last_error_2;
        last_error_2 = error;
        PWM_2_duty = limit_duty(PWM_2_duty);
        motor_set_duty(motor_id, PWM_2_duty);
    }
}

void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        adjust_motor();
        calculate_speed(1);
        DC_MOTOR_PID(1);
        calculate_speed(2);
        DC_MOTOR_PID(2);
        break;
    // case DL_TIMER_IIDX_COMPARE_0:
    //     status = (status + 3 -1) % 3;
    //     /* code */
    //     break;
    
    default:
        break;
    }
}



