#include "huidu.h"

uint8_t huidu_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void huidu_get_value()
{
    uint8_t data = 0;

    if (DL_GPIO_readPins(XUNJI_L1_PORT, XUNJI_L1_PIN) & XUNJI_L1_PIN) data |= (1 << 0);
    if (DL_GPIO_readPins(XUNJI_L2_PORT, XUNJI_L2_PIN) & XUNJI_L2_PIN) data |= (1 << 1);
    if (DL_GPIO_readPins(XUNJI_L3_PORT, XUNJI_L3_PIN) & XUNJI_L3_PIN) data |= (1 << 2);
    if (DL_GPIO_readPins(XUNJI_L4_PORT, XUNJI_L4_PIN) & XUNJI_L4_PIN) data |= (1 << 3);
    if (DL_GPIO_readPins(XUNJI_R1_PORT, XUNJI_R1_PIN) & XUNJI_R1_PIN) data |= (1 << 4);
    if (DL_GPIO_readPins(XUNJI_R2_PORT, XUNJI_R2_PIN) & XUNJI_R2_PIN) data |= (1 << 5);
    if (DL_GPIO_readPins(XUNJI_R3_PORT, XUNJI_R3_PIN) & XUNJI_R3_PIN) data |= (1 << 6);
    if (DL_GPIO_readPins(XUNJI_R4_PORT, XUNJI_R4_PIN) & XUNJI_R4_PIN) data |= (1 << 7);

    data = ~data;

    huidu_value[0] = (data >> 0) & 0x01;
    huidu_value[1] = (data >> 1) & 0x01;
    huidu_value[2] = (data >> 2) & 0x01;
    huidu_value[3] = (data >> 3) & 0x01;
    huidu_value[4] = (data >> 4) & 0x01;
    huidu_value[5] = (data >> 5) & 0x01;
    huidu_value[6] = (data >> 6) & 0x01;
    huidu_value[7] = (data >> 7) & 0x01;
}

extern float target_speed_1;
extern float target_speed_2;

void adjust_motor()
{
    huidu_get_value();
    uint8_t *v = huidu_value;

    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    if (v[0] == 0 && v[1] == 0 && v[2] == 0 && v[3] == 0
        && v[4] == 0 && v[5] == 0 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    else if (v[0] == 1 && v[1] == 1 && v[2] == 1 && v[3] == 1
        && v[4] == 1 && v[5] == 1 && v[6] == 1 && v[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    else if (v[3] == 1 && v[4] == 1
        && v[0] == 0 && v[1] == 0 && v[2] == 0
        && v[5] == 0 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = MIN_SPEED;
        target_speed_2 = MIN_SPEED;
    }
    else if (v[3] == 1 && v[0] == 0 && v[1] == 0 && v[2] == 0)
    {
        target_speed_1 = MIN_SPEED;
        target_speed_2 = 250;
    }
    else if (v[2] == 1 && v[0] == 0 && v[1] == 0)
    {
        target_speed_1 = MIN_SPEED;
        target_speed_2 = 300;
    }
    else if (v[1] == 1 && v[0] == 0)
    {
        target_speed_1 = MIN_SPEED;
        target_speed_2 = 350;
    }
    else if (v[0] == 1)
    {
        target_speed_1 = MIN_SPEED;
        target_speed_2 = 400;
    }
    else if (v[4] == 1 && v[5] == 0 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = 250;
        target_speed_2 = MIN_SPEED;
    }
    else if (v[5] == 1 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = 300;
        target_speed_2 = MIN_SPEED;
    }
    else if (v[6] == 1 && v[7] == 0)
    {
        target_speed_1 = 350;
        target_speed_2 = MIN_SPEED;
    }
    else if (v[7] == 1)
    {
        target_speed_1 = 400;
        target_speed_2 = MIN_SPEED;
    }
}
