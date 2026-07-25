#include "huidu.h"

uint8_t huidu_value[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void huidu_get_value()
{
    uint8_t cmd = 0x30;
    uint8_t data = 0;
    uint32_t timeout;

    DL_I2C_fillControllerTXFIFO(HUIDU_INST, &cmd, 1);
    DL_I2C_startControllerTransfer(HUIDU_INST, HUIDU8_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    timeout = 100000;
    while ((DL_I2C_getControllerStatus(HUIDU_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}

    DL_I2C_startControllerTransfer(HUIDU_INST, HUIDU8_I2C_ADDR, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    timeout = 100000;
    while ((DL_I2C_getControllerStatus(HUIDU_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) && --timeout) {}
    data = DL_I2C_receiveControllerData(HUIDU_INST);
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
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    else if (v[0] == 1 && v[1] == 1 && v[2] == 1 && v[3] == 1
        && v[4] == 1 && v[5] == 1 && v[6] == 1 && v[7] == 1)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
    }
    else if ((v[3] == 1 || v[4] == 1)
        && v[0] == 0 && v[1] == 0 && v[2] == 0
        && v[5] == 0 && v[6] == 0 && v[7] == 0)
    {
        motor_set_direction(1, 1);
        motor_set_direction(2, 1);
        float min_speed = target_speed_1 < target_speed_2 ? target_speed_1 : target_speed_2;
        target_speed_1 = min_speed;
        target_speed_2 = min_speed;
    }
    // 左侧传感器: S3(近中心) > S2 > S1 > S0(最左)，从近到远检查
    else if (v[3] == 1 && v[0] == 0 && v[1] == 0 && v[2] == 0)
    {
        target_speed_1 = 275;
        target_speed_2 = 400;
    }
    else if (v[2] == 1 && v[0] == 0 && v[1] == 0)
    {
        target_speed_1 = 250;
        target_speed_2 = 400;
    }
    else if (v[1] == 1 && v[0] == 0)
    {
        target_speed_1 = 225;
        target_speed_2 = 400;
    }
    else if (v[0] == 1)
    {
        target_speed_1 = 200;
        target_speed_2 = 400;
    }
    // 右侧传感器: S4(近中心) > S5 > S6 > S7(最右)，从近到远检查
    else if (v[4] == 1 && v[5] == 0 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = 400;
        target_speed_2 = 275;
    }
    else if (v[5] == 1 && v[6] == 0 && v[7] == 0)
    {
        target_speed_1 = 400;
        target_speed_2 = 225;
    }
    else if (v[6] == 1 && v[7] == 0)
    {
        target_speed_1 = 400;
        target_speed_2 = 200;
    }
    else if (v[7] == 1)
    {
        target_speed_1 = 400;
        target_speed_2 = 200;
    }
}
