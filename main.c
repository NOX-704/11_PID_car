/*
 * main.c - 12_Encoder_Line_Car 空业务基线
 *
 * 当前阶段只验证复制后的 MSPM0G3507 工程、SysConfig 和底层驱动。
 * 循迹、速度闭环、差速控制和状态机将在用户确认后再生成。
 */

#include "ti_msp_dl_config.h"

int main(void)
{
    /*
     * 保留 11_PID_car 的时钟、GPIO、PWM、定时器、I2C、UART 和全部
     * 引脚配置。这里不启动电机和控制定时器，确保空业务基线安全静止。
     */
    SYSCFG_DL_init();

    while (1) {
        __WFI();
    }
}
