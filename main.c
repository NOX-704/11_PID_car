/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "uart.h"
#include "motor.h"
#include "huidu.h"
#include "heading_pid.h"
#include "mpu6050.h"

extern float target_speed_1;
extern float target_speed_2;

int main(void)
{
    bool mpu_ready;
    uint8_t debug_tick_count = 0U;
    char debug_buffer[96];

    SYSCFG_DL_init();
    // NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
    // NVIC_EnableIRQ(KEY_INT_IRQN);
    /* 启用 DC_MOTOR GPIOB 中断，用于统计两路编码器 A 相上升沿。 */
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST, 50, GPIO_SERVO_C1_IDX);
    target_speed_1 = 0;
    target_speed_2 = 0;

    /*
     * 电机控制定时器尚未启动，车辆保持静止。MPU6050 在这里通过
     * PCB 的 PB2/PB3 I2C1 初始化，并用约 2 秒完成 Z 轴零偏校准。
     */
    UART_send_string(DEBUG_INST, "MPU6050 calibrating, keep car still...\n");
    mpu_ready = mpu6050_init();
    heading_pid_init();

    if (mpu_ready) {
        UART_send_string(DEBUG_INST, "MPU6050 ready\n");
    } else {
        sprintf(
            debug_buffer,
            "MPU6050 error=%d, staircase fallback\n",
            mpu6050_get_last_error());
        UART_send_string(DEBUG_INST, debug_buffer);
    }

    /* 校准完成后再启动电机 PWM、编码器速度 PI 和 10 ms 控制节拍。 */
    motor_init(3);

    while (1) {
        /*
         * 10 ms 中断只提出采样请求；阻塞式 I2C 读取放在主循环，
         * 避免 MPU6050 或接线异常时阻塞电机中断。
         */
        if (motor_take_imu_sample_request()) {
            int yaw_x10;
            int gyro_z_x10;
            int correction_x10;

            if (mpu_ready && !mpu6050_update_10ms()) {
                mpu_ready = false;
            }

            debug_tick_count++;
            if (debug_tick_count >= 50U) {
                debug_tick_count = 0U;
                yaw_x10 = (int) (mpu6050_get_yaw_deg() * 10.0f);
                gyro_z_x10 =
                    (int) (mpu6050_get_gyro_z_dps() * 10.0f);
                correction_x10 =
                    (int) (heading_pid_get_correction() * 10.0f);

                /*
                 * 每 500 ms 输出八路黑线逻辑、MPU 状态、航向角、角速度、
                 * 控制模式和修正量。全部按 0.1 单位转成整数，避免依赖
                 * printf 浮点格式支持。
                 */
                sprintf(
                    debug_buffer,
                    "%d%d%d%d%d%d%d%d IMU=%d Y=%d GZ=%d M=%d C=%d\n",
                    huidu_value[0], huidu_value[1],
                    huidu_value[2], huidu_value[3],
                    huidu_value[4], huidu_value[5],
                    huidu_value[6], huidu_value[7],
                    mpu_ready ? 1 : 0,
                    yaw_x10,
                    gyro_z_x10,
                    heading_pid_get_mode(),
                    correction_x10);
                UART_send_string(DEBUG_INST, debug_buffer);
            }
        }
    }
}
