/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for SERVO */
#define SERVO_INST                                                         TIMG7
#define SERVO_INST_IRQHandler                                   TIMG7_IRQHandler
#define SERVO_INST_INT_IRQN                                     (TIMG7_INT_IRQn)
#define SERVO_INST_CLK_FREQ                                               100000
/* GPIO defines for channel 1 */
#define GPIO_SERVO_C1_PORT                                                 GPIOA
#define GPIO_SERVO_C1_PIN                                         DL_GPIO_PIN_27
#define GPIO_SERVO_C1_IOMUX                                      (IOMUX_PINCM60)
#define GPIO_SERVO_C1_IOMUX_FUNC                     IOMUX_PINCM60_PF_TIMG7_CCP1
#define GPIO_SERVO_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWMAB */
#define PWMAB_INST                                                         TIMG0
#define PWMAB_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWMAB_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWMAB_INST_CLK_FREQ                                             40000000
/* GPIO defines for channel 0 */
#define GPIO_PWMAB_C0_PORT                                                 GPIOA
#define GPIO_PWMAB_C0_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWMAB_C0_IOMUX                                      (IOMUX_PINCM34)
#define GPIO_PWMAB_C0_IOMUX_FUNC                     IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWMAB_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWMAB_C1_PORT                                                 GPIOA
#define GPIO_PWMAB_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWMAB_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWMAB_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWMAB_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for CONTROL_LOOP */
#define CONTROL_LOOP_INST                                                (TIMA0)
#define CONTROL_LOOP_INST_IRQHandler                            TIMA0_IRQHandler
#define CONTROL_LOOP_INST_INT_IRQN                              (TIMA0_INT_IRQn)
#define CONTROL_LOOP_INST_LOAD_VALUE                                     (3999U)




/* Defines for EXPANSION_I2C */
#define EXPANSION_I2C_INST                                                  I2C1
#define EXPANSION_I2C_INST_IRQHandler                            I2C1_IRQHandler
#define EXPANSION_I2C_INST_INT_IRQN                                I2C1_INT_IRQn
#define EXPANSION_I2C_BUS_SPEED_HZ                                        400000
#define GPIO_EXPANSION_I2C_SDA_PORT                                        GPIOB
#define GPIO_EXPANSION_I2C_SDA_PIN                                 DL_GPIO_PIN_3
#define GPIO_EXPANSION_I2C_IOMUX_SDA                             (IOMUX_PINCM16)
#define GPIO_EXPANSION_I2C_IOMUX_SDA_FUNC               IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_EXPANSION_I2C_SCL_PORT                                        GPIOB
#define GPIO_EXPANSION_I2C_SCL_PIN                                 DL_GPIO_PIN_2
#define GPIO_EXPANSION_I2C_IOMUX_SCL                             (IOMUX_PINCM15)
#define GPIO_EXPANSION_I2C_IOMUX_SCL_FUNC               IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for DEBUG */
#define DEBUG_INST                                                         UART0
#define DEBUG_INST_FREQUENCY                                            40000000
#define DEBUG_INST_IRQHandler                                   UART0_IRQHandler
#define DEBUG_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_DEBUG_RX_PORT                                                 GPIOA
#define GPIO_DEBUG_TX_PORT                                                 GPIOA
#define GPIO_DEBUG_RX_PIN                                         DL_GPIO_PIN_31
#define GPIO_DEBUG_TX_PIN                                         DL_GPIO_PIN_28
#define GPIO_DEBUG_IOMUX_RX                                       (IOMUX_PINCM6)
#define GPIO_DEBUG_IOMUX_TX                                       (IOMUX_PINCM3)
#define GPIO_DEBUG_IOMUX_RX_FUNC                        IOMUX_PINCM6_PF_UART0_RX
#define GPIO_DEBUG_IOMUX_TX_FUNC                        IOMUX_PINCM3_PF_UART0_TX
#define DEBUG_BAUD_RATE                                                 (115200)
#define DEBUG_IBRD_40_MHZ_115200_BAUD                                       (21)
#define DEBUG_FBRD_40_MHZ_115200_BAUD                                       (45)
/* Defines for K230_LINK */
#define K230_LINK_INST                                                     UART3
#define K230_LINK_INST_FREQUENCY                                        80000000
#define K230_LINK_INST_IRQHandler                               UART3_IRQHandler
#define K230_LINK_INST_INT_IRQN                                   UART3_INT_IRQn
#define GPIO_K230_LINK_RX_PORT                                             GPIOA
#define GPIO_K230_LINK_TX_PORT                                             GPIOA
#define GPIO_K230_LINK_RX_PIN                                     DL_GPIO_PIN_25
#define GPIO_K230_LINK_TX_PIN                                     DL_GPIO_PIN_26
#define GPIO_K230_LINK_IOMUX_RX                                  (IOMUX_PINCM55)
#define GPIO_K230_LINK_IOMUX_TX                                  (IOMUX_PINCM59)
#define GPIO_K230_LINK_IOMUX_RX_FUNC                   IOMUX_PINCM55_PF_UART3_RX
#define GPIO_K230_LINK_IOMUX_TX_FUNC                   IOMUX_PINCM59_PF_UART3_TX
#define K230_LINK_BAUD_RATE                                               (9600)
#define K230_LINK_IBRD_80_MHZ_9600_BAUD                                    (520)
#define K230_LINK_FBRD_80_MHZ_9600_BAUD                                     (53)





/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for LED0: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LED_LED0_PIN                                            (DL_GPIO_PIN_14)
#define LED_LED0_IOMUX                                           (IOMUX_PINCM36)
/* Defines for LED1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define LED_LED1_PIN                                            (DL_GPIO_PIN_15)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM37)
/* Defines for AIN2: GPIOA.8 with pinCMx 19 on package pin 54 */
#define DC_MOTOR_AIN2_PORT                                               (GPIOA)
#define DC_MOTOR_AIN2_PIN                                        (DL_GPIO_PIN_8)
#define DC_MOTOR_AIN2_IOMUX                                      (IOMUX_PINCM19)
/* Defines for AIN1: GPIOA.9 with pinCMx 20 on package pin 55 */
#define DC_MOTOR_AIN1_PORT                                               (GPIOA)
#define DC_MOTOR_AIN1_PIN                                        (DL_GPIO_PIN_9)
#define DC_MOTOR_AIN1_IOMUX                                      (IOMUX_PINCM20)
/* Defines for STBY: GPIOB.24 with pinCMx 52 on package pin 23 */
#define DC_MOTOR_STBY_PORT                                               (GPIOB)
#define DC_MOTOR_STBY_PIN                                       (DL_GPIO_PIN_24)
#define DC_MOTOR_STBY_IOMUX                                      (IOMUX_PINCM52)
/* Defines for AA: GPIOB.8 with pinCMx 25 on package pin 60 */
#define DC_MOTOR_AA_PORT                                                 (GPIOB)
// pins affected by this interrupt request:["AA","BA"]
#define DC_MOTOR_INT_IRQN                                       (GPIOB_INT_IRQn)
#define DC_MOTOR_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define DC_MOTOR_AA_IIDX                                     (DL_GPIO_IIDX_DIO8)
#define DC_MOTOR_AA_PIN                                          (DL_GPIO_PIN_8)
#define DC_MOTOR_AA_IOMUX                                        (IOMUX_PINCM25)
/* Defines for AB: GPIOB.9 with pinCMx 26 on package pin 61 */
#define DC_MOTOR_AB_PORT                                                 (GPIOB)
#define DC_MOTOR_AB_PIN                                          (DL_GPIO_PIN_9)
#define DC_MOTOR_AB_IOMUX                                        (IOMUX_PINCM26)
/* Defines for BIN2: GPIOB.18 with pinCMx 44 on package pin 15 */
#define DC_MOTOR_BIN2_PORT                                               (GPIOB)
#define DC_MOTOR_BIN2_PIN                                       (DL_GPIO_PIN_18)
#define DC_MOTOR_BIN2_IOMUX                                      (IOMUX_PINCM44)
/* Defines for BIN1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define DC_MOTOR_BIN1_PORT                                               (GPIOA)
#define DC_MOTOR_BIN1_PIN                                        (DL_GPIO_PIN_7)
#define DC_MOTOR_BIN1_IOMUX                                      (IOMUX_PINCM14)
/* Defines for BA: GPIOB.19 with pinCMx 45 on package pin 16 */
#define DC_MOTOR_BA_PORT                                                 (GPIOB)
#define DC_MOTOR_BA_IIDX                                    (DL_GPIO_IIDX_DIO19)
#define DC_MOTOR_BA_PIN                                         (DL_GPIO_PIN_19)
#define DC_MOTOR_BA_IOMUX                                        (IOMUX_PINCM45)
/* Defines for BB: GPIOB.20 with pinCMx 48 on package pin 19 */
#define DC_MOTOR_BB_PORT                                                 (GPIOB)
#define DC_MOTOR_BB_PIN                                         (DL_GPIO_PIN_20)
#define DC_MOTOR_BB_IOMUX                                        (IOMUX_PINCM48)
/* Defines for L1: GPIOA.18 with pinCMx 40 on package pin 11 */
#define XUNJI_L1_PORT                                                    (GPIOA)
#define XUNJI_L1_PIN                                            (DL_GPIO_PIN_18)
#define XUNJI_L1_IOMUX                                           (IOMUX_PINCM40)
/* Defines for L2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define XUNJI_L2_PORT                                                    (GPIOA)
#define XUNJI_L2_PIN                                            (DL_GPIO_PIN_16)
#define XUNJI_L2_IOMUX                                           (IOMUX_PINCM38)
/* Defines for L3: GPIOB.7 with pinCMx 24 on package pin 59 */
#define XUNJI_L3_PORT                                                    (GPIOB)
#define XUNJI_L3_PIN                                             (DL_GPIO_PIN_7)
#define XUNJI_L3_IOMUX                                           (IOMUX_PINCM24)
/* Defines for L4: GPIOA.17 with pinCMx 39 on package pin 10 */
#define XUNJI_L4_PORT                                                    (GPIOA)
#define XUNJI_L4_PIN                                            (DL_GPIO_PIN_17)
#define XUNJI_L4_IOMUX                                           (IOMUX_PINCM39)
/* Defines for R1: GPIOA.21 with pinCMx 46 on package pin 17 */
#define XUNJI_R1_PORT                                                    (GPIOA)
#define XUNJI_R1_PIN                                            (DL_GPIO_PIN_21)
#define XUNJI_R1_IOMUX                                           (IOMUX_PINCM46)
/* Defines for R2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define XUNJI_R2_PORT                                                    (GPIOA)
#define XUNJI_R2_PIN                                            (DL_GPIO_PIN_22)
#define XUNJI_R2_IOMUX                                           (IOMUX_PINCM47)
/* Defines for R3: GPIOA.24 with pinCMx 54 on package pin 25 */
#define XUNJI_R3_PORT                                                    (GPIOA)
#define XUNJI_R3_PIN                                            (DL_GPIO_PIN_24)
#define XUNJI_R3_IOMUX                                           (IOMUX_PINCM54)
/* Defines for R4: GPIOA.2 with pinCMx 7 on package pin 42 */
#define XUNJI_R4_PORT                                                    (GPIOA)
#define XUNJI_R4_PIN                                             (DL_GPIO_PIN_2)
#define XUNJI_R4_IOMUX                                            (IOMUX_PINCM7)
/* Port definition for Pin Group START_KEY */
#define START_KEY_PORT                                                   (GPIOB)

/* Defines for KEY: GPIOB.6 with pinCMx 23 on package pin 58 */
#define START_KEY_KEY_PIN                                        (DL_GPIO_PIN_6)
#define START_KEY_KEY_IOMUX                                      (IOMUX_PINCM23)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_SERVO_init(void);
void SYSCFG_DL_PWMAB_init(void);
void SYSCFG_DL_CONTROL_LOOP_init(void);
void SYSCFG_DL_EXPANSION_I2C_init(void);
void SYSCFG_DL_DEBUG_init(void);
void SYSCFG_DL_K230_LINK_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
