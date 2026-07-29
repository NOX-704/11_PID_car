# 12_Encoder_Line_Car

这是从 `11_PID_car` 完整复制并清除旧业务逻辑后的
MSPM0G3507 小车工程基线。

当前阶段只完成：

- 保留原工程的 CCS/Theia、SysConfig、时钟、启动、链接和 SDK 配置；
- 保留 PCB V1.0 的全部 GPIO、PWM、编码器、8 路循迹、按键、串口、
  I2C、K230 和舵机接线；
- 删除旧的阶梯差速、MPU6050 航向 PID 和旧控制定时器业务；
- `main.c` 只执行 `SYSCFG_DL_init()`，不启动电机或控制循环；
- 保留 TB6612FNG、编码器边沿计数、UART、I2C/MPU6050 和延时底层。

8 路加权误差、循迹外环 PD、双轮速度 PI、动态速度规划、丢线搜索和
小车状态机尚未生成，等待用户确认后进入下一阶段。

## 已确认的硬件基线

MCU 为 MSPM0G3507，外部 40 MHz 时钟经 PLL 生成 80 MHz CPUCLK。
电机 PWM 为 TIMG0，周期 4000；总控制定时器为 TIMA0，周期 5 ms。

| 外设功能 | 芯片/模块型号 | PCB 引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---:|---:|---|---|
| 右轮 PWM/PWMA | TB6612FNG | PA12 | PINCM34 | TIMG0_CCP0 | A 通道，0–4000 |
| 左轮 PWM/PWMB | TB6612FNG | PA13 | PINCM35 | TIMG0_CCP1 | B 通道，0–4000 |
| 右轮 AIN1 | TB6612FNG | PA9 | PINCM20 | GPIO | 前进时低 |
| 右轮 AIN2 | TB6612FNG | PA8 | PINCM19 | GPIO | 前进时高 |
| 左轮 BIN1 | TB6612FNG | PA7 | PINCM14 | GPIO | 前进时低 |
| 左轮 BIN2 | TB6612FNG | PB18 | PINCM44 | GPIO | 前进时高 |
| STBY | TB6612FNG | PB24 | PINCM52 | GPIO | 高电平使能 |
| 右编码器 A | MG310 | PB8 | PINCM25 | GPIO 上升沿中断 | 通道 1 |
| 右编码器 B | MG310 | PB9 | PINCM26 | GPIO 输入 | 保留方向相 |
| 左编码器 A | MG310 | PB19 | PINCM45 | GPIO 上升沿中断 | 通道 2 |
| 左编码器 B | MG310 | PB20 | PINCM48 | GPIO 输入 | 保留方向相 |
| S0/L1 | 亚博智能 8 路循迹 | PA18 | PINCM40 | GPIO 输入 | 最左 |
| S1/L2 | 亚博智能 8 路循迹 | PA16 | PINCM38 | GPIO 输入 | |
| S2/L3 | 亚博智能 8 路循迹 | PB7 | PINCM24 | GPIO 输入 | |
| S3/L4 | 亚博智能 8 路循迹 | PA17 | PINCM39 | GPIO 输入 | |
| S4/R1 | 亚博智能 8 路循迹 | PA21 | PINCM46 | GPIO 输入 | |
| S5/R2 | 亚博智能 8 路循迹 | PA22 | PINCM47 | GPIO 输入 | |
| S6/R3 | 亚博智能 8 路循迹 | PA24 | PINCM54 | GPIO 输入 | |
| S7/R4 | 亚博智能 8 路循迹 | PA2 | PINCM7 | GPIO 输入 | 最右，沿用 PCB |
| 启停键 | PCB SW1 | PB6 | PINCM23 | GPIO 下拉输入 | 按下为高 |
| 调试 TX/RX | 3.3 V USB-UART | PA28/PA31 | PINCM3/PINCM6 | UART0 | 115200 |
| 扩展 SDA/SCL | I2C 扩展口 | PB3/PB2 | PINCM16/PINCM15 | I2C1 | 400 kHz |
| K230 TX/RX | 亚博智能 K230 | PA26/PA25 | PINCM59/PINCM55 | UART3 | 9600 |
| 舵机 PWM | 舵机接口 | PA27 | PINCM60 | TIMG7_CCP1 | 原配置保留 |

从小车前进方向看，8 路实际顺序固定为：

```text
S0    S1    S2    S3    S4    S5    S6    S7
L1    L2    L3    L4    R1    R2    R3    R4
PA18  PA16  PB7   PA17  PA21  PA22  PA24  PA2
```

亚博智能模块的原始数字输出为：

- 黑色胶带：低电平；
- 白色背景：高电平。

下一阶段的读取层应统一转换为“黑=`1`、白=`0`”，控制算法不得在多处
重复反相。

## 基线验证

在 macOS 当前工具链下执行：

```bash
make check
```

验证顺序为：

1. SysConfig CLI 重新生成 `ti_msp_dl_config.c/h`；
2. 检查所有关键宏和 PCB 引脚是否与本表一致；
3. 用 TI ArmClang 逐文件对象编译 `main.c`、全部底层 `.c` 和生成配置。

