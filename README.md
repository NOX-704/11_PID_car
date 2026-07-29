# 11_PID_car

基于 TI MSPM0G3507 的双直流电机循迹小车工程。当前版本使用八路独立 GPIO 读取数字循迹信号，对不亮探头计算黑带横向重心，由转向 PID 连续生成两路目标轮速差，再利用编码器反馈和增量式 PID 调节 TB6612FNG 的 A/B 两个电机通道。

> 外环是八路黑线重心转向 PID，内环是两路编码器速度 PID；没有重新引入 MPU6050。

## PCB V1.0 基准

本工程以 2026-07-28 更新的“电赛小车底板 V1.0”原理图为唯一板级引脚基准。关键接口如下：

| PCB 接口 | 用途 | MSPM0G3507 引脚 |
|---|---|---|
| H1/H6/H7 | OLED 等外设共享 I²C1 | PB2=SCL，PB3=SDA |
| H2-3～H2-10 | 八路数字循迹 L1、L2、L3、L4、R1、R2、R3、R4 | PA18、PA16、PB7、PA17、PA21、PA22、PA24、PA2 |
| H3 | 舵机 PWM / 5V / GND | PA27 |
| H4 | 调试串口 TX / RX / GND | PA28、PA31 |
| H5 | K230 串口 TX / RX / GND | PA26、PA25 |
| SW1 | 启动按键，高电平按下 | PB6 |

原理图的 TB6612FNG `AIN1`、`AIN2` 网络旁都标成了 `PA8`，同一 GPIO 无法控制两路方向信号。结合已经按该底板验证过的 `k230control` 工程，本工程固定使用 `AIN1=PA9、AIN2=PA8`；若后续 PCB 网表与此不一致，必须先改硬件，不能把两个方向输入同时配置为 PA8。

## 当前工程状态

| 功能 | 状态 | 当前实际行为 |
|---|---|---|
| 八路循迹输入 | 已改为 GPIO | `1=亮灯/白色`，`0=不亮/黑带` |
| 循迹转向 PID | 启用 | 10 ms 计算黑线重心，经过误差低通、PID 和差速变化率限制后生成目标轮速 |
| 双路电机 PWM | 启用 | TIMG0 双通道，约 10 kHz，比较值范围 `0~4000` |
| 双编码器测速 | 启用 | 每个编码器只统计 A 相上升沿，10 ms 计算一次速度 |
| 双路增量式 PID | 启用 | 默认 `Kp=0.5`、`Ki=0.4`、`Kd=0.1` |
| UART 调试输出 | TX 启用 | H4 / PA28，115200 8N1，每 500 ms 输出一次八路逻辑值 |
| UART 接收 | 仅配置 | SysConfig 配置了 RX 中断，但 NVIC 和处理代码未启用 |
| K230 UART3 | 引脚已配置 | H5：PA25=RX、PA26=TX，9600 8N1；当前工程尚未解析视觉协议 |
| 扩展 I²C1 | 引脚已配置 | PB2=SCL、PB3=SDA，400 kHz；与三个 4Pin I²C 接口共用 |
| 舵机 PWM | 仅初始化 | PA27、50 Hz，启动时比较值为 `50`，没有动态控制 |
| LED0/LED1 | 仅配置 | PA14/PA15，当前没有运行时控制 |
| 启动按键 | 引脚已配置 | SW1 / PB6，内部下拉、高电平按下；当前尚未接入开始/停止状态机 |
| 干净重生成与对象编译 | 已通过 | 2026-07-29 使用 `make check` 验证通过 |

程序初始化后会立即启动 10 ms 电机控制定时器，没有独立的开始/停止状态机。首次上电调试必须架空车轮。

## 软件运行流程

初始化阶段：

1. `SYSCFG_DL_init()` 初始化 80 MHz 时钟、GPIO、PWM、定时器、UART3、UART0 和 I²C1。
2. 启用 GPIOB 中断，计划用于两个编码器 A 相计数。
3. 启动 TIMG7 舵机 PWM，并把比较值设为 `50`。
4. 把两路目标速度清零。
5. `motor_init(3)` 初始化 TB6612FNG A/B 通道，启动 TIMG0 电机 PWM 和 TIMA0 10 ms 控制定时器。

运行阶段包含两条并行路径：

- 主循环：读取八路 GPIO，拼成 `11111111` 形式的亮灭状态字符串，通过 UART0 每 500 ms 发送一次。
- TIMA0 中断：读取八路 GPIO → 计算黑线重心和转向 PID → 更新两路目标速度 → 计算编码器速度 → 执行双路增量式速度 PID → 更新 PWM。
- K230 UART3 由 SysConfig 完成引脚和波特率初始化，但本工程暂未启用接收中断或协议解析。

`delay_ms(500)` 只阻塞主循环；中断中的循迹、测速和 PID 仍按 10 ms 周期运行。

## 八路 GPIO 循迹模块

### 信号顺序与逻辑

源码把传感器按车体从左到右解释为：

```text
最左                                                     最右
L1      L2      L3      L4      R1      R2      R3      R4
v[0]    v[1]    v[2]    v[3]    v[4]    v[5]    v[6]    v[7]
```

用户实测该模块只有靠近白色物体时指示灯才会亮。每路 GPIO 在 SysConfig 中配置为数字输入和内部下拉；当前模块输出为低电平有效，因此 `huidu_get_value()` 转换后的含义为：

| 引脚原始电平 | 物理指示灯/表面 | `huidu_value[]` | 循迹含义 |
|---|---|---:|---|
| 低电平 | 亮灯，白色 | `1` | 未压到黑带 |
| 高电平 | 不亮，黑色胶带 | `0` | 压到黑带 |

直行时黑色胶带位于 L4 和 R1 两个中心探头之间，八路探头都照到白色并全部亮灯，通常会把 `11111111` 判为直行。若全亮前黑线重心已偏到 `±2` 以外，则保持原搜索方向最多 150 ms，随后把历史误差逐步衰减到零；这样既能在刚越过黑线时继续找线，也不会永久锁在单边大差速状态。

### GPIO 接线

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| 循迹 L1 / 最左 / H2-3 | 八路数字循迹模块，具体型号未注明 | PA18 | PINCM40 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 L2 / H2-4 | 同上 | PA16 | PINCM38 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 L3 / H2-5 | 同上 | PB7 | PINCM24 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 L4 / 左中 / H2-6 | 同上 | PA17 | PINCM39 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 R1 / 右中 | 同上 | PA21 | PINCM46 | GPIO 输入 | 亮/白=`1`，灭/黑=`0`；PA21 兼有 VREF- 相关功能 |
| 循迹 R2 | 同上 | PA22 | PINCM47 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 R3 | 同上 | PA24 | PINCM54 | GPIO 输入 | 内部下拉；亮/白=`1`，灭/黑=`0` |
| 循迹 R4 / 最右 / H2-10 | 同上 | PA2 | PINCM7 | GPIO 输入 | 内部下拉；按 PCB 配置，须确认核心板确实引出 PA2 |

H2-7、H2-8、H2-9 依次对应 R1、R2、R3。IOMUX 索引来自使用 SysConfig 1.26.2 根据当前 `empty.syscfg` 重新生成的配置，不应以仓库中历史 `Debug/ti_msp_dl_config.h` 的旧索引为准。

### 黑带重心转向 PID

不亮探头使用以下横向权重：

```text
L1  L2  L3  L4  |  R1  R2  R3  R4
-4  -3  -2  -1  |  +1  +2  +3  +4
                 车体中心
```

算法只把值为 `0` 的不亮探头计入重心。加权结果为负表示黑带偏左，为正表示黑带偏右，绝对值越大表示偏得越远。多路同时压到胶带时取整体平均位置：

```text
e_raw = 所有压到黑带探头的权重之和 / 压到黑带的探头数量
e(k)  = 0.25 × e_raw + 0.75 × e(k-1)
I(k)  = clamp[I(k-1) + e(k) × 0.01, -2, +2]
D(k)  = [e(k) - e(k-1)] / 0.01
u_req = clamp[20 × e(k) + 2 × I(k) + 0.12 × D(k), -100, +100]
u(k)  = move_toward[u(k-1), u_req, 8]
```

其中 `u` 是差速修正量，每 10 ms 最多改变 `8`，避免数字探头在中心附近切换时差速立即从左打满跳到右打满。最终目标轮速为：

```text
右轮目标 = clamp(260 - u, 140, 380)
左轮目标 = clamp(260 + u, 140, 380)
```

转向环初始参数如下：

| 参数 | 值 | 作用 |
|---|---:|---|
| 控制周期 | `10 ms` | 与 TIMA0 中断一致 |
| 基准轮速 | `260` | 直行目标速度 |
| `Kp` | `20` | 主要转向力度 |
| `Ki` | `2` | 补偿长期机械偏置 |
| `Kd` | `0.12` | 抑制穿越中心后的过冲 |
| 误差低通系数 | `0.25` | 减弱八路数字量跳变 |
| 积分限幅 | `±2` | 防止丢线或长弯道积分饱和 |
| 差速修正限幅 | `±100` | 限制最大左右轮偏置 |
| 差速单周期最大变化 | `8 / 10 ms` | 平滑转向指令 |
| 单轮目标限幅 | `140~380` | 防止一侧停转、另一侧过快 |

根据 2026-07-27 实车反馈，电机 1/A 通道是右轮，电机 2/B 通道是左轮。黑带偏左时右轮加速、左轮减速；黑带偏右时左轮加速、右轮减速。电机 2 的软件正转极性也已经对调：`direction=1` 时 BIN1=低、BIN2=高，使两只电机都朝车辆前进方向转动。

旧版四档差速和额外的 200 ms 目标轮速插值已经删除，因为离散换档和目标滞后会加剧 S 形摆动。现在由转向 PID 自身限制差速变化率；速度 PID 算出的实际 PWM 仍在输出前限制为每个 10 ms 周期最多变化 `200`，作为执行器层保护。

实车整定时可在 CCS Expressions 中观察 `huidu_line_error` 和 `huidu_steer_correction`。如果仍有规律性 S 形摆动，优先把 `TRACK_PID_KP` 从 `20` 降到 `16`；若主要是探头跳变，把 `TRACK_ERROR_FILTER_ALPHA` 从 `0.25` 降到 `0.18`；若过弯后穿越中心明显，可小幅把 `TRACK_PID_KD` 从 `0.12` 提到 `0.16`。不要先增大 `Ki`，否则更容易产生低频摆动。

根据当前 `EDGE_ALIGN_UP` 配置和 2026-07-27 实车验证，本工程直接把逻辑 PWM duty 写入 TIMG0 比较寄存器：`0=停止`，数值增大时实际电机输出增大。不要使用 `compare = 4000 - duty`；该转换会使上电后的首个 PID 输出落在约 `3800/4000`，导致两轮接近满速。

## 电机测速与 PID

源码当前使用以下参数：

| 参数 | 值 |
|---|---:|
| 编码器计数常量 `MOTOR_BIANMAQI` | `260` |
| 轮胎直径 `MOTOR_WHEEL_D` | `48 mm` |
| 控制周期 | `10 ms` |
| `Kp` | `0.5` |
| `Ki` | `0.4` |
| `Kd` | `0.1` |
| PWM 限幅 | `0~4000` |
| PWM 单周期最大变化 | `200 / 10 ms` |
| TIMG0 比较值 | 与逻辑 PWM duty 相同 |

速度计算公式为：

```text
speed_mm_s = encoder_A_rising_count / 260 × 3.14 × 48 × 100
```

增量式 PID 为：

```text
e(k) = target_speed - measured_speed
ΔPWM = Kp × [e(k)-e(k-1)]
     + Ki × e(k)
     + Kd × [e(k)-2e(k-1)+e(k-2)]
PWM_request(k) = clamp(PWM(k-1) + ΔPWM, 0, 4000)
PWM(k) = move_toward(PWM(k-1), PWM_request(k), 200)
```

两个编码器的 B 相都只配置为普通输入，没有参与计数或正交方向判断。因此当前只能统计 A 相上升沿数量，不能得到有符号速度。

## 当前 SysConfig 引脚总表

下表以 `empty.syscfg` 和使用 SysConfig 1.26.2 干净生成的配置为准。

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| 右轮电机 A PWM / PWMA | TB6612FNG | PA12 | PINCM34 | TIMG0_CCP0 | 约 10 kHz |
| 右轮电机 A 方向 AIN1 | TB6612FNG | PA9 | PINCM20 | GPIO 输出 | 软件正转时为低电平 |
| 右轮电机 A 方向 AIN2 | TB6612FNG | PA8 | PINCM19 | GPIO 输出 | 软件正转时为高电平 |
| 左轮电机 B PWM / PWMB | TB6612FNG | PA13 | PINCM35 | TIMG0_CCP1 | 约 10 kHz |
| 左轮电机 B 方向 BIN1 | TB6612FNG | PA7 | PINCM14 | GPIO 输出 | 软件正转时为低电平 |
| 左轮电机 B 方向 BIN2 | TB6612FNG | PB18 | PINCM44 | GPIO 输出 | 软件正转时为高电平 |
| 电机驱动使能 STBY | TB6612FNG | PB24 | PINCM52 | GPIO 输出 | A/B 通道共用 |
| 右轮电机 A 编码器 A 相 | 带编码器直流电机，具体型号未注明 | PB8 | PINCM25 | GPIO 输入/上升沿中断 | 参与计数 |
| 右轮电机 A 编码器 B 相 | 同上 | PB9 | PINCM26 | GPIO 输入 | 当前未参与方向判断 |
| 左轮电机 B 编码器 A 相 | 同上 | PB19 | PINCM45 | GPIO 输入/上升沿中断 | 参与计数 |
| 左轮电机 B 编码器 B 相 | 同上 | PB20 | PINCM48 | GPIO 输入 | 无内部上下拉，当前不参与方向判断 |
| 调试串口 TX / H4-1 | 外接 3.3 V USB-UART | PA28 | PINCM3 | UART0_TX | 115200 8N1 |
| 调试串口 RX / H4-2 | 外接 3.3 V USB-UART | PA31 | PINCM6 | UART0_RX | 当前接收处理未启用 |
| K230 UART RX / H5-2 | 亚博 K230 通信座 TXD / IO9 | PA25 | PINCM55 | UART3_RX | K230 → 小车，9600 8N1 |
| K230 UART TX / H5-1 | 亚博 K230 通信座 RXD / IO10 | PA26 | PINCM59 | UART3_TX | 小车 → K230，当前仅预留 |
| I²C1 SCL / H1、H6、H7 | OLED 等 I²C 外设 | PB2 | PINCM15 | I2C1_SCL | 400 kHz，模块侧需有 3.3 V 上拉 |
| I²C1 SDA / H1、H6、H7 | 同上 | PB3 | PINCM16 | I2C1_SDA | 400 kHz，模块侧需有 3.3 V 上拉 |
| 启动按键 / SW1 | 板载按键 | PB6 | PINCM23 | GPIO 输入 | 内部下拉，按下接 3.3 V |
| 舵机 PWM / H3-1 | 具体型号未注明 | PA27 | PINCM60 | TIMG7_CCP1 | 50 Hz，当前无动态控制 |
| LED0 | 外接 LED，具体型号未注明 | PA14 | PINCM36 | GPIO 输出 | 当前未驱动 |
| LED1 | 外接 LED，具体型号未注明 | PA15 | PINCM37 | GPIO 输出 | 当前未驱动 |
| SWDIO | SEGGER J-Link | PA19 | 专用调试引脚 | SWDIO | 不得复用 |
| SWCLK | SEGGER J-Link | PA20 | 专用调试引脚 | SWCLK | 不得复用 |
| HFXIN | 40 MHz HFXT 时钟节点 | PA5 | PINCM10 | HFXIN | 时钟专用，不得改作普通 GPIO |
| HFXOUT | 40 MHz HFXT 时钟节点 | PA6 | PINCM11 | HFXOUT | 时钟专用，不得改作普通 GPIO |

八路循迹引脚见上一节的 H2 独立接线表。PA16 已由原 ADC 输入改为循迹 L2，ADC/VREF 实例已从 SysConfig 删除。

## 电源与电平

- MSPM0G3507 GPIO 为 3.3 V 电平，任何传感器输出都不能直接向 MCU 输入 5 V。
- 循迹模块型号和供电范围未在仓库中注明。若模块使用 5 V 供电，必须确认 8 路数字输出的高电平不超过 3.3 V，必要时使用电平转换。
- TB6612FNG 逻辑电源按源码注释使用 3.3 V，电机电源 VM 为 7.4 V。
- MCU、K230、循迹模块、TB6612FNG、编码器和外接 USB-UART 必须共地。
- K230 与小车各自供电时，H5 只接 TX、RX、GND，不连接 K230 通信座的 5V，避免两路电源回灌。
- H4 调试 UART 使用 PA28/PA31；PA0/PA1 不再被本工程的 UART 或 I²C 占用。
- PB2/PB3 是开漏 I²C，总线必须有到 3.3 V 的上拉；底板原理图未画出上拉时，应由所接模块提供。
- 循迹 R4 按 PCB 使用 PA2。PA2 兼有 ROSC 功能，烧录前必须确认所用地猛星核心板版本已把该脚实际引到母座，且没有时钟器件占用。

## 工程结构

```text
11_PID_car/
├── main.c                         # 初始化、GPIO 循迹打印和主循环
├── empty.syscfg                   # 时钟、引脚和外设配置源
├── user_driver/
│   ├── delay.c/.h                 # 阻塞毫秒延时
│   ├── huidu.c/.h                 # 八路 GPIO 读取和循迹规则
│   ├── key.c/.h                   # 当前只保留编码器 GPIOB 中断计数
│   ├── motor.c/.h                 # TB6612FNG、测速和双路增量式 PID
│   └── uart.c/.h                  # 阻塞式 UART 发送
├── .vscode/c_cpp_properties.json  # Mac/Windows 共享 IntelliSense 配置
├── Makefile                       # macOS SysConfig/对象编译检查脚本
├── targetConfigs/
│   └── MSPM0G3507.ccxml           # MSPM0G3507 + SEGGER J-Link 调试配置
├── .project/.cproject/.ccsproject # CCS Theia 工程元数据
└── Debug/                         # 历史生成物，不能代替干净生成验证
```

仓库当前跟踪了部分 `Debug/` 生成物，其中可能保留旧宏、旧 IOMUX 索引和其他电脑的绝对路径。跨电脑协作时应以源码、`empty.syscfg` 和当前机器重新生成的文件为准。

## 工具链

| 组件 | 工程使用版本 |
|---|---|
| 目标器件 | MSPM0G3507，LQFP-64 |
| CPU 主频 | 80 MHz |
| MSPM0 SDK | 2.10.00.04 |
| SysConfig | 1.26.2 |
| CCS 工程记录的 TI ArmClang | 4.0.4.LTS |
| 当前 macOS 验证工具链 | TI ArmClang 5.1.1.LTS |
| 调试器 | SEGGER J-Link，SWD |

## VS Code 跨平台配置

`.vscode/c_cpp_properties.json` 使用环境变量，不保存某位开发者的 TI 安装绝对路径。每台电脑需要分别设置：

- `MSPM0_SDK_ROOT`：MSPM0 SDK 根目录，内部应有 `source` 和 `.metadata/product.json`。
- `TI_ARMCLANG_ROOT`：TI ArmClang 版本根目录，内部应有 `bin/tiarmclang` 或 `bin/tiarmclang.exe`。

### macOS

把实际路径加入 `~/.zshrc`，再从新终端启动 VS Code：

```bash
export MSPM0_SDK_ROOT=/Users/你的用户名/ti/mspm0_sdk_2_10_00_04
export TI_ARMCLANG_ROOT=/Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS
```

### Windows

在 PowerShell 中执行，路径按实际安装位置修改：

```powershell
$SdkRoot = "D:\ti\ccs2050\mspm0_sdk_2_10_00_04"
$CompilerRoot = "D:\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS"

[Environment]::SetEnvironmentVariable("MSPM0_SDK_ROOT", $SdkRoot, "User")
[Environment]::SetEnvironmentVariable("TI_ARMCLANG_ROOT", $CompilerRoot, "User")
```

完全关闭并重启 VS Code，直接打开 `11_PID_car` 文件夹，然后在 `C/C++: Select IntelliSense Configuration` 中选择：

- macOS：`Mac`
- Windows：`Win32`

`.vscode` 只负责代码提示，CCS 的正式编译仍由工程元数据、SDK 和 SysConfig 决定。

## 构建与验证

### macOS 命令行检查

在工程根目录执行：

```bash
make check
```

该命令会依次重新生成 SysConfig、检查代码依赖的关键宏，并使用 TI ArmClang 对 `main.c`、全部 `user_driver/*.c` 和生成的 `ti_msp_dl_config.c` 进行对象编译。检查产物写入 `/tmp/11_PID_car_check`，不会覆盖 CCS 的 `Debug/`。

### CCS Theia

1. 安装 MSPM0 SDK 2.10.00.04 和 SysConfig 1.26.2。
2. 导入 `11_PID_car` 工程。
3. 打开 `empty.syscfg`，核对器件、LQFP-64 封装和全部实际接线。
4. Clean/Rebuild，让 CCS 在本机重新生成配置和对象文件。
5. 使用 `targetConfigs/MSPM0G3507.ccxml` 和 SEGGER J-Link 调试或烧录。

不要把仓库中的历史 `Debug/ti_msp_dl_config.h` 当作干净生成结果。

若需要从 macOS 终端复现 CCS 的完整 Debug 构建，应使用 CCS 自带的 GNU Make，例如：

```bash
/Applications/ti/ccs2100/ccs/utils/bin/gmake -C Debug all
```

macOS 系统自带的 GNU Make 3.81 不支持 CCS 生成文件使用的 `-Onone` 参数；直接运行 `make -C Debug all` 会在递归构建处报错。CCS IDE 的 Build 按钮会使用其自带版本，不受此问题影响。

### 当前验证结果

2026-07-29 在 macOS 上使用 MSPM0 SDK 2.10.00.04、SysConfig 1.26.2 和 TI ArmClang 5.1.1.LTS 检查：

- `empty.syscfg` 干净生成成功，UART0、UART3、I²C1、8 路 `XUNJI`、SW1、电机、编码器和舵机之间没有引脚冲突。
- 生成结果确认 K230 为 PA25/PA26 UART3 9600、调试口为 PA28/PA31 UART0 115200、共享 I²C1 为 PB2/PB3 400 kHz。
- `Makefile` 会逐项核对 PCB 关键端口、引脚、外设实例和波特率，任何后续误改都会让 `make check` 失败。
- `main.c`、全部 `user_driver/*.c` 和生成的 `ti_msp_dl_config.c` 对象编译均为 0 报错。

以上结果证明当前工程能够干净生成 SysConfig 并通过对象编译，但不等同于烧录或实车功能验证。本次检查未覆盖完整链接，烧录前仍应在当前电脑上重新执行一次检查和 CCS Clean/Rebuild。

## 串口调试输出

连接外部 3.3 V USB-UART：

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| USB-UART RX ← MCU TX | 3.3 V USB-UART，具体型号未注明 | H4-1 / PA28 | PINCM3 | UART0_TX | 115200 8N1 |
| USB-UART TX → MCU RX | 同上 | H4-2 / PA31 | PINCM6 | UART0_RX | 当前程序不处理接收 |
| 公共地 | 同上 | GND | 不适用 | GND | 必须共地 |

主循环每 500 ms 输出一行 8 个数字。直行时的典型输出为：

```text
11111111
```

数字顺序是 `L1 L2 L3 L4 R1 R2 R3 R4`；其中 `1` 表示探头亮灯、看到白色，`0` 表示不亮、压到黑色胶带。例如 `11101111` 表示 L4 不亮，控制器会轻微向左修正。

## K230 串口接线

亚博 K230 仍使用板载四针通信座和 `YbUart(baudrate=9600)`，只需要在小车端改接到底板 H5：

```text
K230 TXD / IO9   → H5-2 / PA25 / UART3_RX
K230 RXD / IO10  ← H5-1 / PA26 / UART3_TX
K230 GND         ↔ H5-3 / GND
```

TX 与 RX 必须交叉，且两板必须共地。`empty.syscfg` 已初始化 `K230_LINK` 为 9600 8N1，但 `11_PID_car` 目前只完成硬件接口迁移，尚未实现视觉帧解析；需要视觉闭环时可复用同工作区 `k230control` 的 CRC-8/ATM 行协议与接收状态机。

## 上板前检查

1. 确认 H2-3～H2-10 依次是 L1、L2、L3、L4、R1、R2、R3、R4。
2. 把各探头分别放在白底和黑带上，确认串口对应输出为 `1` 和 `0`。
3. 确认循迹模块输出电平不超过 3.3 V，并让所有模块共地。
4. 核对 TB6612FNG 的 `AIN1=PA9、AIN2=PA8`，不要按原理图重复的 `PA8` 标注接成同一信号。
5. 核对编码器 A/B 相；当前 A 通道已经改为 PB8/PB9，不再是 PB13/PA22。
6. 确认核心板已实际引出 PA2，且 R4 在白底/黑带上都能可靠翻转。
7. 确认 H4 的 PA28/PA31 接 3.3 V USB-UART，H5 的 PA25/PA26 与 K230 TX/RX 交叉。
8. 确认 PB2/PB3 I²C 总线存在到 3.3 V 的上拉，并让所接 I²C 设备地址互不冲突。
9. 确认 PA19/PA20 只用于 J-Link SWD。
10. 通过“SysConfig 生成 → PCB 宏检查 → 对象编译”后再烧录。
11. 第一次运行时架空车轮，因为电机控制定时器会在初始化后立即启动。

## 已知限制

- `make check` 只验证 SysConfig、宏和对象编译，不执行完整链接、烧录或实车功能测试。
- 循迹模块的具体型号和供电范围没有记录，接线前仍需确认输出高电平不超过 3.3 V。
- `11111111` 既可能表示黑带正确位于 L4/R1 间隙，也可能表示车辆完全离开黑带进入纯白区域；代码只在上一有效误差绝对值不小于 `2` 时短暂保持搜索方向，150 ms 后开始衰减，因此不能保证所有高速丢线场景都能重新找回黑线。
- 转向 PID 的初始参数以减少 S 形摆动为目标，仍需根据传感器离地高度、车速、轮胎摩擦和实际赛道继续整定。
- 八路全灭时无法判断胶带中心，代码会把两路目标速度设为 `0`。
- SW1 / PB6 已配置为启动按键输入，但开始/停止状态机尚未实现，上电后控制定时器仍会立即运行。
- 编码器只统计 A 相上升沿，无法判断方向，也没有利用 B 相提高分辨率。
- PID 参数、轮径、编码器计数常量和目标速度均写死在源码中，没有在线调参或掉电保存。
- K230 UART3 只完成引脚和波特率初始化，视觉协议解析尚未接入主运行流程。
- UART0 RX、LED 和舵机动态控制尚未接入主运行流程；原 PA16 ADC/VREF 已删除。
- SysConfig 同时配置了 PA5/PA6 的 40 MHz HFXT 和 80 MHz CPU 时钟，修改时钟树前必须重新验证生成结果。
