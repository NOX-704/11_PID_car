# 11_PID_car

基于 TI MSPM0G3507 的双直流电机循迹小车工程。当前实现使用 I2C 八路灰度模块生成两路目标轮速，再使用编码器反馈和增量式 PID 分别调节 TB6612FNG 的 A/B 两个电机通道。

> 当前“循迹方向决策”是离散规则控制，不是横向误差 PID；PID 只用于两个电机通道的速度闭环。

## 当前工程状态

| 功能 | 当前状态 | 实际行为 |
|---|---|---|
| 八路灰度读取 | 启用 | I2C1 地址 `0x12`，发送命令 `0x30` 后读取 1 字节 |
| 循迹决策 | 启用 | 根据 8 个逻辑位选择 A/B 通道目标速度 |
| 双路电机 PWM | 启用 | TIMG0 双通道，10 kHz，比较值范围 `0~4000` |
| 双编码器测速 | 启用 | 只统计每个编码器 A 相上升沿，10 ms 计算一次速度 |
| 双路增量式 PID | 启用 | 10 ms 周期，默认 `Kp=0.5`、`Ki=0.4`、`Kd=0.1` |
| UART 调试输出 | 启用 TX | PA10，115200 8N1，每 500 ms 输出一次八路逻辑值 |
| UART 接收 | 仅配置 | RX 中断已在 SysConfig 配置，但 NVIC 和处理函数未启用 |
| KEY9/KEY10 | 部分启用 | 中断会修改 `status`，但当前控制流程不使用该变量 |
| LED0/LED1 | 仅配置 | 主循环中的闪烁代码已注释 |
| ADC1 通道 1 | 仅配置 | PA16、内部 2.5 V VREF；读取和显示代码已注释 |
| 舵机 PWM | 仅初始化 | PA27、50 Hz；启动时比较值写为 `50`，之后没有控制逻辑 |

程序启动后会立即启用电机控制定时器，没有独立的“开始/停止”按键状态机。首次上电调试必须架空车轮。

## 控制流程

初始化阶段：

1. `SYSCFG_DL_init()` 初始化时钟、GPIO、PWM、I2C、UART、ADC 和 VREF。
2. 启用 GPIOB 组合中断，用于按键和两个编码器 A 相计数。
3. 启用 ADC 转换并启动 TIMG7 舵机 PWM。
4. `motor_init(3)` 初始化 TB6612FNG A/B 两个通道、启动 TIMG0 PWM 和 TIMA0 10 ms 控制定时器。

运行阶段包含两条并行路径：

- 主循环：读取八路灰度值，拼成 `00011000` 形式的字符串，通过 UART0 每 500 ms 发送一次。
- TIMA0 中断（每 10 ms）：读取灰度值 → 更新目标轮速 → 计算编码器速度 → 执行双路增量式 PID → 更新 PWM。

`delay_ms(500)` 只阻塞主循环；中断中的测速和 PID 仍会继续运行。

## 八路灰度数据和循迹策略

灰度模块的具体商品型号未在仓库中注明。代码已知参数如下：

- 接口：I2C1，100 kHz
- 从机地址：`0x12`
- 读取命令：`0x30`
- 数据宽度：8 bit
- 原始数据读回后执行按位取反，因此 `huidu_value[n] == 1` 表示取反后的有效/检测状态
- `S0`/`huidu_value[0]` 为最左侧，`S7`/`huidu_value[7]` 为最右侧

规则按源码中的先后顺序匹配：

| 条件 | 电机 1 / A 通道目标速度 | 电机 2 / B 通道目标速度 |
|---|---:|---:|
| 8 路全为 `0` | 两路取原目标值中的较小值 | 同左 |
| 8 路全为 `1` | `0` | `0` |
| 只有中心 `S3/S4` 命中 | 两路取原目标值中的较小值 | 同左 |
| 左侧 `S3` 命中且未被前述条件匹配 | `275` | `400` |
| 左侧 `S2` 命中 | `250` | `400` |
| 左侧 `S1` 命中 | `225` | `400` |
| 最左 `S0` 命中 | `200` | `400` |
| 右侧 `S4` 命中 | `400` | `275` |
| 右侧 `S5` 命中 | `400` | `225` |
| 右侧 `S6` 命中 | `400` | `200` |
| 最右 `S7` 命中 | `400` | `200` |

这里的速度单位按代码定义为 `mm/s`。电机 A/B 通道分别安装在车体哪一侧没有在仓库中注明，必须以实际接线确认，不能直接假定 A=左轮或 B=右轮。

## 速度计算与 PID

代码假设：

- 编码器线数：`260`
- 轮胎直径：`48 mm`
- 控制周期：`10 ms`

速度计算公式：

```text
speed_mm_s = encoder_count / 260 × π × 48 × 100
```

每个通道使用独立状态的增量式 PID：

```text
e(k) = target_speed - measured_speed
ΔPWM = Kp × [e(k)-e(k-1)]
     + Ki × e(k)
     + Kd × [e(k)-2e(k-1)+e(k-2)]
PWM(k) = clamp(PWM(k-1) + ΔPWM, 0, 4000)
```

默认参数定义在 `user_driver/motor.c`：

| 参数 | 默认值 |
|---|---:|
| `Kp` | `0.5` |
| `Ki` | `0.4` |
| `Kd` | `0.1` |
| PWM 下限 | `0` |
| PWM 上限 | `4000` |

当前编码器 B 相只配置为普通输入，没有参与计数或方向判断，因此测得的是 A 相上升沿数量，不能识别轮速正负方向。

## 当前 SysConfig 引脚

以下内容来自 `empty.syscfg` 和它生成的 `ti_msp_dl_config.h`，是当前代码实际使用的映射。`user_driver/motor.h` 中部分 AIN/BIN 接线注释与 SysConfig 名称相反，应以本表和 `empty.syscfg` 为准。

| 外设功能 | 芯片/模块型号 | MSPM0G3507/地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| 电机 A PWM / PWMA | TB6612FNG | PA12 | PINCM34 | TIMG0_CCP0 | 10 kHz |
| 电机 A 方向 AIN1 | TB6612FNG | PA9 | PINCM20 | GPIO 输出 | 以 SysConfig 命名为准 |
| 电机 A 方向 AIN2 | TB6612FNG | PA8 | PINCM19 | GPIO 输出 | 以 SysConfig 命名为准 |
| 电机 B PWM / PWMB | TB6612FNG | PA13 | PINCM35 | TIMG0_CCP1 | 10 kHz |
| 电机 B 方向 BIN1 | TB6612FNG | PA7 | PINCM14 | GPIO 输出 | 以 SysConfig 命名为准 |
| 电机 B 方向 BIN2 | TB6612FNG | PB18 | PINCM44 | GPIO 输出 | 以 SysConfig 命名为准 |
| 电机驱动使能 STBY | TB6612FNG | PB24 | PINCM52 | GPIO 输出 | A/B 通道共用 |
| 电机 A 编码器 A 相 | 带编码器直流减速电机，具体型号未注明 | PB13 | PINCM30 | GPIO 输入/上升沿中断 | 参与计数 |
| 电机 A 编码器 B 相 | 同上 | PA22 | PINCM47 | GPIO 输入 | 当前未参与方向判断 |
| 电机 B 编码器 A 相 | 同上 | PB19 | PINCM45 | GPIO 输入/上升沿中断 | 参与计数 |
| 电机 B 编码器 B 相 | 同上 | PB20 | PINCM48 | GPIO 输入 | 当前未参与方向判断 |
| 灰度模块 I2C1 SCL | I2C 八路灰度模块，具体型号未注明 | PB2 | PINCM15 | I2C1_SCL | 100 kHz |
| 灰度模块 I2C1 SDA | 同上 | PB3 | PINCM16 | I2C1_SDA | 100 kHz |
| 调试串口 TX | 外接 3.3 V USB-UART | PA10 | PINCM21 | UART0_TX | 115200 8N1 |
| 调试串口 RX | 外接 3.3 V USB-UART | PA11 | PINCM22 | UART0_RX | 当前接收处理未启用 |
| 舵机 PWM | 具体型号未注明 | PA27 | PINCM60 | TIMG7_CCP1 | 50 Hz，当前无动态控制 |
| ADC 输入 | 外部模拟信号，具体模块未注明 | PA16 | PINCM38 | ADC1 通道 1 | 12 bit，内部 2.5 V 参考 |
| KEY9 | 外接按键，具体型号未注明 | PB6 | PINCM23 | GPIO 输入/上升沿中断 | 内部下拉 |
| KEY10 | 外接按键，具体型号未注明 | PB7 | PINCM24 | GPIO 输入/上升沿中断 | 内部下拉 |
| LED0 | 外接 LED，具体型号未注明 | PA14 | PINCM36 | GPIO 输出 | 当前未驱动 |
| LED1 | 外接 LED，具体型号未注明 | PA15 | PINCM37 | GPIO 输出 | 当前未驱动 |
| SWDIO | SEGGER J-Link | PA19 | 专用调试引脚 | SWDIO | 不得复用 |
| SWCLK | SEGGER J-Link | PA20 | 专用调试引脚 | SWCLK | 不得复用 |
| HFXIN | 40 MHz HFXT 时钟节点 | PA5 | PINCM10 | HFXIN | 见下方时钟说明 |
| HFXOUT | 40 MHz HFXT 时钟节点 | PA6 | PINCM11 | HFXOUT | 见下方时钟说明 |

### 电源和电平

- MSPM0G3507 GPIO 是 3.3 V 电平，任何信号都不能直接输入 5 V。
- TB6612FNG 逻辑电源使用 3.3 V；`motor.h` 注释中的电机电源 VM 为 7.4 V。所有电源必须共地。
- 灰度模块供电电压在仓库中没有可靠型号资料。即使模块使用 5 V 供电，也必须确认 I2C 上拉电阻连接到 3.3 V，禁止让 PB2/PB3 承受 5 V。
- 当前调试 UART 使用 PA10/PA11，不是地猛星板载 CH340 默认使用的 PA0/PA1；需要外接 3.3 V USB-UART。
- 电机 A/B 通道与车体左右轮的对应关系必须根据实车确认。

### 时钟配置说明

SysConfig 元数据把 PA5/PA6 分配给 40 MHz HFXT 节点，但当前生成代码使用 SYSOSC 作为 SYSPLL 参考，并生成 `CPUCLK_FREQ=80000000`。在重新核对时钟树之前，不要把 PA5/PA6 改作普通 GPIO，也不要假设外部 40 MHz 晶振一定参与了 CPU 时钟。

## 工程结构

```text
11_PID_car/
├── main.c                         # 初始化、主循环灰度读取和 UART 输出
├── empty.syscfg                   # 时钟、引脚和外设的唯一配置源
├── user_driver/
│   ├── delay.c/.h                 # 基于 CPUCLK_FREQ 的阻塞毫秒延时
│   ├── huidu.c/.h                 # I2C 八路灰度读取和循迹规则
│   ├── key.c/.h                   # 按键与编码器 GPIOB 中断
│   ├── motor.c/.h                 # TB6612FNG、测速和双路增量式 PID
│   └── uart.c/.h                  # 阻塞式 UART 字符串发送
├── .vscode/c_cpp_properties.json  # Mac/Windows 共享 IntelliSense 配置
├── Makefile                       # macOS SysConfig/对象编译验证
├── targetConfigs/
│   └── MSPM0G3507.ccxml           # MSPM0G3507 + SEGGER J-Link 调试配置
├── .project/.cproject/.ccsproject # CCS Theia 工程元数据
└── Debug/                         # CCS 生成物；可能包含旧机器绝对路径
```

`Debug/` 中的 Makefile、`.out`、`.o` 和 SysConfig 文件是历史生成物，不应作为跨电脑配置源。换电脑或更新工具链后应由 CCS 重新生成。

## 工具链版本

| 组件 | 工程记录/当前验证版本 |
|---|---|
| 目标器件 | MSPM0G3507，LQFP-64 |
| MSPM0 SDK | 2.10.00.04 |
| SysConfig | 1.26.2 |
| CCS 工程记录的 TI ArmClang | 4.0.4.LTS |
| 当前 macOS `make check` 验证的 TI ArmClang | 5.1.1.LTS |
| 调试器 | SEGGER J-Link，SWD |

## VS Code 跨平台路径配置

`.vscode/c_cpp_properties.json` 不保存个人电脑的绝对路径。每台电脑只配置以下两个用户环境变量：

- `MSPM0_SDK_ROOT`：MSPM0 SDK 根目录，目录内应存在 `source` 和 `.metadata/product.json`
- `TI_ARMCLANG_ROOT`：TI ArmClang 版本根目录，目录内应存在 `bin/tiarmclang` 或 `bin/tiarmclang.exe`

### macOS

把以下内容加入 `~/.zshrc`，然后从新终端启动 VS Code：

```bash
export MSPM0_SDK_ROOT=/Users/rylum/ti/mspm0_sdk_2_10_00_04
export TI_ARMCLANG_ROOT=/Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS
```

### Windows

在 PowerShell 中执行，路径按实际安装位置修改：

```powershell
$SdkRoot = "D:\ti\ccs2050\mspm0_sdk_2_10_00_04"
$CompilerRoot = "D:\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS"

[Environment]::SetEnvironmentVariable("MSPM0_SDK_ROOT", $SdkRoot, "User")
[Environment]::SetEnvironmentVariable("TI_ARMCLANG_ROOT", $CompilerRoot, "User")

$env:MSPM0_SDK_ROOT = $SdkRoot
$env:TI_ARMCLANG_ROOT = $CompilerRoot
```

检查环境变量：

```powershell
Test-Path "$env:MSPM0_SDK_ROOT\source"
Test-Path "$env:MSPM0_SDK_ROOT\.metadata\product.json"
Test-Path "$env:TI_ARMCLANG_ROOT\bin\tiarmclang.exe"
Test-Path "$env:TI_ARMCLANG_ROOT\include"
```

四项都应为 `True`。完全关闭并重启 VS Code，打开 `11_PID_car` 文件夹，然后执行 `C/C++: Select IntelliSense Configuration`：

- macOS 选择 `MSPM0G3507-macOS`
- Windows 选择 `MSPM0G3507-Windows`

`.vscode` 只负责代码提示，实际 CCS 编译仍由 `.cproject`、SDK 和 SysConfig 产品配置决定。

## 构建与验证

### macOS 快速验证

在工程根目录运行：

```bash
make check
```

该命令会执行：

1. 检查本机 SDK、SysConfig 和 TI ArmClang 路径。
2. 根据 `empty.syscfg` 重新生成配置文件。
3. 检查代码依赖的关键 SysConfig 宏。
4. 用 TI ArmClang 逐文件编译 `main.c`、全部 `user_driver/*.c` 和生成的 `ti_msp_dl_config.c`。

`make check` 只做 SysConfig 和对象编译验证，不执行完整链接，也不会烧录。

### Windows / CCS Theia

1. 安装或导入 MSPM0 SDK 2.10.00.04 与 SysConfig 1.26.2。
2. 在 CCS Theia 中导入现有工程目录。
3. 检查 `empty.syscfg` 的器件、封装、引脚和实际接线。
4. 执行 Clean/Rebuild，让 CCS 在当前电脑上重新生成 `Debug/`。
5. 使用 `targetConfigs/MSPM0G3507.ccxml` 和 SEGGER J-Link 进行 SWD 调试/烧录。

不要直接复用仓库里历史 `Debug/makefile` 的绝对 Windows 路径。

## 串口输出

连接外部 3.3 V USB-UART：

| USB-UART | MSPM0G3507 |
|---|---|
| RX | PA10 / UART0_TX |
| TX | PA11 / UART0_RX（当前程序未处理接收） |
| GND | GND |

串口参数为 `115200, 8N1`。当前每 500 ms 输出一行 8 个数字，例如：

```text
00011000
```

数字顺序为 `S0` 到 `S7`，即从最左到最右的取反后逻辑值。

## 上板前检查

1. 核对 TB6612FNG 的 PWMA/PWMB、AIN1/AIN2、BIN1/BIN2 和 A/B 通道对应的实际车轮。
2. 确认编码器 A/B 相接线与 SysConfig 一致；当前代码只统计 A 相。
3. 确认灰度模块地址确为 `0x12`、命令确为 `0x30`，且 I2C 上拉为 3.3 V。
4. 确认 PA10/PA11 使用外接 3.3 V USB-UART，不与其他模块冲突。
5. 确认 PA19/PA20 只用于 J-Link SWD。
6. 架空车轮后再上电，因为 PID 定时器会在初始化后立即启动。
7. 烧录前重新执行 SysConfig 生成和对象编译验证。

## 已知限制与后续改进

- 上电时两路目标速度初始为 `0`；如果灰度状态只命中“全 0”或“中心”规则，目标速度仍可能保持为 `0`，车辆不会自行起步。
- `status` 虽可由 KEY9/KEY10 修改，但没有接入启动、停止或模式切换逻辑。
- I2C 灰度读取在 10 ms 定时器中断内使用阻塞等待，会增加中断延迟；超时后也没有向上层返回错误状态。
- 编码器只统计 A 相上升沿，B 相未用于正交解码，不能判断方向，分辨率也取决于代码中的“260 线”定义方式。
- 循迹采用规则表，不会计算连续横向误差；多传感器同时命中时由 `if/else` 顺序决定结果。
- PID 参数、轮径、编码器线数和目标速度均为源码常量，尚无串口在线调参和参数持久化。
- UART RX、LED、ADC、舵机和按键模式控制尚未进入主运行流程。
- `motor.h` 的部分方向引脚注释与 `empty.syscfg` 不一致，维护时必须以 SysConfig 生成宏为准。
- SysConfig 的 PA5/PA6 HFXT 分配与生成代码的 SYSOSC PLL 参考需要进一步核对。

## 验证记录

2026-07-25 在 macOS 上完成：

- `c_cpp_properties.json` JSON 结构检查通过
- SysConfig 1.26.2 生成通过
- 关键生成宏检查通过
- `main.c`、全部 `user_driver/*.c` 和生成的 `ti_msp_dl_config.c` 使用 TI ArmClang 5.1.1.LTS 对象编译通过

以上是静态生成与编译验证，不代表所有引脚、模块电压、左右轮方向和 PID 参数已经在当前实车上验证。
