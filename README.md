# 11_PID_car

基于 TI MSPM0G3507 的双直流电机循迹小车工程。当前版本使用八路独立 GPIO 读取数字循迹信号，按离散规则生成两路目标轮速，再利用编码器反馈和增量式 PID 调节 TB6612FNG 的 A/B 两个电机通道。

> 当前“循迹方向决策”是规则控制，不是横向误差 PID；PID 只负责两个电机通道的速度闭环。

## GPIO 迁移确认

队友已经完成了从 I2C 循迹模块到普通 GPIO 循迹模块的代码迁移。Git 提交 `86135d4` 中可以看到：

- `empty.syscfg` 删除了名为 `HUIDU` 的 I2C1 实例及 PB2/PB3 的 SCL/SDA 配置。
- `empty.syscfg` 新增了名为 `XUNJI` 的 8 路 GPIO 输入组。
- `huidu_get_value()` 已不再发送 I2C 地址 `0x12` 和命令 `0x30`，而是直接读取 `L1~L4`、`R1~R4`。
- 原来的 PB6/PB7 按键被删除，这两个引脚现在用于 `XUNJI_L1/L2`。
- 电机 A 编码器从 PB13/PA22 改到了 PB8/PB9。

因此，本工程当前不是 I2C 循迹版本。商品型号未在仓库中注明，README 只按源码确认它是八路独立数字输出模块。

## 当前工程状态

| 功能 | 状态 | 当前实际行为 |
|---|---|---|
| 八路循迹输入 | 已改为 GPIO | 8 路输入、内部下拉，读取后统一按位取反 |
| 循迹决策 | 启用 | 10 ms 执行一次规则表，产生 A/B 通道目标速度 |
| 双路电机 PWM | 启用 | TIMG0 双通道，约 10 kHz，比较值范围 `0~4000` |
| 双编码器测速 | 启用 | 每个编码器只统计 A 相上升沿，10 ms 计算一次速度 |
| 双路增量式 PID | 启用 | 默认 `Kp=0.5`、`Ki=0.4`、`Kd=0.1` |
| UART 调试输出 | TX 启用 | PA10，115200 8N1，每 500 ms 输出一次八路逻辑值 |
| UART 接收 | 仅配置 | SysConfig 配置了 RX 中断，但 NVIC 和处理代码未启用 |
| ADC1 通道 1 | 仅初始化 | PA16、内部 2.5 V VREF，主循环没有读取结果 |
| 舵机 PWM | 仅初始化 | PA27、50 Hz，启动时比较值为 `50`，没有动态控制 |
| LED0/LED1 | 仅配置 | PA14/PA15，当前没有运行时控制 |
| 按键 | 已移除 | PB6/PB7 已让给循迹输入，当前没有开始/停止按键 |
| 干净重生成与编译 | 当前未通过 | 详见“构建与验证”中的两个旧宏问题 |

程序初始化后会立即启动 10 ms 电机控制定时器，没有独立的开始/停止状态机。首次上电调试必须架空车轮。

## 软件运行流程

初始化阶段：

1. `SYSCFG_DL_init()` 初始化 80 MHz 时钟、GPIO、PWM、定时器、UART、ADC 和 VREF。
2. 启用 GPIOB 中断，计划用于两个编码器 A 相计数。
3. 启用 ADC 转换，启动 TIMG7 舵机 PWM，并把比较值设为 `50`。
4. 把两路目标速度清零。
5. `motor_init(3)` 初始化 TB6612FNG A/B 通道，启动 TIMG0 电机 PWM 和 TIMA0 10 ms 控制定时器。

运行阶段包含两条并行路径：

- 主循环：读取八路 GPIO，拼成 `00011000` 形式的字符串，通过 UART0 每 500 ms 发送一次。
- TIMA0 中断：读取八路 GPIO → 更新两路目标速度 → 计算编码器速度 → 执行双路增量式 PID → 更新 PWM。

`delay_ms(500)` 只阻塞主循环；中断中的循迹、测速和 PID 仍按 10 ms 周期运行。

## 八路 GPIO 循迹模块

### 信号顺序与逻辑

源码把传感器按车体从左到右解释为：

```text
最左                                                     最右
L1      L2      L3      L4      R1      R2      R3      R4
v[0]    v[1]    v[2]    v[3]    v[4]    v[5]    v[6]    v[7]
```

每路 GPIO 在 SysConfig 中配置为数字输入和内部下拉。`huidu_get_value()` 读取原始电平后执行按位取反，因此：

| 引脚原始电平 | `huidu_value[]` | 控制代码含义 |
|---|---:|---|
| 低电平 | `1` | 有效/命中 |
| 高电平 | `0` | 无效/未命中 |

仓库没有循迹模块的型号和黑白电平说明，所以不能仅凭代码断定“黑线一定输出低电平”。上板前应分别把每个探头放到黑线和底色上，确认输出与上述软件逻辑一致；如果模块的有效电平相反，应统一修改取反逻辑，不能只交换打印含义。

### GPIO 接线

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| 循迹 L1 / 最左 | 八路数字循迹模块，具体型号未注明 | PB6 | PINCM23 | GPIO 输入 | 内部下拉，低电平记为 `1` |
| 循迹 L2 | 同上 | PB7 | PINCM24 | GPIO 输入 | 内部下拉，低电平记为 `1` |
| 循迹 L3 | 同上 | PA17 | PINCM39 | GPIO 输入 | 内部下拉，低电平记为 `1` |
| 循迹 L4 / 左中 | 同上 | PA18 | PINCM40 | GPIO 输入 | 内部下拉；PA18 兼有 BSL 相关功能，需核对实板 |
| 循迹 R1 / 右中 | 同上 | PA21 | PINCM46 | GPIO 输入 | 内部下拉；PA21 兼有 VREF- 相关功能，需核对实板 |
| 循迹 R2 | 同上 | PA22 | PINCM47 | GPIO 输入 | 内部下拉，低电平记为 `1` |
| 循迹 R3 | 同上 | PA24 | PINCM54 | GPIO 输入 | 内部下拉，低电平记为 `1` |
| 循迹 R4 / 最右 | 同上 | PA25 | PINCM55 | GPIO 输入 | 内部下拉，低电平记为 `1` |

IOMUX 索引来自使用 SysConfig 1.26.2 根据当前 `empty.syscfg` 重新生成的配置，不应以仓库中历史 `Debug/ti_msp_dl_config.h` 的旧索引为准。

### 循迹规则

`MIN_SPEED` 当前为 `150`。每次进入 `adjust_motor()` 时，代码先把两个电机方向都设为正转，再按下表从上到下匹配：

| 条件 | 电机 1 / A 通道目标速度 | 电机 2 / B 通道目标速度 |
|---|---:|---:|
| 8 路全为 `0` | `0` | `0` |
| 8 路全为 `1` | `0` | `0` |
| 只有中心 L4、R1 为 `1` | `150` | `150` |
| L4 为 `1`，且 L1~L3 为 `0` | `150` | `250` |
| L3 为 `1`，且 L1~L2 为 `0` | `150` | `300` |
| L2 为 `1`，且 L1 为 `0` | `150` | `350` |
| L1 为 `1` | `150` | `400` |
| R1 为 `1`，且 R2~R4 为 `0` | `250` | `150` |
| R2 为 `1`，且 R3~R4 为 `0` | `300` | `150` |
| R3 为 `1`，且 R4 为 `0` | `350` | `150` |
| R4 为 `1` | `400` | `150` |

多路同时命中时由 `if/else if` 的先后顺序决定结果。电机 A/B 通道分别安装在车体哪一侧没有在仓库中注明，必须按实车接线确认；如果转向与规则相反，应先核对左右轮和电机方向，不能直接把规则表当成绝对左右关系。

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
PWM(k) = clamp(PWM(k-1) + ΔPWM, 0, 4000)
```

两个编码器的 B 相都只配置为普通输入，没有参与计数或正交方向判断。因此当前只能统计 A 相上升沿数量，不能得到有符号速度。

## 当前 SysConfig 引脚总表

下表以 `empty.syscfg` 和使用 SysConfig 1.26.2 干净生成的配置为准。

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| 电机 A PWM / PWMA | TB6612FNG | PA12 | PINCM34 | TIMG0_CCP0 | 约 10 kHz |
| 电机 A 方向 AIN1 | TB6612FNG | PA9 | PINCM20 | GPIO 输出 | 以 SysConfig 宏命名为准 |
| 电机 A 方向 AIN2 | TB6612FNG | PA8 | PINCM19 | GPIO 输出 | 以 SysConfig 宏命名为准 |
| 电机 B PWM / PWMB | TB6612FNG | PA13 | PINCM35 | TIMG0_CCP1 | 约 10 kHz |
| 电机 B 方向 BIN1 | TB6612FNG | PA7 | PINCM14 | GPIO 输出 | 以 SysConfig 宏命名为准 |
| 电机 B 方向 BIN2 | TB6612FNG | PB18 | PINCM44 | GPIO 输出 | 以 SysConfig 宏命名为准 |
| 电机驱动使能 STBY | TB6612FNG | PB24 | PINCM52 | GPIO 输出 | A/B 通道共用 |
| 电机 A 编码器 A 相 | 带编码器直流电机，具体型号未注明 | PB8 | PINCM25 | GPIO 输入/上升沿中断 | 参与计数 |
| 电机 A 编码器 B 相 | 同上 | PB9 | PINCM26 | GPIO 输入 | 当前未参与方向判断 |
| 电机 B 编码器 A 相 | 同上 | PB19 | PINCM45 | GPIO 输入/上升沿中断 | 参与计数 |
| 电机 B 编码器 B 相 | 同上 | PB20 | PINCM48 | GPIO 输入 | 无内部上下拉，当前不参与方向判断 |
| 调试串口 TX | 外接 3.3 V USB-UART | PA10 | PINCM21 | UART0_TX | 115200 8N1 |
| 调试串口 RX | 外接 3.3 V USB-UART | PA11 | PINCM22 | UART0_RX | 当前接收处理未启用 |
| 舵机 PWM | 具体型号未注明 | PA27 | PINCM60 | TIMG7_CCP1 | 50 Hz，当前无动态控制 |
| ADC 输入 | 外部模拟信号，具体模块未注明 | PA16 | PINCM38 | ADC1 通道 1 | 12 bit，内部 2.5 V 参考 |
| LED0 | 外接 LED，具体型号未注明 | PA14 | PINCM36 | GPIO 输出 | 当前未驱动 |
| LED1 | 外接 LED，具体型号未注明 | PA15 | PINCM37 | GPIO 输出 | 当前未驱动 |
| SWDIO | SEGGER J-Link | PA19 | 专用调试引脚 | SWDIO | 不得复用 |
| SWCLK | SEGGER J-Link | PA20 | 专用调试引脚 | SWCLK | 不得复用 |
| HFXIN | 40 MHz HFXT 时钟节点 | PA5 | PINCM10 | HFXIN | 时钟专用，不得改作普通 GPIO |
| HFXOUT | 40 MHz HFXT 时钟节点 | PA6 | PINCM11 | HFXOUT | 时钟专用，不得改作普通 GPIO |

八路循迹引脚见上一节的独立接线表。

## 电源与电平

- MSPM0G3507 GPIO 为 3.3 V 电平，任何传感器输出都不能直接向 MCU 输入 5 V。
- 循迹模块型号和供电范围未在仓库中注明。若模块使用 5 V 供电，必须确认 8 路数字输出的高电平不超过 3.3 V，必要时使用电平转换。
- TB6612FNG 逻辑电源按源码注释使用 3.3 V，电机电源 VM 为 7.4 V。
- MCU、循迹模块、TB6612FNG、编码器和外接 USB-UART 必须共地。
- 当前调试 UART 使用 PA10/PA11，需要外接 3.3 V USB-UART，不是地猛星板载 CH340 默认使用的 PA0/PA1。
- 当前 XUNJI_L4 使用 PA18。PA18 与 BSL/按键资源有关，使用 J-Link 调试时仍应确认地猛星实板没有外部电路干扰该输入。

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

### CCS Theia

1. 安装 MSPM0 SDK 2.10.00.04 和 SysConfig 1.26.2。
2. 导入 `11_PID_car` 工程。
3. 打开 `empty.syscfg`，核对器件、LQFP-64 封装和全部实际接线。
4. Clean/Rebuild，让 CCS 在本机重新生成配置和对象文件。
5. 使用 `targetConfigs/MSPM0G3507.ccxml` 和 SEGGER J-Link 调试或烧录。

不要把仓库中的历史 `Debug/ti_msp_dl_config.h` 当作干净生成结果。

### 当前验证结果

2026-07-27 在 macOS 上使用 MSPM0 SDK 2.10.00.04、SysConfig 1.26.2 和 TI ArmClang 5.1.1.LTS 检查：

- 当前 `empty.syscfg` 的 SysConfig 生成成功，确认 I2C1 已删除，8 路 `XUNJI` GPIO 可以生成。
- 直接运行 `make check` 会在宏检查阶段失败，因为 `Makefile` 仍要求旧 I2C 宏 `HUIDU_INST`。
- 临时把宏检查项改成 `XUNJI_*` 后，编译 `main.c` 又会失败：源码使用历史生成宏 `GPIO_MULTIPLE_GPIOB_INT_IRQN`，而当前 SysConfig 实际生成的是 `DC_MOTOR_INT_IRQN`。

因此，当前提交不能声明“干净重生成和对象编译均通过”。仓库里的历史 `Debug` 文件含有旧的兼容宏，可能掩盖这个问题。后续修复时应同步：

1. 把 `main.c` 的 GPIOB 中断 IRQ 宏改为当前 SysConfig 生成的 `DC_MOTOR_INT_IRQN`。
2. 把 `Makefile` 的 `HUIDU_INST` 检查替换为实际使用的 `XUNJI_L1~R4` 端口/引脚宏。
3. 重新执行 SysConfig 生成、关键宏检查和所有 `.c` 文件对象编译，全部 0 报错后再烧录。

## 串口调试输出

连接外部 3.3 V USB-UART：

| 外设功能 | 芯片/模块型号 | 地猛星引脚 | IOMUX 索引 | 片上复用功能 | 备注 |
|---|---|---|---|---|---|
| USB-UART RX ← MCU TX | 3.3 V USB-UART，具体型号未注明 | PA10 | PINCM21 | UART0_TX | 115200 8N1 |
| USB-UART TX → MCU RX | 同上 | PA11 | PINCM22 | UART0_RX | 当前程序不处理接收 |
| 公共地 | 同上 | GND | 不适用 | GND | 必须共地 |

主循环每 500 ms 输出一行 8 个数字，例如：

```text
00011000
```

数字顺序是 `L1 L2 L3 L4 R1 R2 R3 R4`；其中 `1` 表示对应 GPIO 原始电平为低，`0` 表示原始电平为高。

## 上板前检查

1. 逐路确认 L1~L4、R1~R4 的物理顺序与 GPIO 接线一致。
2. 实测黑线和底色对应的高低电平，确认软件中的按位取反符合实际模块。
3. 确认循迹模块输出电平不超过 3.3 V，并让所有模块共地。
4. 核对 TB6612FNG 的 PWMA/PWMB、AIN1/AIN2、BIN1/BIN2 以及 A/B 通道对应的左右车轮。
5. 核对编码器 A/B 相；当前 A 通道已经改为 PB8/PB9，不再是 PB13/PA22。
6. 确认 PA18 没有被实板 BSL/按键电路占用或拉偏。
7. 确认 PA10/PA11 使用外接 3.3 V USB-UART。
8. 确认 PA19/PA20 只用于 J-Link SWD。
9. 修复并通过“SysConfig 生成 → 宏检查 → 对象编译”后再烧录。
10. 第一次运行时架空车轮，因为电机控制定时器会在初始化后立即启动。

## 已知限制

- 当前干净编译被两个旧宏问题阻塞，不能仅凭仓库中的 `.out` 或历史 `Debug` 文件判断工程可重新构建。
- 循迹模块的具体型号、供电范围以及黑/白对应电平没有记录，必须实测校准。
- 循迹使用离散规则而不是连续误差算法；多探头同时命中时，结果依赖 `if/else if` 顺序。
- 两路全 `0` 或全 `1` 都会把目标速度设为 `0`。
- 没有开始/停止按键，上电后控制定时器立即运行。
- 编码器只统计 A 相上升沿，无法判断方向，也没有利用 B 相提高分辨率。
- 电机 A/B 通道与车体左右轮的对应关系没有记录。
- PID 参数、轮径、编码器计数常量和目标速度均写死在源码中，没有在线调参或掉电保存。
- UART RX、LED、ADC 结果和舵机动态控制尚未接入主运行流程。
- `motor.h` 的部分方向引脚注释可能与 SysConfig 命名顺序不同，维护时应以 `empty.syscfg` 生成宏为准。
- SysConfig 同时配置了 PA5/PA6 的 40 MHz HFXT 和 80 MHz CPU 时钟，修改时钟树前必须重新验证生成结果。
