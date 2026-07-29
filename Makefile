SHELL := /bin/zsh

# 本机 TI 工具链路径；若以后升级 SDK/CCS，可在命令行覆盖这些变量。
SDK_DIR ?= /Users/rylum/ti/mspm0_sdk_2_10_00_04
SYSCONFIG_CLI ?= /Users/rylum/ti/sysconfig_1.26.2/sysconfig_cli.sh
TIARMCLANG ?= /Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang
TIARMCLANG_INCLUDE ?= /Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/include/c

# 验证产物写入临时目录，避免改动 CCS 自动生成的 Debug 构建文件。
PROJECT_ROOT := $(CURDIR)
SYSCONFIG_FILE := $(PROJECT_ROOT)/empty.syscfg
CHECK_ROOT ?= /tmp/12_Encoder_Line_Car_check
SYSCONFIG_OUT := $(CHECK_ROOT)/syscfg
OBJECT_OUT := $(CHECK_ROOT)/objects

# 自动收集工程内的应用源文件，确保新增驱动也会进入对象编译检查。
PROJECT_SOURCES := main.c $(sort $(wildcard user_driver/*.c))
REQUIRED_GENERATED_MACROS := \
	DEBUG_INST \
	K230_LINK_INST \
	EXPANSION_I2C_INST \
	START_KEY_PORT \
	START_KEY_KEY_PIN \
	DC_MOTOR_INT_IRQN \
	GPIO_PWMAB_C0_IDX \
	GPIO_PWMAB_C1_IDX \
	GPIO_SERVO_C1_IDX \
	CONTROL_LOOP_INST \
	PWMAB_INST \
	SERVO_INST \
	XUNJI_L1_PORT \
	XUNJI_L1_PIN \
	XUNJI_L2_PORT \
	XUNJI_L2_PIN \
	XUNJI_L3_PORT \
	XUNJI_L3_PIN \
	XUNJI_L4_PORT \
	XUNJI_L4_PIN \
	XUNJI_R1_PORT \
	XUNJI_R1_PIN \
	XUNJI_R2_PORT \
	XUNJI_R2_PIN \
	XUNJI_R3_PORT \
	XUNJI_R3_PIN \
	XUNJI_R4_PORT \
	XUNJI_R4_PIN

# “电赛小车底板 V1.0”的关键引脚必须与 SysConfig 生成结果完全一致。
# 这组检查用于阻止以后在 GUI 中误改引脚后仍然“能编译但接线不匹配”。
EXPECTED_GENERATED_DEFINES := \
	DEBUG_INST:UART0 \
	GPIO_DEBUG_TX_PORT:GPIOA \
	GPIO_DEBUG_TX_PIN:DL_GPIO_PIN_28 \
	GPIO_DEBUG_RX_PORT:GPIOA \
	GPIO_DEBUG_RX_PIN:DL_GPIO_PIN_31 \
	DEBUG_BAUD_RATE:115200 \
	K230_LINK_INST:UART3 \
	GPIO_K230_LINK_TX_PORT:GPIOA \
	GPIO_K230_LINK_TX_PIN:DL_GPIO_PIN_26 \
	GPIO_K230_LINK_RX_PORT:GPIOA \
	GPIO_K230_LINK_RX_PIN:DL_GPIO_PIN_25 \
	K230_LINK_BAUD_RATE:9600 \
	EXPANSION_I2C_INST:I2C1 \
	GPIO_EXPANSION_I2C_SCL_PORT:GPIOB \
	GPIO_EXPANSION_I2C_SCL_PIN:DL_GPIO_PIN_2 \
	GPIO_EXPANSION_I2C_SDA_PORT:GPIOB \
	GPIO_EXPANSION_I2C_SDA_PIN:DL_GPIO_PIN_3 \
	GPIO_PWMAB_C0_PORT:GPIOA \
	GPIO_PWMAB_C0_PIN:DL_GPIO_PIN_12 \
	GPIO_PWMAB_C1_PORT:GPIOA \
	GPIO_PWMAB_C1_PIN:DL_GPIO_PIN_13 \
	DC_MOTOR_AIN1_PORT:GPIOA \
	DC_MOTOR_AIN1_PIN:DL_GPIO_PIN_9 \
	DC_MOTOR_AIN2_PORT:GPIOA \
	DC_MOTOR_AIN2_PIN:DL_GPIO_PIN_8 \
	DC_MOTOR_BIN1_PORT:GPIOA \
	DC_MOTOR_BIN1_PIN:DL_GPIO_PIN_7 \
	DC_MOTOR_BIN2_PORT:GPIOB \
	DC_MOTOR_BIN2_PIN:DL_GPIO_PIN_18 \
	DC_MOTOR_STBY_PORT:GPIOB \
	DC_MOTOR_STBY_PIN:DL_GPIO_PIN_24 \
	DC_MOTOR_AA_PORT:GPIOB \
	DC_MOTOR_AA_PIN:DL_GPIO_PIN_8 \
	DC_MOTOR_AB_PORT:GPIOB \
	DC_MOTOR_AB_PIN:DL_GPIO_PIN_9 \
	DC_MOTOR_BA_PORT:GPIOB \
	DC_MOTOR_BA_PIN:DL_GPIO_PIN_19 \
	DC_MOTOR_BB_PORT:GPIOB \
	DC_MOTOR_BB_PIN:DL_GPIO_PIN_20 \
	GPIO_SERVO_C1_PORT:GPIOA \
	GPIO_SERVO_C1_PIN:DL_GPIO_PIN_27 \
	XUNJI_L1_PORT:GPIOA \
	XUNJI_L1_PIN:DL_GPIO_PIN_18 \
	XUNJI_L2_PORT:GPIOA \
	XUNJI_L2_PIN:DL_GPIO_PIN_16 \
	XUNJI_L3_PORT:GPIOB \
	XUNJI_L3_PIN:DL_GPIO_PIN_7 \
	XUNJI_L4_PORT:GPIOA \
	XUNJI_L4_PIN:DL_GPIO_PIN_17 \
	XUNJI_R1_PORT:GPIOA \
	XUNJI_R1_PIN:DL_GPIO_PIN_21 \
	XUNJI_R2_PORT:GPIOA \
	XUNJI_R2_PIN:DL_GPIO_PIN_22 \
	XUNJI_R3_PORT:GPIOA \
	XUNJI_R3_PIN:DL_GPIO_PIN_24 \
	XUNJI_R4_PORT:GPIOA \
	XUNJI_R4_PIN:DL_GPIO_PIN_2 \
	START_KEY_PORT:GPIOB \
	START_KEY_KEY_PIN:DL_GPIO_PIN_6

.PHONY: check check-paths

# 先检查工具链路径，错误时给出明确提示，而不是继续产生难读的编译错误。
check-paths:
	@set -eu; \
	for required_path in \
		"$(SDK_DIR)/.metadata/product.json" \
		"$(SDK_DIR)/source" \
		"$(SDK_DIR)/source/third_party/CMSIS/Core/Include" \
		"$(SYSCONFIG_CLI)" \
		"$(TIARMCLANG)" \
		"$(TIARMCLANG_INCLUDE)" \
		"$(SYSCONFIG_FILE)"; do \
		if [[ ! -e "$$required_path" ]]; then \
			echo "缺少依赖路径: $$required_path"; \
			exit 1; \
		fi; \
	done

# 完整验证顺序：SysConfig 生成 → 生成宏检查 → 所有 C 文件对象编译。
check: check-paths
	@mkdir -p "$(SYSCONFIG_OUT)" "$(OBJECT_OUT)"
	@echo "1/3 运行 SysConfig..."
	@"$(SYSCONFIG_CLI)" \
		-s "$(SDK_DIR)/.metadata/product.json" \
		--script "$(SYSCONFIG_FILE)" \
		--output "$(SYSCONFIG_OUT)"
	@echo "2/3 检查代码依赖的 SysConfig 宏..."
	@set -eu; \
	for macro_name in $(REQUIRED_GENERATED_MACROS); do \
		if ! grep -Eq "^[[:space:]]*#define[[:space:]]+$$macro_name([[:space:]]|$$)" \
			"$(SYSCONFIG_OUT)/ti_msp_dl_config.h"; then \
			echo "生成配置中缺少宏: $$macro_name"; \
			exit 1; \
		fi; \
	done; \
	for expected_define in $(EXPECTED_GENERATED_DEFINES); do \
		macro_name=$${expected_define%%:*}; \
		expected_value=$${expected_define#*:}; \
		actual_value=$$(awk -v name="$$macro_name" \
			'$$1 == "#define" && $$2 == name { gsub(/[()]/, "", $$3); print $$3; exit }' \
			"$(SYSCONFIG_OUT)/ti_msp_dl_config.h"); \
		if [[ "$$actual_value" != "$$expected_value" ]]; then \
			echo "生成配置引脚不匹配: $$macro_name=$$actual_value，期望 $$expected_value"; \
			exit 1; \
		fi; \
	done
	@echo "3/3 使用 TI ArmClang 逐文件对象编译..."
	@set -eu; \
	for source_file in $(PROJECT_SOURCES) "$(SYSCONFIG_OUT)/ti_msp_dl_config.c"; do \
		object_name=$$(basename "$${source_file%.c}").o; \
		echo "  编译 $$source_file"; \
		"$(TIARMCLANG)" \
			-c @"$(SYSCONFIG_OUT)/device.opt" \
			-mcpu=cortex-m0plus \
			-march=thumbv6m \
			-mthumb \
			-mfloat-abi=soft \
			-std=c11 \
			-Wall \
			-I"$(PROJECT_ROOT)" \
			-I"$(PROJECT_ROOT)/user_driver" \
			-I"$(SYSCONFIG_OUT)" \
			-I"$(SDK_DIR)/source" \
			-I"$(SDK_DIR)/source/third_party/CMSIS/Core/Include" \
			-I"$(TIARMCLANG_INCLUDE)" \
			"$$source_file" \
			-o "$(OBJECT_OUT)/$$object_name"; \
	done
	@echo "检查通过：SysConfig 生成、宏名检查和对象编译均为 0 报错。"
