SHELL := /bin/zsh

# 本机 TI 工具链路径；若以后升级 SDK/CCS，可在命令行覆盖这些变量。
SDK_DIR ?= /Users/rylum/ti/mspm0_sdk_2_10_00_04
SYSCONFIG_CLI ?= /Users/rylum/ti/sysconfig_1.26.2/sysconfig_cli.sh
TIARMCLANG ?= /Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang
TIARMCLANG_INCLUDE ?= /Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/include/c

# 验证产物写入临时目录，避免改动 CCS 自动生成的 Debug 构建文件。
PROJECT_ROOT := $(CURDIR)
SYSCONFIG_FILE := $(PROJECT_ROOT)/empty.syscfg
CHECK_ROOT ?= /tmp/11_PID_car_check
SYSCONFIG_OUT := $(CHECK_ROOT)/syscfg
OBJECT_OUT := $(CHECK_ROOT)/objects

# 自动收集工程内的应用源文件，确保新增驱动也会进入对象编译检查。
PROJECT_SOURCES := main.c $(sort $(wildcard user_driver/*.c))
REQUIRED_GENERATED_MACROS := \
	DEBUG_INST \
	GPIO_PWMAB_C0_IDX \
	GPIO_PWMAB_C1_IDX \
	GPIO_SERVO_C1_IDX \
	HUIDU_INST \
	MOTOR_PID_INST \
	PWMAB_INST \
	SERVO_INST

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
