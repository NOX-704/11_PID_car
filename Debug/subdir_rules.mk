################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-681305811: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"/Users/rylum/ti/sysconfig_1.26.2/sysconfig_cli.sh" -s "/Users/rylum/ti/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-681305811 ../empty.syscfg
device.opt: build-681305811
device.cmd.genlibs: build-681305811
ti_msp_dl_config.c: build-681305811
ti_msp_dl_config.h: build-681305811
Event.dot: build-681305811

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"/Users/rylum/ti/ti_cgt_arm_llvm_4.0.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/user_driver" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/Debug" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: /Users/rylum/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"/Users/rylum/ti/ti_cgt_arm_llvm_4.0.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/user_driver" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/Debug" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"/Users/rylum/ti/ti_cgt_arm_llvm_4.0.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/user_driver" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/Debug" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


