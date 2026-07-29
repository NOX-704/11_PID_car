################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
user_driver/%.o: ../user_driver/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"/Users/rylum/ti/ti_cgt_arm_llvm_4.0.2.LTS/bin/tiarmclang" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/user_driver" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car" -I"/Users/rylum/workspace_ccstheia/12_Encoder_Line_Car/Debug" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"/Users/rylum/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"user_driver/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


