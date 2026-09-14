################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../utilities/fsl_assert.c 

S_UPPER_SRCS += \
../utilities/fsl_memcpy.S 

C_DEPS += \
./utilities/fsl_assert.d 

OBJS += \
./utilities/fsl_assert.o \
./utilities/fsl_memcpy.o 


# Each subdirectory must supply rules for building sources it contributes
utilities/%.o: ../utilities/%.c utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DMCUX_META_BUILD -DMCXN947_cm33_core0_SERIES -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\drivers" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS\m-profile" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device\periph" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\str" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\debug_console_lite" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\component\uart" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS_driver\Include" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\board" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\include" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

utilities/%.o: ../utilities/%.S utilities/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU Assembler'
	arm-none-eabi-gcc -c -x assembler-with-cpp -D__REDLIB__ -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\drivers" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS\m-profile" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device\periph" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\str" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\debug_console_lite" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\component\uart" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS_driver\Include" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\board" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\include" -g3 -gdwarf-4 -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-utilities

clean-utilities:
	-$(RM) ./utilities/fsl_assert.d ./utilities/fsl_assert.o ./utilities/fsl_memcpy.o

.PHONY: clean-utilities

