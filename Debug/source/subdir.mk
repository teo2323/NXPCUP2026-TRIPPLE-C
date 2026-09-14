################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/detection.c \
../source/esc.c \
../source/hbridge.c \
../source/main.c \
../source/pixy.c \
../source/semihost_hardfault.c \
../source/servo.c \
../source/simple_movement.c 

C_DEPS += \
./source/detection.d \
./source/esc.d \
./source/hbridge.d \
./source/main.d \
./source/pixy.d \
./source/semihost_hardfault.d \
./source/servo.d \
./source/simple_movement.d 

OBJS += \
./source/detection.o \
./source/esc.o \
./source/hbridge.o \
./source/main.o \
./source/pixy.o \
./source/semihost_hardfault.o \
./source/servo.o \
./source/simple_movement.o 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c source/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DMCUX_META_BUILD -DMCXN947_cm33_core0_SERIES -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\drivers" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS\m-profile" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device\periph" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\str" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\debug_console_lite" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\component\uart" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS_driver\Include" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\board" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\include" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-source

clean-source:
	-$(RM) ./source/detection.d ./source/detection.o ./source/esc.d ./source/esc.o ./source/hbridge.d ./source/hbridge.o ./source/main.d ./source/main.o ./source/pixy.d ./source/pixy.o ./source/semihost_hardfault.d ./source/semihost_hardfault.o ./source/servo.d ./source/servo.o ./source/simple_movement.d ./source/simple_movement.o

.PHONY: clean-source

