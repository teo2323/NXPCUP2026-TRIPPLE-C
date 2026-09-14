################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/fsl_clock.c \
../drivers/fsl_common.c \
../drivers/fsl_common_arm.c \
../drivers/fsl_ctimer.c \
../drivers/fsl_edma.c \
../drivers/fsl_edma_soc.c \
../drivers/fsl_gpio.c \
../drivers/fsl_inputmux.c \
../drivers/fsl_lpflexcomm.c \
../drivers/fsl_lpi2c.c \
../drivers/fsl_lpi2c_cmsis.c \
../drivers/fsl_lpi2c_edma.c \
../drivers/fsl_lpuart.c \
../drivers/fsl_pwm.c \
../drivers/fsl_reset.c \
../drivers/fsl_spc.c 

C_DEPS += \
./drivers/fsl_clock.d \
./drivers/fsl_common.d \
./drivers/fsl_common_arm.d \
./drivers/fsl_ctimer.d \
./drivers/fsl_edma.d \
./drivers/fsl_edma_soc.d \
./drivers/fsl_gpio.d \
./drivers/fsl_inputmux.d \
./drivers/fsl_lpflexcomm.d \
./drivers/fsl_lpi2c.d \
./drivers/fsl_lpi2c_cmsis.d \
./drivers/fsl_lpi2c_edma.d \
./drivers/fsl_lpuart.d \
./drivers/fsl_pwm.d \
./drivers/fsl_reset.d \
./drivers/fsl_spc.d 

OBJS += \
./drivers/fsl_clock.o \
./drivers/fsl_common.o \
./drivers/fsl_common_arm.o \
./drivers/fsl_ctimer.o \
./drivers/fsl_edma.o \
./drivers/fsl_edma_soc.o \
./drivers/fsl_gpio.o \
./drivers/fsl_inputmux.o \
./drivers/fsl_lpflexcomm.o \
./drivers/fsl_lpi2c.o \
./drivers/fsl_lpi2c_cmsis.o \
./drivers/fsl_lpi2c_edma.o \
./drivers/fsl_lpuart.o \
./drivers/fsl_pwm.o \
./drivers/fsl_reset.o \
./drivers/fsl_spc.o 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o: ../drivers/%.c drivers/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MCXN947VDF -DCPU_MCXN947VDF_cm33 -DCPU_MCXN947VDF_cm33_core0 -DMCUXPRESSO_SDK -DSDK_DEBUGCONSOLE=0 -DMCUX_META_BUILD -DMCXN947_cm33_core0_SERIES -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\drivers" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS\m-profile" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\device\periph" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\str" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\utilities\debug_console_lite" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\component\uart" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\CMSIS_driver\Include" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\board" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source" -I"D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\include" -O0 -fno-common -g3 -gdwarf-4 -mcpu=cortex-m33 -c -ffunction-sections -fdata-sections -fno-builtin -imacros "D:\Facultate\FACULTATE\an2\sem2\robotica\bootcampWorkspace\nxpcup_official\source\mcux_config.h" -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m33 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-drivers

clean-drivers:
	-$(RM) ./drivers/fsl_clock.d ./drivers/fsl_clock.o ./drivers/fsl_common.d ./drivers/fsl_common.o ./drivers/fsl_common_arm.d ./drivers/fsl_common_arm.o ./drivers/fsl_ctimer.d ./drivers/fsl_ctimer.o ./drivers/fsl_edma.d ./drivers/fsl_edma.o ./drivers/fsl_edma_soc.d ./drivers/fsl_edma_soc.o ./drivers/fsl_gpio.d ./drivers/fsl_gpio.o ./drivers/fsl_inputmux.d ./drivers/fsl_inputmux.o ./drivers/fsl_lpflexcomm.d ./drivers/fsl_lpflexcomm.o ./drivers/fsl_lpi2c.d ./drivers/fsl_lpi2c.o ./drivers/fsl_lpi2c_cmsis.d ./drivers/fsl_lpi2c_cmsis.o ./drivers/fsl_lpi2c_edma.d ./drivers/fsl_lpi2c_edma.o ./drivers/fsl_lpuart.d ./drivers/fsl_lpuart.o ./drivers/fsl_pwm.d ./drivers/fsl_pwm.o ./drivers/fsl_reset.d ./drivers/fsl_reset.o ./drivers/fsl_spc.d ./drivers/fsl_spc.o

.PHONY: clean-drivers

