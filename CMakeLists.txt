# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

if (DEFINED ENV{SdkRootDirPath} AND NOT "$ENV{SdkRootDirPath}" STREQUAL "")
	SET(SdkRootDirPath "$ENV{SdkRootDirPath}")
else()
	SET(SdkRootDirPath "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

# RESET FLAG VALUES
SET(CMAKE_ASM_FLAGS_DEBUG "")
SET(CMAKE_ASM_FLAGS_RELEASE "")
SET(CMAKE_C_FLAGS_DEBUG "")
SET(CMAKE_C_FLAGS_RELEASE "")
SET(CMAKE_CXX_FLAGS_DEBUG "")
SET(CMAKE_CXX_FLAGS_RELEASE "")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "")

# CURRENT DIRECTORY
SET(ProjDirPath .)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_USER_MAKE_RULES_OVERRIDE "user_override.cmake")

project(nxpcup_official)
set(MCUX_SDK_PROJECT_NAME nxpcup_official)

include(flags.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} "CMSIS/tz_context.h"
"CMSIS/core_cm33.h"
"CMSIS/cmsis_version.h"
"CMSIS/cmsis_gcc.h"
"CMSIS/cmsis_compiler.h"
"CMSIS/m-profile/cmsis_gcc_m.h"
"CMSIS/m-profile/armv8m_mpu.h"
"CMSIS_driver/Include/Driver_I2C.h"
"CMSIS_driver/Include/Driver_Common.h"
"board/RTE_Device.h"
"board/pin_mux.h"
"board/pin_mux.c"
"board/peripherals.h"
"board/peripherals.c"
"board/hardware_init.c"
"board/clock_config.h"
"board/clock_config.c"
"board/board.h"
"board/board.c"
"board/app.h"
"component/uart/fsl_adapter_uart.h"
"component/uart/fsl_adapter_lpuart.c"
"device/system_MCXN947_cm33_core0.h"
"device/system_MCXN947_cm33_core0.c"
"device/MCXN947_cm33_core0_features.h"
"device/MCXN947_cm33_core0_COMMON.h"
"device/MCXN947_cm33_core0.h"
"device/fsl_device_registers.h"
"device/periph/PERI_WWDT.h"
"device/periph/PERI_WUU.h"
"device/periph/PERI_VREF.h"
"device/periph/PERI_VBAT.h"
"device/periph/PERI_UTICK.h"
"device/periph/PERI_USDHC.h"
"device/periph/PERI_USBPHY.h"
"device/periph/PERI_USBNC.h"
"device/periph/PERI_USBHSDCD.h"
"device/periph/PERI_USBHS.h"
"device/periph/PERI_USBDCD.h"
"device/periph/PERI_USB.h"
"device/periph/PERI_TSI.h"
"device/periph/PERI_TRDC.h"
"device/periph/PERI_SYSPM.h"
"device/periph/PERI_SYSCON.h"
"device/periph/PERI_SPC.h"
"device/periph/PERI_SMARTDMA.h"
"device/periph/PERI_SINC.h"
"device/periph/PERI_SEMA42.h"
"device/periph/PERI_SCT.h"
"device/periph/PERI_SCG.h"
"device/periph/PERI_S50.h"
"device/periph/PERI_RTC.h"
"device/periph/PERI_QDC.h"
"device/periph/PERI_PWM.h"
"device/periph/PERI_PUF.h"
"device/periph/PERI_POWERQUAD.h"
"device/periph/PERI_PORT.h"
"device/periph/PERI_PLU.h"
"device/periph/PERI_PKC.h"
"device/periph/PERI_PINT.h"
"device/periph/PERI_PDM.h"
"device/periph/PERI_OTPC.h"
"device/periph/PERI_OSTIMER.h"
"device/periph/PERI_OPAMP.h"
"device/periph/PERI_NPX.h"
"device/periph/PERI_MRT.h"
"device/periph/PERI_MAILBOX.h"
"device/periph/PERI_LP_FLEXCOMM.h"
"device/periph/PERI_LPUART.h"
"device/periph/PERI_LPTMR.h"
"device/periph/PERI_LPSPI.h"
"device/periph/PERI_LPI2C.h"
"device/periph/PERI_LPDAC.h"
"device/periph/PERI_LPCMP.h"
"device/periph/PERI_ITRC.h"
"device/periph/PERI_INTM.h"
"device/periph/PERI_INPUTMUX.h"
"device/periph/PERI_I3C.h"
"device/periph/PERI_I2S.h"
"device/periph/PERI_HPDAC.h"
"device/periph/PERI_GPIO.h"
"device/periph/PERI_GDET.h"
"device/periph/PERI_FREQME.h"
"device/periph/PERI_FMUTEST.h"
"device/periph/PERI_FMU.h"
"device/periph/PERI_FLEXSPI.h"
"device/periph/PERI_FLEXIO.h"
"device/periph/PERI_EWM.h"
"device/periph/PERI_EVTG.h"
"device/periph/PERI_ERM.h"
"device/periph/PERI_ENET.h"
"device/periph/PERI_EMVSIM.h"
"device/periph/PERI_EIM.h"
"device/periph/PERI_DMA.h"
"device/periph/PERI_DM.h"
"device/periph/PERI_DIGTMP.h"
"device/periph/PERI_CTIMER.h"
"device/periph/PERI_CRC.h"
"device/periph/PERI_CMC.h"
"device/periph/PERI_CDOG.h"
"device/periph/PERI_CAN.h"
"device/periph/PERI_CACHE64_POLSEL.h"
"device/periph/PERI_CACHE64_CTRL.h"
"device/periph/PERI_BSP32.h"
"device/periph/PERI_AHBSC.h"
"device/periph/PERI_ADC.h"
"drivers/fsl_spc.h"
"drivers/fsl_spc.c"
"drivers/fsl_reset.h"
"drivers/fsl_reset.c"
"drivers/fsl_pwm.h"
"drivers/fsl_pwm.c"
"drivers/fsl_port.h"
"drivers/fsl_lpuart.h"
"drivers/fsl_lpuart.c"
"drivers/fsl_lpi2c_edma.h"
"drivers/fsl_lpi2c_edma.c"
"drivers/fsl_lpi2c_cmsis.h"
"drivers/fsl_lpi2c_cmsis.c"
"drivers/fsl_lpi2c.h"
"drivers/fsl_lpi2c.c"
"drivers/fsl_lpflexcomm.h"
"drivers/fsl_lpflexcomm.c"
"drivers/fsl_inputmux_connections.h"
"drivers/fsl_inputmux.h"
"drivers/fsl_inputmux.c"
"drivers/fsl_gpio.h"
"drivers/fsl_gpio.c"
"drivers/fsl_edma_soc.h"
"drivers/fsl_edma_soc.c"
"drivers/fsl_edma_core.h"
"drivers/fsl_edma.h"
"drivers/fsl_edma.c"
"drivers/fsl_ctimer.h"
"drivers/fsl_ctimer.c"
"drivers/fsl_common_arm.h"
"drivers/fsl_common_arm.c"
"drivers/fsl_common.h"
"drivers/fsl_common.c"
"drivers/fsl_clock.h"
"drivers/fsl_clock.c"
"source/simple_movement.c"
"source/servo.c"
"source/semihost_hardfault.c"
"source/pixy.c"
"source/mcux_config.h"
"source/main.c"
"source/hbridge.c"
"source/esc.c"
"source/detection.c"
"source/wifi.c"
"startup/startup_mcxn947_cm33_core0.c"
"startup/boot_multicore_slave.h"
"startup/boot_multicore_slave.c"
"utilities/fsl_memcpy.S"
"utilities/fsl_assert.h"
"utilities/fsl_assert.c"
"utilities/str/fsl_str.h"
"utilities/str/fsl_str.c"
"utilities/debug_console_lite/fsl_debug_console.h"
"utilities/debug_console_lite/fsl_debug_console.c"
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES SUFFIX ".axf")
endif()

if(${LIBRARY_TYPE} STREQUAL "REDLIB")
    target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TOOLCHAIN_DIR}/redlib/include)
endif()

set(asm_include_directories "${CMAKE_CURRENT_SOURCE_DIR}/drivers"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/m-profile"
"${CMAKE_CURRENT_SOURCE_DIR}/device"
"${CMAKE_CURRENT_SOURCE_DIR}/device/periph"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities/str"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities/debug_console_lite"
"${CMAKE_CURRENT_SOURCE_DIR}/component/uart"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_driver/Include"
"${CMAKE_CURRENT_SOURCE_DIR}/board"
"${CMAKE_CURRENT_SOURCE_DIR}/source"
"${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:ASM>:${asm_include_directories}>)

set(c_include_directories "${CMAKE_CURRENT_SOURCE_DIR}/drivers"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/m-profile"
"${CMAKE_CURRENT_SOURCE_DIR}/device"
"${CMAKE_CURRENT_SOURCE_DIR}/device/periph"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities/str"
"${CMAKE_CURRENT_SOURCE_DIR}/utilities/debug_console_lite"
"${CMAKE_CURRENT_SOURCE_DIR}/component/uart"
"${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_driver/Include"
"${CMAKE_CURRENT_SOURCE_DIR}/board"
"${CMAKE_CURRENT_SOURCE_DIR}/source"
"${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:C>:${c_include_directories}>)

set(cpp_include_directories "")

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${cpp_include_directories}>)

set(include_files "")

target_precompile_headers(${MCUX_SDK_PROJECT_NAME} PRIVATE ${include_files})

set(link_directories "")

set(link_libraries "")

target_link_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE ${link_directories})
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES} ${link_libraries})
target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	add_custom_command(
		TARGET ${MCUX_SDK_PROJECT_NAME}
		POST_BUILD
		COMMAND "${TOOLCHAIN_DIR}/bin/arm-none-eabi-size" "$<TARGET_FILE_NAME:${MCUX_SDK_PROJECT_NAME}>"
	)
endif()

