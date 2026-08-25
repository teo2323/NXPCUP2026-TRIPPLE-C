#include "ultrasonic.h"
#include "fsl_gpio.h" // GPIO functions such as GPIO_PinWrite and GPIO_PinRead
#include "fsl_clock.h" // functions for clock configuration, if needed
#include "fsl_common.h" 
#include "pin_mux.h"

// import the frequency of the proccesor clock defined in SDK 
extern uint32_t SystemCoreClock;

void Ultrasonic_Init(void)
{
    // enable internal CPU hardware cycle counter for accurate timing
    MSDK_EnableCpuCycleCounter();
    // set the Trigger pin as output and initialize it to LOW
    // GPIO_PinWrite(PORT, PIN, 0U) 
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
}

float Ultrasonic_ReadDistanceCm(void)
{
    // 1 secound = 1,000,000 microseconds
    // nr of beats of the CPU clock in 1 microsecond
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 150U; 
    }

    // Read the iniital state of the Echo pin 
    uint32_t initial_echo = GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN);

    // Write Trigger: LOW -> delay of 2 microseconds -> HIGH -> delay of 10 microseconds -> LOW
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
    SDK_DelayAtLeastUs(2U, SystemCoreClock);
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 1U);
    SDK_DelayAtLeastUs(10U, SystemCoreClock);
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
    // Echo becomes 1 when the Trigger signal finishes (falling edge: 1 -> 0)

    // we capture the start time in CPU cycles and wait for the Echo pin to go HIGH 
    uint32_t start_cycles = MSDK_GetCpuCycleCount();
    uint32_t max_wait_cycles = 10000U * cycles_per_us; 

    while (GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN) == 0U)
    {
        if ((MSDK_GetCpuCycleCount() - start_cycles) > max_wait_cycles)
        {
            if (initial_echo != 0U) {
                return -4.0f; // the initial state of the Echo pin was HIGH  before the Trigger pulse
            }
            return -1.0f; // the Echo pin didn't go in HIGH state after the Trigger pulse
        }
    }

    // we capture the time when the Echo pin goes HIGH
    uint32_t echo_start_cycles = MSDK_GetCpuCycleCount();
    uint32_t max_echo_cycles = 30000U * cycles_per_us; // ~30ms max (~5m)

    while (GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN) != 0U)
    {
        if ((MSDK_GetCpuCycleCount() - echo_start_cycles) > max_echo_cycles)
        {
            return -2.0f; // The Echo pin remain HIGH for too long (stuck HIGH / out of range)
        }
    }

    // we capture the time when the Echo pin goes LOW
    uint32_t echo_end_cycles = MSDK_GetCpuCycleCount();
    uint32_t echo_duration_us = (echo_end_cycles - echo_start_cycles) / cycles_per_us;

    if (echo_duration_us == 0U) {
        return -3.0f; // It is imposible to have a zero duration for the Echo pulse
    }

    // the speed of sound is aproximately 343m/s
    // d total = 2d = (speed * time) / 2 
    // distance in cm = (343 * 100cm/1.000.000us  *  echo_duration_us) / 2  
    // distance = (0.0343 * t) / 2 = t / (2/0.0343) = t / 58.31
    return (float)echo_duration_us / 58.31f;
}
