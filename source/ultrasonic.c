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

// #include "ultrasonic.h"
// #include "fsl_gpio.h"
// #include "fsl_common.h"
// #include "fsl_debug_console.h"
// #include "Config.h"

// /*******************************************************************************
//  * Private helpers — DWT cycle-counter timing
//  *
//  * The Cortex-M33 DWT (Data Watchpoint and Trace) unit contains a 32-bit free-
//  * running CPU cycle counter (DWT->CYCCNT).  Reading it before and after an
//  * operation gives the elapsed cycles, which can be converted to microseconds
//  * with (cycles * 1 000 000) / SystemCoreClock.  This is far more accurate than
//  * a busy-wait loop whose iteration count is sensitive to compiler optimisation,
//  * cache state, and pipeline effects.
//  ******************************************************************************/

// /* Enable the DWT cycle counter.  Safe to call more than once. */
// static void dwt_init(void)
// {
//     CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* Enable trace / DWT */
//     DWT->CYCCNT       = 0U;                           /* Reset counter       */
//     DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;      /* Start counting      */
// }

// /* Return elapsed microseconds since 'start_cycles' was captured. */
// static inline uint32_t cycles_to_us(uint32_t start_cycles)
// {
//     uint32_t elapsed = DWT->CYCCNT - start_cycles;
//     /* Avoid 32-bit overflow: divide first, then multiply.
//        At 150 MHz one µs = 150 cycles; dividing by (SystemCoreClock/1 000 000)
//        keeps the result in microseconds. */
//     return elapsed / (SystemCoreClock / 1000000U);
// }

// /* Busy-wait for the requested number of microseconds using DWT. */
// static void delay_us(uint32_t us)
// {
//     uint32_t start = DWT->CYCCNT;
//     uint32_t ticks = us * (SystemCoreClock / 1000000U);
//     while ((DWT->CYCCNT - start) < ticks) {}
// }

// /*******************************************************************************
//  * Public API
//  ******************************************************************************/

// void Ultrasonic_Init(void)
// {
//     /* Initialise DWT so cycle-accurate timing is available. */
//     dwt_init();

//     /* Configure TRIG pin as digital output, initially LOW. */
//     gpio_pin_config_t trig_cfg = {
//         .pinDirection = kGPIO_DigitalOutput,
//         .outputLogic  = 0U
//     };
//     GPIO_PinInit(ULTRASONIC_GPIO_PORT, ULTRASONIC_TRIG_PIN, &trig_cfg);

//     /* Configure ECHO pin as digital input. */
//     gpio_pin_config_t echo_cfg = {
//         .pinDirection = kGPIO_DigitalInput,
//         .outputLogic  = 0U   /* unused for inputs */
//     };
//     GPIO_PinInit(ULTRASONIC_GPIO_PORT, ULTRASONIC_ECHO_PIN, &echo_cfg);
// }

// float Ultrasonic_MeasureMm(void)
// {
//     uint32_t t_start;
//     uint32_t elapsed_us;

//     /* --- 1. Send 10 µs trigger pulse --- */
//     GPIO_PinWrite(ULTRASONIC_GPIO_PORT, ULTRASONIC_TRIG_PIN, 0U);
//     delay_us(2U);   /* Ensure TRIG is LOW before pulsing */

//     GPIO_PinWrite(ULTRASONIC_GPIO_PORT, ULTRASONIC_TRIG_PIN, 1U);
//     delay_us(10U);  /* HC-SR04 requires >= 10 µs HIGH pulse */

//     GPIO_PinWrite(ULTRASONIC_GPIO_PORT, ULTRASONIC_TRIG_PIN, 0U);

//     /* --- 2. Wait for ECHO rising edge, with timeout --- */
//     t_start = DWT->CYCCNT;
//     while (GPIO_PinRead(ULTRASONIC_GPIO_PORT, ULTRASONIC_ECHO_PIN) == 0U)
//     {
//         if (cycles_to_us(t_start) >= ULTRASONIC_TIMEOUT_US)
//         {
//             return -1.0f; /* No rising edge — sensor not responding */
//         }
//     }

//     /* --- 3. Measure ECHO HIGH pulse width using DWT --- */
//     t_start = DWT->CYCCNT;  /* Capture timestamp at rising edge */

//     while (GPIO_PinRead(ULTRASONIC_GPIO_PORT, ULTRASONIC_ECHO_PIN) != 0U)
//     {
//         if (cycles_to_us(t_start) >= ULTRASONIC_TIMEOUT_US)
//         {
//             return -1.0f; /* Echo stuck HIGH — out of range */
//         }
//     }

//     elapsed_us = cycles_to_us(t_start);  /* Exact pulse width in microseconds */

//     /* --- 4. Convert pulse width to distance ---
//      *
//      * Speed of sound ≈ 343 m/s at 20 °C.
//      * Round-trip time per millimetre ≈ 5.831 µs.
//      * The sensor measures the round-trip, so: distance = elapsed_us / 5.831f. */
//     return (float)elapsed_us / 5.831f;
// }

// bool Ultrasonic_CheckObstacle(Hbridge *h)
// {
//     float dist_mm = Ultrasonic_MeasureMm();

//     /* Ignore invalid / out-of-range readings — don't false-trigger a stop. */
//     if (dist_mm < 0.0f)
//     {
//         return false;
//     }

//     if (dist_mm < ULTRASONIC_STOP_THRESHOLD_MM)
//     {
//         /* Obstacle within threshold — stopping logic commented out for testing.
//            Uncomment the block below to re-enable the emergency stop. */
// /*
//         HbridgeBrake(h);
//         is_running = false;

//         PRINTF("[ULTRASONIC] Obstacle at %d mm — EMERGENCY STOP\r\n",
//                (int)dist_mm);
// */
//         return true;
//     }

//     return false;
// }
