#include "ultrasonic.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "pin_mux.h"

extern uint32_t SystemCoreClock;

void Ultrasonic_Init(void)
{
    /* Activează contorul de cicluri hardware CPU pentru măsurare exactă */
    MSDK_EnableCpuCycleCounter();
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
}

float Ultrasonic_ReadDistanceCm(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 150U; // Fallback pentru 150 MHz
    }

    /* 0. Verificăm starea inițială a pinului Echo înaintea impulsului Trigger */
    uint32_t initial_echo = GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN);

    /* 1. Asigurăm starea LOW pe Trigger */
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
    SDK_DelayAtLeastUs(2U, SystemCoreClock);

    /* 2. Trimitem impulsul OBLIGATORIU de 10us pe Trigger */
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 1U);
    SDK_DelayAtLeastUs(10U, SystemCoreClock);
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);

    /* 3. Așteptăm ca Echo să treacă în 1 (HIGH) cu timeout de ~10ms */
    uint32_t start_cycles = MSDK_GetCpuCycleCount();
    uint32_t max_wait_cycles = 10000U * cycles_per_us; 

    while (GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN) == 0U)
    {
        if ((MSDK_GetCpuCycleCount() - start_cycles) > max_wait_cycles)
        {
            if (initial_echo != 0U) {
                return -4.0f; // Pinul Echo era deja HIGH (1) înainte de Trigger
            }
            return -1.0f; // Timeout waiting for Echo HIGH (Pinul Echo a rămas LOW=0)
        }
    }

    /* 4. Măsurăm durata semnalului HIGH pe Echo folosind ciclurile CPU */
    uint32_t echo_start_cycles = MSDK_GetCpuCycleCount();
    uint32_t max_echo_cycles = 30000U * cycles_per_us; // ~30ms max (~5m)

    while (GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN) != 0U)
    {
        if ((MSDK_GetCpuCycleCount() - echo_start_cycles) > max_echo_cycles)
        {
            return -2.0f; // Timeout waiting for Echo LOW (Pinul Echo a trecut în 1, dar a rămas 1 prea mult / out of range)
        }
    }

    uint32_t echo_end_cycles = MSDK_GetCpuCycleCount();
    uint32_t echo_duration_us = (echo_end_cycles - echo_start_cycles) / cycles_per_us;

    if (echo_duration_us == 0U) {
        return -3.0f; // Durată 0 microsecunde
    }

    /* 5. Calculăm distanța în centimetri: timp (us) / 58.0 */
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
