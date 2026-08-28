#include "ultrasonic.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "fsl_common.h" 
#include "pin_mux.h"

extern uint32_t SystemCoreClock;

typedef enum {
    ULTRASONIC_STATE_IDLE = 0,
    ULTRASONIC_STATE_WAIT_RISING,
    ULTRASONIC_STATE_WAIT_FALLING
} ultrasonic_state_t;

static volatile ultrasonic_state_t s_ultra_state = ULTRASONIC_STATE_IDLE;
static volatile uint32_t s_echo_start_cycles = 0;
static volatile uint32_t s_echo_end_cycles = 0;
static volatile float s_latest_distance_cm = -1.0f;
static uint32_t s_last_trig_cycles = 0;

void Ultrasonic_Init(void)
{
    MSDK_EnableCpuCycleCounter();
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);

    /* Configure GPIO Echo pin interrupt on either edge */
    GPIO_SetPinInterruptConfig(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN, kGPIO_InterruptEitherEdge);
    EnableIRQ(GPIO40_IRQn);
}

void GPIO40_IRQHandler(void)
{
    uint32_t flags = GPIO_GpioGetInterruptFlags(BOARD_INITPINS_senzor2_echo_GPIO);
    if (flags & (1U << BOARD_INITPINS_senzor2_echo_PIN))
    {
        GPIO_PinClearInterruptFlag(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN);

        uint32_t current_cycles = MSDK_GetCpuCycleCount();
        uint32_t pin_val = GPIO_PinRead(BOARD_INITPINS_senzor2_echo_GPIO, BOARD_INITPINS_senzor2_echo_PIN);

        if (pin_val != 0U) // Echo went HIGH (Rising Edge)
        {
            s_echo_start_cycles = current_cycles;
            s_ultra_state = ULTRASONIC_STATE_WAIT_FALLING;
        }
        else // Echo went LOW (Falling Edge)
        {
            if (s_ultra_state == ULTRASONIC_STATE_WAIT_FALLING)
            {
                s_echo_end_cycles = current_cycles;
                uint32_t cycles_per_us = SystemCoreClock / 1000000U;
                if (cycles_per_us == 0U) cycles_per_us = 150U;

                uint32_t duration_us = (s_echo_end_cycles - s_echo_start_cycles) / cycles_per_us;
                // HC-SR04 valid physical range: 2 cm (~116 us) to 400 cm (~23300 us)
                // Any pulse < 116 us is electrical noise/glitch from motor PWM switching and MUST be discarded!
                if (duration_us >= 116U && duration_us <= 23300U)
                {
                    s_latest_distance_cm = (float)duration_us / 58.31f;
                }
                s_ultra_state = ULTRASONIC_STATE_IDLE;
            }
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

void Ultrasonic_StartTrigger(void)
{
    if (s_ultra_state != ULTRASONIC_STATE_IDLE) return;

    uint32_t now = MSDK_GetCpuCycleCount();
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) cycles_per_us = 150U;

    // Minimum 60 ms (60,000 us) inter-trigger interval to prevent acoustic echo overlap
    if (s_last_trig_cycles != 0U && (now - s_last_trig_cycles) < (60000U * cycles_per_us))
    {
        return;
    }

    s_ultra_state = ULTRASONIC_STATE_WAIT_RISING;
    s_last_trig_cycles = now;

    // Pulse Trigger: 10us pulse
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
    SDK_DelayAtLeastUs(2U, SystemCoreClock);
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 1U);
    SDK_DelayAtLeastUs(10U, SystemCoreClock);
    GPIO_PinWrite(BOARD_INITPINS_senzor2_trig_GPIO, BOARD_INITPINS_senzor2_trig_PIN, 0U);
}

float Ultrasonic_GetDistanceCm(void)
{
    uint32_t now = MSDK_GetCpuCycleCount();
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0U) cycles_per_us = 150U;

    /* Timeout check: if echo didn't finish within 40 ms, reset state */
    if (s_ultra_state != ULTRASONIC_STATE_IDLE)
    {
        if ((now - s_last_trig_cycles) > (40000U * cycles_per_us))
        {
            s_ultra_state = ULTRASONIC_STATE_IDLE;
            s_latest_distance_cm = -2.0f; // Timeout
        }
    }
    return s_latest_distance_cm;
}

float Ultrasonic_ReadDistanceCm(void)
{
    /* Non-blocking trigger and read: starts trigger if idle and returns latest measurement */
    Ultrasonic_StartTrigger();
    return Ultrasonic_GetDistanceCm();
}
