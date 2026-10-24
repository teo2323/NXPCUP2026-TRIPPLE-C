#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "Config.h"

void Steer(double angle)
{
    // 1. Clamp `angle` to the range [-100.0, 100.0]
    if (angle > 100.0) angle = 100.0;
    if (angle < -100.0) angle = -100.0;

    // 2. Map the angle to a duty cycle between 5.0% and 10.0%
    double duty = 5.0 + ((angle + 100.0) * 5.0) / 200.0;

    // 3. Read the PWM period from the period channel match register
    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];

    // 4. Convert the duty cycle into pulse ticks; match value is where pulse starts
    uint32_t pulseTicks = (uint32_t)(periodTicks * ((100.0 - duty) / 100.0));

    // 5. Write the result to both servo match channels:
    // Match 1 controls CT2_MAT1 on PIO3_19 (connected to Shield J17 header)
    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_1_CHANNEL] = pulseTicks;
    // Match 2 controls CT2_MAT2 on PIO1_12 (connected to J13 header)
    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_2_CHANNEL] = pulseTicks;
}

void TestServo(void)
{
    volatile int Delay;
    volatile int SteerStrength;
    while (1) {
        for (SteerStrength = (int)DEFAULT_STEERING_LIMIT_LEFT; SteerStrength <= (int)DEFAULT_STEERING_LIMIT_RIGHT; SteerStrength++) {
            Delay = 200000;
            while (Delay) {
                Delay--;
            }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer((double)SteerStrength);
        }
    }
}
