#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"

#define SERVO_OFFSET -3

void Steer(double angle)
{
    // Apply zero-point offset (SERVO_OFFSET corresponds to physical straight calibration)
    double effective_angle = angle + SERVO_OFFSET;

    if (effective_angle > 100.0)  effective_angle = 100.0;
    if (effective_angle < -100.0) effective_angle = -100.0;

    // Map angle range [-100, +100] to PWM duty cycle range [5.0%, 10.0%] (1.0ms to 2.0ms pulse)
    double duty = 5.0 + ((effective_angle + 100.0) / 200.0) * 5.0;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    if (periodTicks == 0U) {
        periodTicks = 19999U; // Fallback to default 20ms period (50Hz)
    }

    uint32_t pulseTicks = (uint32_t)(((double)periodTicks * (100.0 - duty)) / 100.0);

    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_3_CHANNEL] = pulseTicks;
}

void TestServo(){
	volatile int Delay;
	volatile int SteerStrength;
	while(1){
		for(SteerStrength = -45; SteerStrength <=45; SteerStrength++){
			Delay = 200000;
			while(Delay){
				Delay--;
			}
			// PRINTF("Steer: %d\n", SteerStrength);
			Steer(SteerStrength);
		}
	}
}

void TestServoRightLeft()
{
    extern uint32_t SystemCoreClock;
    // PRINTF("Servo Test: 3 seconds to the right...\r\n");
    Steer(30.0); /* Steer right */
    SDK_DelayAtLeastUs(3000000U, SystemCoreClock);

    // PRINTF("Servo Test: 7 seconds to the left...\r\n");
    Steer(-30.0); /* Steer left */
    SDK_DelayAtLeastUs(3000000U, SystemCoreClock);

    // PRINTF("Servo Test: Centering...\r\n");
    Steer(0.0); /* Center steering */
}