#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"

void Steer(double angle)
{
    if (angle > 100.0)  angle = 100.0;
    if (angle < -100.0) angle = -100.0;

    double duty = 5.0 + ((angle + 100.0) / 200.0) * 5.0;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    uint32_t pulseTicks = (uint32_t)((periodTicks * (100.0 - duty)) / 100.0);
    //uint32_t pulseTicks = (uint32_t)((periodTicks * duty) / 100.0);

    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_3_CHANNEL] = pulseTicks;
}

void TestServo(){
	volatile int Delay;
	volatile int SteerStrength;
	while(1){
		for(SteerStrength = -100; SteerStrength <=100; SteerStrength++){
			Delay = 200000;
			while(Delay){
				Delay--;
			}
			PRINTF("Steer: %d\n", SteerStrength);
			Steer(SteerStrength);
		}
	}
}

void TestServoRightLeft(void)
{
    extern uint32_t SystemCoreClock;
    PRINTF("Servo Test: 3 seconds to the right...\r\n");
    Steer(50.0); /* Steer right */
    SDK_DelayAtLeastUs(3000000U, SystemCoreClock);

    PRINTF("Servo Test: 7 seconds to the left...\r\n");
    Steer(-50.0); /* Steer left */
    SDK_DelayAtLeastUs(7000000U, SystemCoreClock);

    PRINTF("Servo Test: Centering...\r\n");
    Steer(0.0); /* Center steering */
}