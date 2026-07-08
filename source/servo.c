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

    CTIMER2_PERIPHERAL->MR[2] = pulseTicks;
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
