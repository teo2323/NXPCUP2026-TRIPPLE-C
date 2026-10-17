#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "Config.h"

void Steer(double angle)
{
    // 1. TODO: clamp `angle` to the range [-100.0, 100.0]

	if(angle > 100) angle = 100;
	if(angle < -100) angle = -100;



    // 2. TODO: map the angle to a duty cycle between 5.0% and 10.0%
    double duty = 5+ ((angle+100)*5)/200;

    // 3. TODO: read the PWM period from the period channel match register
    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];

    // 4. TODO: convert the duty cycle into pulse ticks; the match value is
    //          where the pulse starts, so use (100.0 - duty)
    uint32_t pulseTicks =periodTicks * ((100-duty)/100) ;

    // 5. Write the result to the servo channel match register:
    // Match 1 controls CT2_MAT1 on PIO3_19 (connected to Shield J17 header)
    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_1_CHANNEL] = pulseTicks;
    // Also write to Match 2 for backwards compatibility with J13 header
    CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_2_CHANNEL] = pulseTicks;
}

void TestServo(){
	volatile int Delay;
	volatile int SteerStrength;
	while(1){
		for(SteerStrength = DEFAULT_STEERING_LIMIT_LEFT; SteerStrength <=DEFAULT_STEERING_LIMIT_RIGHT; SteerStrength++){
			Delay = 200000;
			while(Delay){
				Delay--;
			}
			PRINTF("Steer: %d\n", SteerStrength);
			Steer(SteerStrength);
		}
	}
}
