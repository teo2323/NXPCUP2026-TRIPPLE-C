#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

Hbridge g_hbridge;
static uint32_t s_srcClockHz;

void HbridgeInit(Hbridge *h,
		 	 	 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    ctimer_config_t config;

    
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;

    g_hbridge = *h;
}

void HbridgeSpeed(Hbridge *h, int16_t speed)
{
    uint8_t duty = (uint8_t)(speed < 0 ? -speed : speed);
    if (duty > 100U) duty = 100U;

    if (speed > 0) {
        // Forward: IN1 = 1, IN2 = 0
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U); // IN1 (P0_27) -> HIGH
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U); // IN2 (P0_26) -> LOW
    } 
    else if (speed < 0) {
        // Reverse: IN1 = 0, IN2 = 1
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U); // IN1 (P0_27) -> LOW
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U); // IN2 (P0_26) -> HIGH
    } 
    else {
        // Brake: IN1 = 0, IN2 = 0
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
    }

    // Update ENA PWM duty cycle (assuming pwm1Channel is mapped to ENA)
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm2Channel, 0U);
}