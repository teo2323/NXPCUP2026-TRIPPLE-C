#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"

Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1In1Port, uint32_t m1In1Pin,
                 GPIO_Type *m1In2Port, uint32_t m1In2Pin,
                 GPIO_Type *m2In3Port, uint32_t m2In3Pin,
                 GPIO_Type *m2In4Port, uint32_t m2In4Pin)
{
    h->pwmPeripheral = pwmPeriph;
    h->periodChannel = periodCh;
    h->pwm1Channel   = pwm1Ch;
    h->pwm2Channel   = pwm2Ch;

    h->motor1In1Port = m1In1Port;
    h->motor1In1Pin  = m1In1Pin;
    h->motor1In2Port = m1In2Port;
    h->motor1In2Pin  = m1In2Pin;

    h->motor2In3Port = m2In3Port;
    h->motor2In3Pin  = m2In3Pin;
    h->motor2In4Port = m2In4Port;
    h->motor2In4Pin  = m2In4Pin;

    g_hbridge = *h;
}

void HbridgeMotor1Speed(Hbridge *h, int16_t speed)
{
    uint8_t duty = (uint8_t)(speed < 0 ? -speed : speed);
    if (duty > 100U) duty = 100U;

    if (speed > 0) {
        // Forward Motor 1: IN1 = 1, IN2 = 0
        GPIO_PinWrite(h->motor1In1Port, h->motor1In1Pin, 1U);
        GPIO_PinWrite(h->motor1In2Port, h->motor1In2Pin, 0U);
    } 
    else if (speed < 0) {
        // Reverse Motor 1: IN1 = 0, IN2 = 1
        GPIO_PinWrite(h->motor1In1Port, h->motor1In1Pin, 0U);
        GPIO_PinWrite(h->motor1In2Port, h->motor1In2Pin, 1U);
    } 
    else {
        // Brake Motor 1: IN1 = 0, IN2 = 0
        GPIO_PinWrite(h->motor1In1Port, h->motor1In1Pin, 0U);
        GPIO_PinWrite(h->motor1In2Port, h->motor1In2Pin, 0U);
    }

    // Update ENA PWM duty cycle
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty);
}

void HbridgeMotor2Speed(Hbridge *h, int16_t speed)
{
    uint8_t duty = (uint8_t)(speed < 0 ? -speed : speed);
    if (duty > 100U) duty = 100U;

    if (speed > 0) {
        // Forward Motor 2: IN3 = 1, IN4 = 0
        GPIO_PinWrite(h->motor2In3Port, h->motor2In3Pin, 1U);
        GPIO_PinWrite(h->motor2In4Port, h->motor2In4Pin, 0U);
    } 
    else if (speed < 0) {
        // Reverse Motor 2: IN3 = 0, IN4 = 1
        GPIO_PinWrite(h->motor2In3Port, h->motor2In3Pin, 0U);
        GPIO_PinWrite(h->motor2In4Port, h->motor2In4Pin, 1U);
    } 
    else {
        // Brake Motor 2: IN3 = 0, IN4 = 0
        GPIO_PinWrite(h->motor2In3Port, h->motor2In3Pin, 0U);
        GPIO_PinWrite(h->motor2In4Port, h->motor2In4Pin, 0U);
    }

    // Update ENB PWM duty cycle
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty);
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2)
{
    HbridgeMotor1Speed(h, speed1);
    HbridgeMotor2Speed(h, speed2);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1In1Port, h->motor1In1Pin, 0U);
    GPIO_PinWrite(h->motor1In2Port, h->motor1In2Pin, 0U);
    GPIO_PinWrite(h->motor2In3Port, h->motor2In3Pin, 0U);
    GPIO_PinWrite(h->motor2In4Port, h->motor2In4Pin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 0U);
}