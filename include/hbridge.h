#ifndef HBRIDGE_H
#define HBRIDGE_H

#include "fsl_ctimer.h"
#include "fsl_gpio.h"

typedef struct {
    CTIMER_Type   *pwmPeripheral;
    ctimer_match_t periodChannel;

    /* Motor 1 (Left / Canal A): ENA, IN1, IN2 */
    ctimer_match_t pwm1Channel;
    GPIO_Type     *motor1In1Port;
    uint32_t       motor1In1Pin;
    GPIO_Type     *motor1In2Port;
    uint32_t       motor1In2Pin;

    /* Motor 2 (Right / Canal B): ENB, IN3, IN4 */
    ctimer_match_t pwm2Channel;
    GPIO_Type     *motor2In3Port;
    uint32_t       motor2In3Pin;
    GPIO_Type     *motor2In4Port;
    uint32_t       motor2In4Pin;
} Hbridge;

extern Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1In1Port, uint32_t m1In1Pin,
                 GPIO_Type *m1In2Port, uint32_t m1In2Pin,
                 GPIO_Type *m2In3Port, uint32_t m2In3Pin,
                 GPIO_Type *m2In4Port, uint32_t m2In4Pin);

void HbridgeMotor1Speed(Hbridge *h, int16_t speed);
void HbridgeMotor2Speed(Hbridge *h, int16_t speed);
void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2);
void HbridgeBrake(Hbridge *h);

#ifdef __cplusplus
}
#endif

#endif
