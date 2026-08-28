#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ULTRASONIC_GPIO_PORT    GPIO4
#define ULTRASONIC_TRIG_PIN     22U   /* P4_22 — Trigger output */
#define ULTRASONIC_ECHO_PIN     23U   /* P4_23 — Echo input     */

void Ultrasonic_Init(void);
float Ultrasonic_ReadDistanceCm(void);

/* Non-blocking API functions */
void  Ultrasonic_StartTrigger(void);
float Ultrasonic_GetDistanceCm(void);
void  GPIO40_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H_ */
