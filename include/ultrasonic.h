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

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H_ */


// #ifndef ULTRASONIC_H_
// #define ULTRASONIC_H_

// #include <stdint.h>
// #include <float.h>
// #include "hbridge.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

// /*******************************************************************************
//  * Definitions
//  ******************************************************************************/

// /* HC-SR04 ultrasonic sensor pin mapping (GPIO5, matching pin_mux config) */
// #define ULTRASONIC_GPIO_PORT    GPIO4
// #define ULTRASONIC_TRIG_PIN     22U    /* P5_8 — Trigger output (J9[31]) */
// #define ULTRASONIC_ECHO_PIN     23U    /* P5_9 — Echo input     (J9[29]) */

// /* Maximum echo wait time in microseconds (~38 ms for out-of-range).
//    HC-SR04 max range: ~400 cm  →  roundtrip ~23 ms; 38 ms = sensor timeout. */
// #define ULTRASONIC_TIMEOUT_US   38000U

// /* Distance returned when no echo is received (sensor timeout or out of range). */
// #define ULTRASONIC_NO_OBJECT_CM  (-1.0f)

// /* Object closer than this (mm) triggers an emergency stop.
//    Adjust to taste — 150 mm (15 cm) is a safe default for a small RC car. */
// #define ULTRASONIC_STOP_THRESHOLD_MM  50.0f

// /* Distance (mm) at which the car begins to slow down proportionally.
//    Speed ramps linearly from full at SLOWDOWN_START down to zero at STOP_THRESHOLD. */
// #define ULTRASONIC_SLOWDOWN_START_MM  200.0f

// /*******************************************************************************
//  * API
//  ******************************************************************************/

// /**
//  * @brief Initialise the TRIG (output) and ECHO (input) GPIO pins.
//  *
//  * Call once after BOARD_InitBootPins() / BOARD_InitBootPeripherals().
//  * The TRIG pin is driven LOW initially; ECHO is configured as a
//  * floating input (the HC-SR04 drives it directly).
//  */
// void Ultrasonic_Init(void);

// /**
//  * @brief Trigger a single measurement and return the distance in centimetres.
//  *
//  * Sends a 10 µs TRIG pulse, waits for the rising edge on ECHO, measures
//  * the pulse width, then converts to centimetres using the speed of sound
//  * (approximately 58 µs per centimetre round-trip).
//  *
//  * @return Distance in millimetres as a float, or ULTRASONIC_NO_OBJECT_CM when
//  *         no echo is received within ULTRASONIC_TIMEOUT_US microseconds.
//  */
// float Ultrasonic_MeasureMm(void);

// /**
//  * @brief Measure distance and brake the car if an obstacle is within
//  *        ULTRASONIC_STOP_THRESHOLD_MM millimetres.
//  *
//  * Returns true if an obstacle was detected and the car was stopped,
//  * false if the path is clear.
//  *
//  * @param h  Pointer to the initialised Hbridge instance (e.g. &g_hbridge).
//  */
// bool Ultrasonic_CheckObstacle(Hbridge *h);

// #ifdef __cplusplus
// }
// #endif

// #endif /* ULTRASONIC_H_ */

