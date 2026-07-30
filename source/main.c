#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"
#include "detection.h"
#include <math.h>

#define MAX_VECTORS          10
#define AUTOMATED_BASE_SPEED 40

int main(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootClocks();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL, // ENA (P0_25)
                CTIMER0_PWM_2_CHANNEL, // ENB (P0_24)
                GPIO0, 27U,            // IN1 (P0_27)
                GPIO0, 26U,            // IN2 (P0_26)
                GPIO0, 28U,            // IN3 (P0_28)
                GPIO0, 31U             // IN4 (P0_31)
    );
    extern uint32_t SystemCoreClock;

    CTIMER_StartTimer(CTIMER0_PERIPHERAL);

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 0, 255, 0); // Green LED indicates active automated mode

    /* 1. Continuous H-bridge drive speed */
    //HbridgeSpeed(&g_hbridge, 70, 70);
    Steer(0.0);

    double last_steering_angle = 0.0;

    while (1)
    {
        /* Maintain continuous motor speed rate */
        HbridgeSpeed(&g_hbridge, 70, 70);

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            // PRINTF("[Pixy Camera] Detected Vectors Count: Raw = %u, Valid = %u\r\n",
                   //(unsigned)num_vectors, (unsigned)det.valid_vectors);

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                /* Use virtual centerline steering computed by detection_process_dual_lines */
                double steer_angle = det.steering_angle;

                if (steer_angle > 0) steer_angle *= STEERING_P_RIGHT;
                else steer_angle *= STEERING_P_LEFT;

                if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                Steer(steer_angle);
                last_steering_angle = steer_angle;

                int ang_int = (int)steer_angle;

                if (det.both_lines_present) {
                    // PRINTF("[Detection] 2 Track Lines (BOTH) | Raw: %u, Valid: %u | Steer: %d deg\r\n",
                           //(unsigned)num_vectors, (unsigned)det.valid_vectors, ang_int);
                }
                else if (!det.left_line_present && det.right_line_present) {
                    // PRINTF("[Detection] 1 Track Line (RIGHT only) | Raw: %u, Valid: %u | Virtual Center Steer: %d deg\r\n",
                           //(unsigned)num_vectors, (unsigned)det.valid_vectors, ang_int);
                }
                else if (det.left_line_present && !det.right_line_present) {
                    // PRINTF("[Detection] 1 Track Line (LEFT only) | Raw: %u, Valid: %u | Virtual Center Steer: %d deg\r\n",
                           //(unsigned)num_vectors, (unsigned)det.valid_vectors, ang_int);
                }
            }
            else {
                /* 0 lines detected -> Gently decay search angle toward 0° straight to prevent lockup */
                last_steering_angle *= 0.8;
                if (fabs(last_steering_angle) < 1.0) {
                    last_steering_angle = 0.0;
                }
                Steer(last_steering_angle);

                int ang_int = (int)last_steering_angle;
                // PRINTF("[Detection] 0 Track Lines | Raw: %u, Valid: %u | Decaying search angle: %d deg\r\n",
                       //(unsigned)num_vectors, (unsigned)det.valid_vectors, ang_int);
            }
        }
    }
}