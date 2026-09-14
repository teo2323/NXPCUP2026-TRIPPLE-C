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

#define MAX_VECTORS 10

int main(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* Initialize H-bridge with bootcampWorkspace pins for new car */
    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 0, 255, 0); // Green LED indicates active mode

    /* Drive speed hardcoded to constant 60 for both motors */
    HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);
    Steer(0 + STEERING_OFFSET);

    double last_steering_angle = 0.0;
    double previous_error      = 0.0;  // D term: stores last frame's error

    while (1)
    {
        /* Maintain continuous constant motor speed (60) */
        HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {

            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                /* Continuous PID steering control */
                double error = det.steering_angle;

                /* D term: calculated from raw error difference */
                double derivative = error - previous_error;
                previous_error    = error;

                /* P + D combination: output = P*error + D*derivative */
                double p_term = (error > 0) ? (STEERING_P_RIGHT * error) : (STEERING_P_LEFT * error);
                double d_term = (derivative > 0) ? (STEERING_D_RIGHT * derivative) : (STEERING_D_LEFT * derivative);
                double steer_angle = p_term + d_term;

                if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                Steer(steer_angle + STEERING_OFFSET);
                last_steering_angle = steer_angle;
            }
            else {
                /* 0 track lines detected -> search for horizontal turn-track fallback vector */
                turn_track_result_t turn;
                if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
                    double error = turn.steering_angle;

                    double derivative = error - previous_error;
                    previous_error    = error;

                    double p_term = (error > 0) ? (STEERING_P_RIGHT * error) : (STEERING_P_LEFT * error);
                    double d_term = (derivative > 0) ? (STEERING_D_RIGHT * derivative) : (STEERING_D_LEFT * derivative);
                    double steer_angle = p_term + d_term;

                    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                    Steer(steer_angle + STEERING_OFFSET);
                    last_steering_angle = steer_angle;
                }
                else {
                    /* No vectors detected -> gently decay angle using DECAY_FACTOR (0.9) */
                    last_steering_angle *= DECAY_FACTOR;
                    if (fabs(last_steering_angle) < 1.0) {
                        last_steering_angle = 0.0;
                    }
                    Steer(last_steering_angle + STEERING_OFFSET);
                }
            }
        }
    }
}
