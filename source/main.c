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

static inline double compute_variable_pid(double error, double *previous_error, bool is_sharp_turn)
{
    /* D term: calculated from error difference */
    double derivative = error - *previous_error;
    *previous_error   = error;

    /* 2-Zone Gain Selection: Straight vs Curve */
    double kp, kd;
    if (fabs(error) > CURVE_ERROR_THRESHOLD || is_sharp_turn) {
        kp = STEERING_KP_CURVE;
        kd = STEERING_KD_CURVE;
    } else {
        kp = STEERING_KP_STRAIGHT;
        kd = STEERING_KD_STRAIGHT;
    }

    /* Variable P + D output */
    double steer_angle = (kp * error) + (kd * derivative);

    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

    return steer_angle;
}

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

    /* Initial base drive speed and straight steering */
    HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);
    Steer(0 + STEERING_OFFSET);

    double last_steering_angle = 0.0;
    double previous_error      = 0.0;  // D term: stores last frame's error

    while (1)
    {
        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {

            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            /* Display line detection status and coordinates */
            if (det.left_line_present) {
                PRINTF("Left Line: DETECTED  | Start: (%u, %u), End: (%u, %u)\r\n",
                       det.left_line.vector.x0, det.left_line.vector.y0,
                       det.left_line.vector.x1, det.left_line.vector.y1);
            } else {
                PRINTF("Left Line: NOT DETECTED\r\n");
            }

            if (det.right_line_present) {
                PRINTF("Right Line: DETECTED | Start: (%u, %u), End: (%u, %u)\r\n",
                       det.right_line.vector.x0, det.right_line.vector.y0,
                       det.right_line.vector.x1, det.right_line.vector.y1);
            } else {
                PRINTF("Right Line: NOT DETECTED\r\n");
            }

            bool is_sharp_turn = false;

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                if (det.sharp_turn_detected) {
                    is_sharp_turn = true;
                }

                /* 2-Zone Variable PID steering control */
                double steer_angle = compute_variable_pid(det.steering_angle, &previous_error, is_sharp_turn);

                Steer(steer_angle + STEERING_OFFSET);
                last_steering_angle = steer_angle;
            }
            // else {
            //     /* 0 track lines detected -> search for horizontal turn-track fallback vector */
            //     turn_track_result_t turn;
            //     if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
            //         /* Horizontal fallback vector indicates a sharp turn in progress */
            //         is_sharp_turn = true;

            //         /* 2-Zone Variable PID steering control */
            //         double steer_angle = compute_variable_pid(turn.steering_angle, &previous_error, is_sharp_turn);

            //         Steer(steer_angle + STEERING_OFFSET);
            //         last_steering_angle = steer_angle;
            //     }
            // }
            else {
                /* No vectors detected -> gently decay angle using DECAY_FACTOR (0.9) */
                last_steering_angle *= DECAY_FACTOR;
                if (fabs(last_steering_angle) < 1.0) {
                    last_steering_angle = 0.0;
                }
                Steer(last_steering_angle + STEERING_OFFSET);
            }

            /* Differential drive: outer wheel faster, inner wheel slower.
             * diff > 0 → right turn → left (outer) faster, right (inner) slower.
             * diff < 0 → left turn  → right (outer) faster, left (inner) slower. */
            double diff = last_steering_angle * DIFF_SPEED_COEFF;
            int16_t current_speed_left  = (int16_t)(SPEED_LEFT  + diff);
            int16_t current_speed_right = (int16_t)(SPEED_RIGHT - diff);

            HbridgeSpeed(&g_hbridge, current_speed_left, current_speed_right);
        }
    }
}
