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
#include "wifi.h"
#include <math.h>

#define MAX_VECTORS 10

volatile uint32_t g_systick_ms = 0;

void SysTick_Handler(void)
{
    g_systick_ms++;
}

static inline double compute_variable_pid(double error,
                                          double curve_angle,
                                          double *previous_error,
                                          double last_steering_angle,
                                          bool *is_in_curve,
                                          uint8_t *straight_frames)
{
    double abs_error       = fabs(error);
    double abs_curve_angle = fabs(curve_angle);

    /* Hysteresis band for transitioning between Straight and Curve zones based on track line angle:
     * - Enter curve mode if |curve_angle| > CURVE_ENTER_ANGLE_DEG.
     * - Stay in curve mode until |curve_angle| < CURVE_EXIT_ANGLE_DEG
     *   for CURVE_EXIT_FRAMES consecutive frames. */
    if (*is_in_curve) {
        if (abs_curve_angle < CURVE_EXIT_ANGLE_DEG) {
            (*straight_frames)++;
            if (*straight_frames >= CURVE_EXIT_FRAMES) {
                *is_in_curve = false;
                *straight_frames = 0;
            }
        } else {
            *straight_frames = 0;
        }
    } else {
        if (abs_curve_angle > CURVE_ENTER_ANGLE_DEG) {
            *is_in_curve = true;
            *straight_frames = 0;
        }
    }

    /* Dual Deadband & Response Logic (Combination of Idea A & C) */
    if (!(*is_in_curve)) {
        /* Straight Zone: wide deadband; return 0 to lock servo centered */
        if (abs_error <= STEERING_DEADBAND_STRAIGHT) {
            *previous_error = error;
            return 0.0;
        }
    } else {
        /* Curve Zone: tight deadband; hold current curvature arc */
        if (abs_error <= STEERING_DEADBAND_CURVE) {
            *previous_error = error;
            return last_steering_angle;  // Keep steady turn without servo oscillation
        }
    }

    /* D term: calculated from error difference */
    double derivative = error - *previous_error;
    *previous_error   = error;

    /* 2-Zone Gain Selection: Straight vs Curve */
    double kp = (*is_in_curve) ? STEERING_KP_CURVE : STEERING_KP_STRAIGHT;
    double kd = (*is_in_curve) ? STEERING_KD_CURVE : STEERING_KD_STRAIGHT;

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

    /* Configure SysTick to generate a 1ms interrupt for tracking timeouts */
    SysTick_Config(SystemCoreClock / 1000U);

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

    /* Initialize WiFi UART (LP_FLEXCOMM7) — enables RX interrupt and ring buffer */
    Wifi_Init();

    /* Initial base drive speed: respect engine-enabled state from ESP32 */
    int16_t initial_speed = g_engine_enabled ? SPEED_LEFT : 0;
    HbridgeSpeed(&g_hbridge, initial_speed, initial_speed);
    Steer(0 + STEERING_OFFSET);
    

    double   last_steering_angle   = 0.0;
    double   previous_error        = 0.0;   // D term: stores last frame's error
    bool     is_in_curve           = false; // Hysteresis state: true if in curve zone
    uint8_t  straight_frames       = 0;     // Counter for consecutive frames below exit threshold
    uint8_t  no_line_lost_frames   = 0;     // Counter for consecutive frames with no detected track lines
    bool     motors_stopped        = false; // State flag: true when no lines detected for > NO_LINE_MAX_LOST_FRAMES frames

    while (1)
    {
        Wifi_Process_Rx();

        /* Periodic Dual-Duplex Communication Test between FRDM and ESP32 (~1 Hz) */
        static uint32_t comm_test_tick = 0U;
        static uint32_t ping_seq = 0U;
        if (++comm_test_tick >= 60U)
        {
            comm_test_tick = 0U;
            ping_seq++;
            Wifi_SendPing(ping_seq);

            PRINTF("\r\n======================================================\r\n");
            PRINTF(" [DUAL-DUPLEX COMM TEST] FRDM <-> ESP32 (LPUART7 @ 115200)\r\n");
            PRINTF("  FRDM TX -> ESP32: %lu packets sent (Sent PING #%lu)\r\n",
                   (unsigned long)g_wifi_tx_count, (unsigned long)ping_seq);
            PRINTF("  FRDM RX <- ESP32: %lu packets received\r\n",
                   (unsigned long)g_wifi_rx_count);
            if (g_wifi_rx_count > 0U) {
                PRINTF("  >>> STATUS: [FULL-DUPLEX ACTIVE - BIDIRECTIONAL OK] <<<\r\n");
                PRINTF("  Last Msg from ESP32: \"%s\"\r\n", g_wifi_last_rx_cmd);
            } else {
                PRINTF("  >>> STATUS: [WAITING FOR ESP32 RX - TX ONLY] <<<\r\n");
                PRINTF("  Check: ESP32 TX (GPIO17) -> FRDM RX (P3_2), Shared GND\r\n");
            }
            PRINTF("======================================================\r\n\r\n");
        }

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            Wifi_Process_Rx();

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

            bool lines_detected = (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present));

            if (lines_detected) {
                no_line_lost_frames   = 0;
                motors_stopped        = false;

                /* 2-Zone Variable PID steering control with hysteresis */
                double steer_angle = compute_variable_pid(det.steering_angle,
                                                          det.curve_angle,
                                                          &previous_error,
                                                          last_steering_angle,
                                                          &is_in_curve,
                                                          &straight_frames);

                Steer(steer_angle + STEERING_OFFSET);
                last_steering_angle = steer_angle;
            }
            else {
                /* Case: No track lines detected -> check if lost frames exceeded threshold */
                no_line_lost_frames++;
                if (no_line_lost_frames >= NO_LINE_MAX_LOST_FRAMES) {
                    motors_stopped = true;
                }

                /* No vectors detected -> gently decay angle using DECAY_FACTOR */
                last_steering_angle *= DECAY_FACTOR;
                if (fabs(last_steering_angle) < 1.0) {
                    last_steering_angle = 0.0;
                    previous_error      = 0.0;
                    is_in_curve         = false;
                    straight_frames     = 0;
                }
                Steer(last_steering_angle + STEERING_OFFSET);
            }

            if (motors_stopped || !g_engine_enabled) {
                /* When no track lines are detected or engine is disabled, stop the motors */
                HbridgeSpeed(&g_hbridge, 0, 0);
            } else {
                int16_t current_speed_left, current_speed_right;

                if (is_in_curve) {
                    /* Differential drive with corner braking (applied only in curve):
                     * 1) corner_brake slows BOTH wheels proportionally to turn sharpness
                     * 2) diff splits inner/outer: outer gets a boost, inner brakes harder
                     * Result: outer stays ~80-90 in curves, inner drops aggressively */
                    double abs_angle    = fabs(last_steering_angle);
                    double corner_brake = abs_angle * CORNER_BRAKE_COEFF;
                    double diff         = abs_angle * DIFF_SPEED_COEFF;

                    if (last_steering_angle >= 0.0) {
                        /* Right turn: left is outer, right is inner */
                        current_speed_left  = (int16_t)(SPEED_LEFT  - corner_brake + diff);
                        current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake - diff);
                    } else {
                        /* Left turn: right is outer, left is inner */
                        current_speed_left  = (int16_t)(SPEED_LEFT  - corner_brake - diff);
                        current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake + diff);
                    }
                } else {
                    /* Straight section: maintain full symmetric base speed with no differential */
                    current_speed_left  = SPEED_LEFT;
                    current_speed_right = SPEED_RIGHT;
                }

                HbridgeSpeed(&g_hbridge, current_speed_left, current_speed_right);
            }

            /* Send Telemetry to ESP32 over UART at ~10 Hz rate (every 6 camera frames at 60FPS) */
            static uint32_t telemetry_tick = 0U;
            if (++telemetry_tick >= 6U)
            {
                telemetry_tick = 0U;

                uint8_t line_cnt = 0U;
                const char *which_str = "NONE";

                if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present))
                {
                    if (det.both_lines_present) {
                        line_cnt = 2U;
                        which_str = "BOTH";
                    } else if (det.left_line_present) {
                        line_cnt = 1U;
                        which_str = "LEFT";
                    } else {
                        line_cnt = 1U;
                        which_str = "RIGHT";
                    }
                }

                int lx0 = 0, ly0 = 0, lx1 = 0, ly1 = 0;
                int rx0 = 0, ry0 = 0, rx1 = 0, ry1 = 0;

                if (det.left_line_present) {
                    lx0 = (int)det.left_line.vector.x0;
                    ly0 = (int)det.left_line.vector.y0;
                    lx1 = (int)det.left_line.vector.x1;
                    ly1 = (int)det.left_line.vector.y1;
                }
                if (det.right_line_present) {
                    rx0 = (int)det.right_line.vector.x0;
                    ry0 = (int)det.right_line.vector.y0;
                    rx1 = (int)det.right_line.vector.x1;
                    ry1 = (int)det.right_line.vector.y1;
                }

                Wifi_SendTelemetry(line_cnt, which_str, num_vectors, 0U, last_steering_angle, lx0, ly0, lx1, ly1, rx0, ry0, rx1, ry1);
            }
        }
        else {
            /* Camera read failed: check if lost frames exceeded threshold */
            no_line_lost_frames++;
            if (no_line_lost_frames >= NO_LINE_MAX_LOST_FRAMES) {
                motors_stopped = true;
                HbridgeSpeed(&g_hbridge, 0, 0);
            }
            SDK_DelayAtLeastUs(16000U, SystemCoreClock);
        }
    }
}
