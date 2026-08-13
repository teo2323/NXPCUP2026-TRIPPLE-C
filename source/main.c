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
#include "wifi.h" // Include noul header pentru funcțiile Wi-Fi

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

    /* 1. Continuous H-bridge drive speed (Dynamic via Web Server) */
    int current_speed = g_engine_enabled ? (int)g_motor_speed : 0;
    HbridgeSpeed(&g_hbridge, current_speed, current_speed);
    Steer(0.0);
    //TestServo();

    double last_steering_angle = 0.0;
    double previous_error      = 0.0;  // D term: stores last frame's angle

    Wifi_Init(); // Inițializează modulul Wi-Fi

    static bool last_printed_engine_state = false;

    while (1)
    {
        Wifi_Process_Rx(); // Procesează datele primite de la modulul Wi-Fi
        
        /* Maintain continuous dynamic motor speed rate */
        if (g_engine_enabled != last_printed_engine_state) {
            last_printed_engine_state = g_engine_enabled;
            if (g_engine_enabled) {
                PRINTF("\r\n[NXP CONSOLA] ▶️ MOTOARELE AU FOST PORNITE (Viteza aplicata: %d%%)\r\n", (int)g_motor_speed);
            } else {
                PRINTF("\r\n[NXP CONSOLA] ⏸️ MOTOARELE AU FOST OPRITE (Viteza aplicata: 0%%)\r\n");
            }
        }

        current_speed = g_engine_enabled ? (int)g_motor_speed : 0;
        HbridgeSpeed(&g_hbridge, current_speed, current_speed);



        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            /* Print per-frame vector categorization over serial:
             * Shows how each raw vector is classified as VERTICAL LEFT/RIGHT,
             * HORIZONTAL TURN, or REJECTED by the detection pipeline. */
            // detection_debug_vectors(vectors, num_vectors);

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                double raw_steering_angle = 0.0;

                if (det.both_lines_present) {
                    /* Case 1: 2 Track lines detected -> keep same logic for steering */
                    double error = det.steering_angle;

                    /* D term: calculated from the raw error */
                    double derivative = error - previous_error;
                    previous_error    = error;

                    /* P + D combinate: output = P*error + D*derivative */
                    double p_term = (error > 0) ? (STEERING_P_RIGHT * error) : (STEERING_P_LEFT * error);
                    double d_term = (derivative > 0) ? (STEERING_D_RIGHT * derivative) : (STEERING_D_LEFT * derivative);
                    double steer_angle = p_term + d_term;

                    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                    Steer(steer_angle);
                    last_steering_angle = steer_angle;

                    PRINTF("Vede ambii vectori\n");
                }
                else {
                    /* Single line detected case (lines 76-87 commented out):
                    else if (det.left_line_present && !det.right_line_present) {
                        // Case 2: Only LEFT track line detected -> find center by adding 25px
                        double track_center_x = det.left_line.bottom_x + 25.0;
                        double center_offset  = track_center_x - (double)PIXY_FRAME_CENTER_X;
                        raw_steering_angle    = (-1.0 * det.left_line.inverse_slope) + (center_offset * 0.25);
                    }
                    else if (!det.left_line_present && det.right_line_present) {
                        // Case 3: Only RIGHT track line detected -> find center by removing 25px
                        double track_center_x = det.right_line.bottom_x - 25.0;
                        double center_offset  = track_center_x - (double)PIXY_FRAME_CENTER_X;
                        raw_steering_angle    = (-1.0 * det.right_line.inverse_slope) + (center_offset * 0.25);
                    }
                    */

                    /* Find slope of the visible track line */
                    const line_track_t *visible_line = det.left_line_present ? &det.left_line : &det.right_line;

                    double slope = visible_line->inverse_slope;

                    int slope_x100 = (int)(slope * 100.0);
                    int abs_x100 = slope_x100 < 0 ? -slope_x100 : slope_x100;

                    if (det.left_line_present) {
                        PRINTF("Linia stanga prezenta!\r\n");
                    } else {
                        PRINTF("Linia dreapta prezenta!\r\n");
                    }

                    if (slope < 0 && slope_x100 / 100 == 0) {
                        PRINTF("Slope: -0.%02d\r\n", abs_x100 % 100);
                    } else {
                        PRINTF("Slope: %d.%02d\r\n", slope_x100 / 100, abs_x100 % 100);
                    }

                    
                        double steer_angle = (slope >= 0.0) ? (double)STEERING_LIMIT_LEFT : (double)STEERING_LIMIT_RIGHT;
                        Steer(steer_angle);
                        last_steering_angle = steer_angle;
                    
                }
            }
            else {
                /* 0 track lines detected -> search for horizontal turn-track vector */
                turn_track_result_t turn;
                if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
                    /* A horizontal vector found: steer proportionally toward the turn */
                    double error = turn.steering_angle;

                    /* D term: calculated from the raw error */
                    double derivative = error - previous_error;
                    previous_error    = error;

                    /* P*error + D*derivative */
                    double p_term = (error > 0) ? (STEERING_P_RIGHT * error) : (STEERING_P_LEFT * error);
                    double d_term = (derivative > 0) ? (STEERING_D_RIGHT * derivative) : (STEERING_D_LEFT * derivative);
                    double steer_angle = p_term + d_term;

                    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                    Steer(steer_angle);
                    last_steering_angle = steer_angle;

                    // PRINTF("[Turn] Horizontal vector detected | center_x: %d | dir: %s | Steer: %d deg\r\n",
                           //(int)turn.center_x, turn.turn_left ? "LEFT" : "RIGHT", (int)steer_angle);

                    PRINTF("Nu detectez track lines, am gasit o linie orizontala\r\n");
                }
                else {
                    /* No horizontal vector either -> gently decay angle toward straight */
                    last_steering_angle *= 0.8;
                    if (fabs(last_steering_angle) < 1.0) {
                        last_steering_angle = 0.0;
                    }
                    Steer(last_steering_angle);

                    PRINTF("Nu am gasit niciun vector, ma pis pe ea de detectie\r\n");

                    // PRINTF("[Turn] No turn vector | Decaying angle: %d deg\r\n", (int)last_steering_angle);
                }
            }
        }
    }
}