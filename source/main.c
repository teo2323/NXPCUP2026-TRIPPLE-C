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
#include "ultrasonic.h" // Include senzorul ultrasonic

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
    Ultrasonic_Init(); // Inițializează senzorul ultrasonic

    #define OBSTACLE_STOP_DIST_CM    20.0f /* Obstacle stop threshold distance (in cm) */
    #define OBSTACLE_CONFIRM_COUNT   2     /* Number of consecutive readings below threshold required to confirm stop */

    static uint32_t ultrasonic_print_counter = 0;
    static uint8_t  obstacle_detected_count  = 0;

    static bool last_printed_engine_state = false;

    while (1)
    {
        Wifi_Process_Rx(); // Procesează datele primite de la modulul Wi-Fi

        // Measure distance in front of the vehicle using non-blocking ultrasonic sensor 
        float distance_cm = Ultrasonic_ReadDistanceCm();

        if (distance_cm >= 2.0f && distance_cm <= OBSTACLE_STOP_DIST_CM) {
            if (++obstacle_detected_count >= OBSTACLE_CONFIRM_COUNT) {
                obstacle_detected_count = OBSTACLE_CONFIRM_COUNT; // Prevenim overflow
                PRINTF("[OBSTACLE] Obstacle detected at %d cm! Braking...\r\n", (int)distance_cm);
                HbridgeBrake(&g_hbridge);
                pixy_set_led(&cam1, 255, 0, 0); // Pixy Red LED: STOPPED AT OBSTACLE
                Wifi_Process_Rx(); // Make sure ESP commands are still processed while braking
                continue; // Menținem frâna fără a bloca bucla
            }
        } else {
            if (obstacle_detected_count >= OBSTACLE_CONFIRM_COUNT) {
                PRINTF("[OBSTACLE] Path clear (%d cm)! Resuming movement...\r\n", (int)distance_cm);
                pixy_set_led(&cam1, 0, 255, 0); // Pixy Green LED: Resumed
            }
            // Reset counter if the path is clear or out-of-range timeout
            obstacle_detected_count = 0;
        }

        // Periodic debug output over serial (throttled every 20 iterations) 
        if (++ultrasonic_print_counter >= 20U) {
            ultrasonic_print_counter = 0U;
            
            if (distance_cm >= 0.0f) {
                PRINTF("[ULTRASONIC] Distance: %d cm\r\n", (int)distance_cm);
            } else if (distance_cm == -1.0f) {
                PRINTF("[ULTRASONIC Err -1] Echo stayed LOW (Trigger not sent or sensor not powered)\r\n");
            } else if (distance_cm == -2.0f) {
                PRINTF("[ULTRASONIC Err -2] Echo stayed HIGH too long (out of range timeout)\r\n");
            } else if (distance_cm == -3.0f) {
                PRINTF("[ULTRASONIC Err -3] Echo duration was 0 us ERROR\r\n");
            } else if (distance_cm == -4.0f) {
                PRINTF("[ULTRASONIC Err -4] Echo was already HIGH before Trigger pulse\r\n");
            }
        }
        
        /* Maintain continuous dynamic motor speed rate */
        if (g_engine_enabled != last_printed_engine_state) {
            last_printed_engine_state = g_engine_enabled;
        }

        static uint32_t g_horizontal_vector_count = 0U;
        static bool last_engine_state = false;
        static bool g_horiz_delay_in_progress = false;
        static uint32_t g_horiz_delay_start_cycles = 0U;
        static bool g_horiz_speed_reduced = false;

        /* Reset counter, delay, and speed reduction state on motor start (rising edge of g_engine_enabled) */
        if (g_engine_enabled && !last_engine_state) {
            g_horizontal_vector_count = 0U;
            g_horiz_delay_in_progress = false;
            g_horiz_speed_reduced = false;
            pixy_set_led(&cam1, 0, 255, 0); // Pixy Green LED: Active engine
        }
        last_engine_state = g_engine_enabled;

        /* Non-blocking state machine for 1-second delay then speed reduction to current_speed / 2 */
        if (g_horiz_delay_in_progress) {
            uint32_t now_cycles = MSDK_GetCpuCycleCount();
            /* 1 second = SystemCoreClock CPU cycles */
            if ((now_cycles - g_horiz_delay_start_cycles) >= SystemCoreClock) {
                PRINTF("[PIXY HORIZONTAL] Non-blocking 1s delay finished! Reducing speed to current_speed/2...\r\n");
                g_horiz_delay_in_progress = false;
                g_horiz_speed_reduced = true;
                pixy_set_led(&cam1, 255, 255, 0); // Pixy Yellow LED: Speed reduced
            }
        }

        /* Determine motor speed: reduce to current_speed / 2 if 1-second delay after > 2 horizontal vectors has elapsed */
        if (g_horiz_speed_reduced) {
            current_speed = g_engine_enabled ? ((int)g_motor_speed / 2) : 0;
        } else {
            current_speed = g_engine_enabled ? (int)g_motor_speed : 0;
        }
        HbridgeSpeed(&g_hbridge, current_speed, current_speed);

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            Wifi_Process_Rx();

            uint32_t horiz_in_frame = (uint32_t)detection_count_horizontal_vectors(vectors, num_vectors);
            if (horiz_in_frame > 0) {
                g_horizontal_vector_count += horiz_in_frame;
                PRINTF("[PIXY HORIZONTAL] Detectat %u linie/linii orizontala/e in cadrul curent! Total acumulat: %u\r\n",
                       (unsigned)horiz_in_frame, (unsigned)g_horizontal_vector_count);

                /* If > 2 horizontal vectors detected and delay not yet started/reduced, start non-blocking 1-second delay */
                if (g_engine_enabled && g_horizontal_vector_count > 2U && !g_horiz_delay_in_progress && !g_horiz_speed_reduced) {
                    PRINTF("[PIXY HORIZONTAL] More than 2 horizontal vectors detected (%u)! Starting non-blocking 1s timer before speed reduction...\r\n",
                           (unsigned)g_horizontal_vector_count);
                    g_horiz_delay_in_progress = true;
                    g_horiz_delay_start_cycles = MSDK_GetCpuCycleCount();
                    pixy_set_led(&cam1, 0, 255, 255); // Pixy Cyan LED: Timer running
                }
            }

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

                    //PRINTF("Vede ambii vectori\n");
                }
                else {

                    /* Find slope of the visible track line */
                    const line_track_t *visible_line = det.left_line_present ? &det.left_line : &det.right_line;

                    double slope = visible_line->inverse_slope;

                    int slope_x100 = (int)(slope * 100.0);
                    int abs_x100 = slope_x100 < 0 ? -slope_x100 : slope_x100;

                    // if (det.left_line_present) {
                    //     PRINTF("Linia stanga prezenta!\r\n");
                    // } else {
                    //     PRINTF("Linia dreapta prezenta!\r\n");
                    // }

                    // if (slope < 0 && slope_x100 / 100 == 0) {
                    //     PRINTF("Slope: -0.%02d\r\n", abs_x100 % 100);
                    // } else {
                    //     PRINTF("Slope: %d.%02d\r\n", slope_x100 / 100, abs_x100 % 100);
                    // }

                    
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
                    //        (int)turn.center_x, turn.turn_left ? "LEFT" : "RIGHT", (int)steer_angle);
                    // PRINTF("Nu detectez track lines, am gasit o linie orizontala\r\n");
                }
                else {
                    /* No horizontal vector either -> gently decay angle toward straight */
                    last_steering_angle *= g_decay_factor; // Decay factor (dynamic)
                    if (fabs(last_steering_angle) < 1.0) {
                        last_steering_angle = 0.0;
                    }
                    Steer(last_steering_angle);

                    // static uint32_t no_vector_counter = 0;
                    // if (++no_vector_counter >= 100U) {
                    //     no_vector_counter = 0U;
                    //     PRINTF("Nu am gasit niciun vector, caut in continuare...\r\n");
                    //     fflush(stdout);
                    // }

                    PRINTF("[Turn] No turn vector | Decaying angle: %d deg\r\n", (int)last_steering_angle);
                }
            }

            /* Send Telemetry to ESP32 over UART at 10 Hz rate (~every 6 camera frames at 60FPS) */
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
                else
                {
                    turn_track_result_t turn_t;
                    if (detection_detect_turn_track(vectors, num_vectors, &turn_t)) {
                        line_cnt = 0U;
                        which_str = turn_t.turn_left ? "TURN_LEFT" : "TURN_RIGHT";
                    } else {
                        line_cnt = 0U;
                        which_str = "NONE";
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

                Wifi_SendTelemetry(line_cnt, which_str, num_vectors, g_horizontal_vector_count, last_steering_angle, (double)current_speed, g_engine_enabled, 7.40, lx0, ly0, lx1, ly1, rx0, ry0, rx1, ry1);
            }
        }
    }
}