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

volatile uint32_t g_systick_ms = 0;
lpi2c_master_edma_handle_t g_lpi2c2_edma_handle;

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
    double abs_error = fabs(error);
    double abs_curve_angle = fabs(curve_angle);

    /* Hysteresis band for transitioning between Straight and Curve zones based on track line angle:
     * - Enter curve mode if |curve_angle| > CURVE_ENTER_ANGLE_DEG.
     * - Stay in curve mode until |curve_angle| < CURVE_EXIT_ANGLE_DEG
     *   for CURVE_EXIT_FRAMES consecutive frames. */
    if (*is_in_curve)
    {
        if (abs_curve_angle < CURVE_EXIT_ANGLE_DEG)
        {
            (*straight_frames)++;
            if (*straight_frames >= CURVE_EXIT_FRAMES)
            {
                *is_in_curve = false;
                *straight_frames = 0;
            }
        }
        else
        {
            *straight_frames = 0;
        }
    }
    else
    {
        if (abs_curve_angle > CURVE_ENTER_ANGLE_DEG)
        {
            *is_in_curve = true;
            *straight_frames = 0;
        }
    }

    /* Dual Deadband & Response Logic (Combination of Idea A & C) */
    if (!(*is_in_curve))
    {
        /* Straight Zone: wide deadband; return 0 to lock servo centered */
        if (abs_error <= STEERING_DEADBAND_STRAIGHT)
        {
            *previous_error = error;
            return 0.0;
        }
    }
    else
    {
        /* Curve Zone: tight deadband; hold current curvature arc */
        if (abs_error <= STEERING_DEADBAND_CURVE)
        {
            *previous_error = error;
            return last_steering_angle; // Keep steady turn without servo oscillation
        }
    }

    /* D term: calculated from error difference */
    double derivative = error - *previous_error;
    *previous_error = error;

    /* 2-Zone Gain Selection: Straight vs Curve */
    double kp = (*is_in_curve) ? STEERING_KP_CURVE : STEERING_KP_STRAIGHT;
    double kd = (*is_in_curve) ? STEERING_KD_CURVE : STEERING_KD_STRAIGHT;

    /* Variable P + D output */
    double steer_angle = (kp * error) + (kd * derivative);

    if (steer_angle > STEERING_LIMIT_RIGHT)
        steer_angle = STEERING_LIMIT_RIGHT;
    if (steer_angle < STEERING_LIMIT_LEFT)
        steer_angle = STEERING_LIMIT_LEFT;

    return steer_angle;
}
typedef enum
{
    TRACK_STATE_STRAIGHT = 0,
    TRACK_STATE_ANTICIPATING,
    TRACK_STATE_IN_CURVE
} track_motion_state_t;

int main(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t num_vectors;
    uint16_t vectors2[MAX_VECTORS * 4];
    size_t num_vectors2;

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

    /* Initialize the I2C EDMA bus once for both cameras */
    pixy_init_bus(LPI2C2, &g_lpi2c2_edma_handle, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &g_lpi2c2_edma_handle);
    pixy_set_led(&cam1, CAM1_COLOR_STRAIGHT_R, CAM1_COLOR_STRAIGHT_G, CAM1_COLOR_STRAIGHT_B); // Initial state: straight

    pixy_t cam2;
    pixy_init(&cam2, LPI2C2, 0x55U, &g_lpi2c2_edma_handle);
    pixy_set_led(&cam2, 0, 0, 255); // Blue LED indicates active mode pt a doua camera

    /* Initial base drive speed and straight steering */
    HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);
    Steer(0 + STEERING_OFFSET);

    double last_steering_angle = 0.0;
    double previous_error = 0.0;                                 // D term: stores last frame's error
    bool is_in_curve = false;                                    // Hysteresis state: true if in curve zone
    bool approaching_curve = false;                              // State flag: true if cam2 anticipates a curve
    double desired_target_offset = 0.0;                          // Racing line offset goal
    double current_target_offset = 0.0;                          // Smoothed offset applied to camera
    uint8_t straight_frames = 0;                                 // Counter for consecutive frames below exit threshold
    uint8_t no_line_lost_frames = 0;                             // Counter for consecutive frames with no detected track lines
    bool motors_stopped = false;                                 // State flag: true when no lines detected for > NO_LINE_MAX_LOST_FRAMES frames
    track_motion_state_t last_cam1_state = TRACK_STATE_STRAIGHT; // Track Camera 1 LED state
    bool reached_peak = false;                                   // Memorează dacă am trecut de apexul virajului

    while (1)
    {
        /* Camera 2: Look-ahead for anticipation and racing line */
        if (pixy_get_vectors(&cam2, vectors2, MAX_VECTORS, &num_vectors2) == kStatus_Success)
        {
            dual_line_detection_result_t det2;
            // Target offset is 0 for cam2 since it's just looking for the raw curve angle
            detection_process_dual_lines(vectors2, num_vectors2, 0.0, &det2);

            if (det2.valid_vectors > 0)
            {
                double abs_cam2_angle = fabs(det2.curve_angle);

                /* 1. FAZA DE INTRARE: dacă depășește 8° și nu am început deja manevra */
                if (!approaching_curve && abs_cam2_angle > ANTICIPATION_ENTER_ANGLE_DEG)
                {
                    approaching_curve = true;
                    reached_peak = false; // Resetăm vârful pentru noul viraj

                    // Decalăm spre exterior (racing line)
                    if (det2.curve_angle > 0)
                    {
                        desired_target_offset = RACING_LINE_OFFSET_PX; // Dreapta -> decalează stânga (+offset)
                    }
                    else
                    {
                        desired_target_offset = -RACING_LINE_OFFSET_PX; // Stânga -> decalează dreapta (-offset)
                    }
                }

                /* 2. FAZA DE APEX (VÂRF): dacă suntem în viraj și unghiul depășește pragul de vârf */
                if (approaching_curve)
                {
                    if (abs_cam2_angle >= 50)
                    { // Prag de vârf (ajustabil: 25° - 30°)
                        reached_peak = true;
                    }

                    /* 3. FAZA DE IEȘIRE:
                     * Cazul A: Am atins vârful și unghiul a coborât sub 15° -> recentrare timpurie!
                     * Cazul B: Curba a fost mai lină (sub 25°), dar unghiul a coborât sub 8° -> recentrare normală */
                    if ((reached_peak && abs_cam2_angle < 20) || (abs_cam2_angle < ANTICIPATION_ENTER_ANGLE_DEG))
                    {
                        approaching_curve = false;
                        reached_peak = false;
                        desired_target_offset = 0.0; // Revine la centru!
                    }
                }
            } else {
                /* No valid track vectors ahead (intersection/crossing or lines ending) -> force center */
                approaching_curve = false;
                desired_target_offset = 0.0;
            }
        }

        /* Smooth the transition: gradual when entering curve, rapid when recentering on exit */
        current_target_offset += (desired_target_offset - current_target_offset);

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success)
        {

            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, current_target_offset, &det);

            /* Display line detection status and coordinates */
            if (det.left_line_present)
            {
                PRINTF("Left Line: DETECTED  | Start: (%u, %u), End: (%u, %u)\r\n",
                       det.left_line.vector.x0, det.left_line.vector.y0,
                       det.left_line.vector.x1, det.left_line.vector.y1);
            }
            else
            {
                PRINTF("Left Line: NOT DETECTED\r\n");
            }

            if (det.right_line_present)
            {
                PRINTF("Right Line: DETECTED | Start: (%u, %u), End: (%u, %u)\r\n",
                       det.right_line.vector.x0, det.right_line.vector.y0,
                       det.right_line.vector.x1, det.right_line.vector.y1);
            }
            else
            {
                PRINTF("Right Line: NOT DETECTED\r\n");
            }

            bool lines_detected = (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present));

            if (lines_detected)
            {
                no_line_lost_frames = 0;
                motors_stopped = false;

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
            // else {
            //     /* 0 track lines detected -> search for horizontal turn-track fallback vector */
            //     turn_track_result_t turn;
            //     if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
            //         /* Horizontal fallback vector indicates a turn in progress */
            //         double steer_angle = compute_variable_pid(turn.steering_angle, turn.steering_angle, &previous_error, last_steering_angle, &is_in_curve, &straight_frames);
            //         Steer(steer_angle + STEERING_OFFSET);
            //         last_steering_angle = steer_angle;
            //     }
            // }
            else
            {
                /* Case: No track lines detected -> check if lost frames exceeded threshold */
                no_line_lost_frames++;
                if (no_line_lost_frames >= NO_LINE_MAX_LOST_FRAMES)
                {
                    motors_stopped = true;
                }

                /* No vectors detected -> gently decay angle using DECAY_FACTOR */
                last_steering_angle *= DECAY_FACTOR;
                if (fabs(last_steering_angle) < 1.0)
                {
                    last_steering_angle = 0.0;
                    previous_error = 0.0;
                    is_in_curve = false;
                    straight_frames = 0;
                }
                Steer(last_steering_angle + STEERING_OFFSET);
            }

            if (motors_stopped)
            {
                /* When no track lines are detected for more than NO_LINE_MAX_LOST_FRAMES frames, stop the motors */
                HbridgeSpeed(&g_hbridge, 0, 0);
            }
            else
            {
                int16_t current_speed_left, current_speed_right;

                if (is_in_curve)
                {
                    double abs_angle = fabs(last_steering_angle);
                    double corner_brake = abs_angle * CORNER_BRAKE_COEFF;

                    // if (approaching_curve) {
                    //     /* Anticipated curve: Racing Line approach, symmetrical braking, no drift */
                    //     current_speed_left  = (int16_t)(SPEED_LEFT  - corner_brake);
                    //     current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake);
                    // } else {
                    //     /* Unanticipated curve: Fallback to aggressive differential drift */

                    double diff = abs_angle * DIFF_SPEED_COEFF;

                    if (last_steering_angle >= 0.0)
                    {
                        /* Right turn: left is outer, right is inner */
                        current_speed_left = (int16_t)(SPEED_LEFT - corner_brake + diff);
                        current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake - diff);
                    }
                    else
                    {
                        /* Left turn: right is outer, left is inner */
                        current_speed_left = (int16_t)(SPEED_LEFT - corner_brake - diff);
                        current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake + diff);
                    }
                    // }

                    // current_speed_left  = (int16_t)(SPEED_LEFT  - corner_brake);
                    // current_speed_right = (int16_t)(SPEED_RIGHT - corner_brake);

                    /* Prevent motor stalling in sharp curves by enforcing a minimum speed */
                    int16_t min_speed = 30; // 30% duty cycle is usually enough to overcome static friction
                    if (current_speed_left < min_speed)
                        current_speed_left = min_speed;
                    if (current_speed_right < min_speed)
                        current_speed_right = min_speed;
                }
                else if (approaching_curve)
                {
                    /* Straight section but curve anticipated: slow down symmetrically */
                    current_speed_left = PRE_CURVE_SPEED;
                    current_speed_right = PRE_CURVE_SPEED;
                }
                else
                {
                    /* Straight section: maintain full symmetric base speed */
                    current_speed_left = SPEED_LEFT;
                    current_speed_right = SPEED_RIGHT;
                }

                HbridgeSpeed(&g_hbridge, current_speed_left, current_speed_right);
            }

            /* Update Camera 1 LED color based on trajectory state (straight, anticipating, in curve) */
            track_motion_state_t current_cam1_state;
            if (is_in_curve)
            {
                current_cam1_state = TRACK_STATE_IN_CURVE;
            }
            else if (approaching_curve)
            {
                current_cam1_state = TRACK_STATE_ANTICIPATING;
            }
            else
            {
                current_cam1_state = TRACK_STATE_STRAIGHT;
            }

            if (current_cam1_state != last_cam1_state)
            {
                last_cam1_state = current_cam1_state;
                switch (current_cam1_state)
                {
                case TRACK_STATE_STRAIGHT:
                    pixy_set_led(&cam1, CAM1_COLOR_STRAIGHT_R, CAM1_COLOR_STRAIGHT_G, CAM1_COLOR_STRAIGHT_B);
                    break;
                case TRACK_STATE_ANTICIPATING:
                    pixy_set_led(&cam1, CAM1_COLOR_ANTICIPATE_R, CAM1_COLOR_ANTICIPATE_G, CAM1_COLOR_ANTICIPATE_B);
                    break;
                case TRACK_STATE_IN_CURVE:
                    pixy_set_led(&cam1, CAM1_COLOR_CURVE_R, CAM1_COLOR_CURVE_G, CAM1_COLOR_CURVE_B);
                    break;
                }
            }
        }
        else
        {
            /* Camera read failed: check if lost frames exceeded threshold */
            no_line_lost_frames++;
            if (no_line_lost_frames >= NO_LINE_MAX_LOST_FRAMES)
            {
                motors_stopped = true;
                HbridgeSpeed(&g_hbridge, 0, 0);
            }
        }
    }
}
