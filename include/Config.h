#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>

/* ========================================================================= */
/* WiFi Configuration Toggle                                                 */
/* Set to true  -> ESP32 controls start/stop + live parameter tuning         */
/* Set to false -> Robot auto-starts, uses Config.h defaults, no WiFi/UART   */
/* ========================================================================= */
#define WIFI_CONFIG_ENABLED  true
static const bool wifi_config_enabled = WIFI_CONFIG_ENABLED;

#if WIFI_CONFIG_ENABLED
#include "wifi.h"
#endif

/* ========================================================================= */
/* Default Values — used to initialise the global variables in wifi.c        */
/* These match the original hand-tuned constants exactly.                    */
/* ========================================================================= */

// 2-Zone Variable Steering PID control coefficients
#define DEFAULT_STEERING_KP_STRAIGHT        1.5
#define DEFAULT_STEERING_KD_STRAIGHT       10.0

#define DEFAULT_STEERING_KP_CURVE           2.4
#define DEFAULT_STEERING_KD_CURVE           4.0

// Dual Steering Error Deadbands (deg)
#define DEFAULT_STEERING_DEADBAND_STRAIGHT  2.5   // Wide: stops straight-line servo chatter
#define DEFAULT_STEERING_DEADBAND_CURVE     1.0   // Narrow: high precision along the arc

// Curve hysteresis thresholds
#define DEFAULT_CURVE_ENTER_ANGLE_DEG      25.0
#define DEFAULT_CURVE_EXIT_ANGLE_DEG       16.0   // good values = 10
#define DEFAULT_CURVE_EXIT_FRAMES           2.0   // Consecutive frames below EXIT threshold

// Camera perspective convergence offset (deg)
#define DEFAULT_PERSPECTIVE_ANGLE_OFFSET    5.0   // Inward tilt from vanishing-point perspective

// Physical steering limits (deg)
#define DEFAULT_STEERING_LIMIT_RIGHT       65.0  //maybe 76
#define DEFAULT_STEERING_LIMIT_LEFT       -65.0

// Steering angle calibration offset (deg)
#define DEFAULT_STEERING_OFFSET            15.0

// Base wheel speeds (%)
#define DEFAULT_SPEED_RIGHT               100.0
#define DEFAULT_SPEED_LEFT                100.0

// Corner braking: how much BOTH wheels slow down in curves
// Higher = more overall speed reduction in turns
#define DEFAULT_CORNER_BRAKE_COEFF          1.0

// Differential drive: how much EXTRA the inner wheel brakes vs the outer
// Formula: outer = SPEED - |angle|*CORNER_BRAKE + |angle|*DIFF
//          inner = SPEED - |angle|*CORNER_BRAKE - |angle|*DIFF
#define DEFAULT_DIFF_SPEED_COEFF            0.4

// Consecutive curves & hairpin protection:
// - If a new curve is entered within CONSECUTIVE_CURVE_TIME_MS (ms) after exiting a valid curve
//   (one that lasted at least MIN_CURVE_DURATION_MS), or if curve direction reverses (S-curve),
//   or if a single curve exceeds MAX_SINGLE_CURVE_TIME_MS (180-deg hairpin), bypass differential
//   and set both motors to CONSECUTIVE_CURVE_SPEED (%).
#define DEFAULT_CONSECUTIVE_CURVE_TIME_MS   500.0
#define DEFAULT_CONSECUTIVE_CURVE_SPEED     70.0
#define DEFAULT_MIN_CURVE_DURATION_MS       100.0
#define DEFAULT_MAX_SINGLE_CURVE_TIME_MS    450.0

// Minimum slope (|dy/dx|) for a vector to be classified as vertical
// Higher = stricter (only nearly-vertical lines pass)
// 0.375 means dy must be at least 0.375x dx
#define DEFAULT_VERTICAL_MIN_SLOPE          0.375

// Decay factor for steering angle when no lines are detected
#define DEFAULT_DECAY_FACTOR                0.5

// Maximum consecutive frames with no detected track lines before stopping motors
#define DEFAULT_NO_LINE_MAX_LOST_FRAMES     10.0

/* ========================================================================= */
/* Runtime Aliases — all existing code uses these names unchanged.           */
/* Each macro now resolves to a volatile global variable that can be         */
/* updated live over WiFi from the ESP32 web interface.                      */
/* ========================================================================= */

#if WIFI_CONFIG_ENABLED

// Steering PID
#define STEERING_KP_STRAIGHT        g_steering_kp_straight
#define STEERING_KD_STRAIGHT        g_steering_kd_straight
#define STEERING_KP_CURVE           g_steering_kp_curve
#define STEERING_KD_CURVE           g_steering_kd_curve

// Deadbands
#define STEERING_DEADBAND_STRAIGHT  g_steering_deadband_straight
#define STEERING_DEADBAND_CURVE     g_steering_deadband_curve

// Curve hysteresis  (integer uses: cast to preserve original semantics)
#define CURVE_ENTER_ANGLE_DEG       g_curve_enter_angle_deg
#define CURVE_EXIT_ANGLE_DEG        g_curve_exit_angle_deg
#define CURVE_EXIT_FRAMES           ((uint8_t)g_curve_exit_frames)

// Camera
#define PERSPECTIVE_ANGLE_OFFSET    g_perspective_angle_offset

// Steering limits & offset
#define STEERING_LIMIT_RIGHT        g_steering_limit_right
#define STEERING_LIMIT_LEFT         g_steering_limit_left
#define STEERING_OFFSET             g_steering_offset

// Motor speeds (integer uses: cast to preserve original semantics)
#define SPEED_RIGHT                 ((int16_t)g_speed_right)
#define SPEED_LEFT                  ((int16_t)g_speed_left)

// Dynamics
#define CORNER_BRAKE_COEFF          g_corner_brake_coeff
#define DIFF_SPEED_COEFF            g_diff_speed_coeff
#define CONSECUTIVE_CURVE_TIME_MS   ((uint32_t)g_consecutive_curve_time_ms)
#define CONSECUTIVE_CURVE_SPEED     ((int16_t)g_consecutive_curve_speed)
#define MIN_CURVE_DURATION_MS       ((uint32_t)g_min_curve_duration_ms)
#define MAX_SINGLE_CURVE_TIME_MS    ((uint32_t)g_max_single_curve_time_ms)

// Detection
#define VERTICAL_MIN_SLOPE          g_vertical_min_slope
#define DECAY_FACTOR                g_decay_factor
#define NO_LINE_MAX_LOST_FRAMES     ((uint32_t)g_no_line_max_lost_frames)

#else /* WIFI_CONFIG_ENABLED == false: use compile-time constants */

// Steering PID
#define STEERING_KP_STRAIGHT        DEFAULT_STEERING_KP_STRAIGHT
#define STEERING_KD_STRAIGHT        DEFAULT_STEERING_KD_STRAIGHT
#define STEERING_KP_CURVE           DEFAULT_STEERING_KP_CURVE
#define STEERING_KD_CURVE           DEFAULT_STEERING_KD_CURVE

// Deadbands
#define STEERING_DEADBAND_STRAIGHT  DEFAULT_STEERING_DEADBAND_STRAIGHT
#define STEERING_DEADBAND_CURVE     DEFAULT_STEERING_DEADBAND_CURVE

// Curve hysteresis
#define CURVE_ENTER_ANGLE_DEG       DEFAULT_CURVE_ENTER_ANGLE_DEG
#define CURVE_EXIT_ANGLE_DEG        DEFAULT_CURVE_EXIT_ANGLE_DEG
#define CURVE_EXIT_FRAMES           ((uint8_t)DEFAULT_CURVE_EXIT_FRAMES)

// Camera
#define PERSPECTIVE_ANGLE_OFFSET    DEFAULT_PERSPECTIVE_ANGLE_OFFSET

// Steering limits & offset
#define STEERING_LIMIT_RIGHT        DEFAULT_STEERING_LIMIT_RIGHT
#define STEERING_LIMIT_LEFT         DEFAULT_STEERING_LIMIT_LEFT
#define STEERING_OFFSET             DEFAULT_STEERING_OFFSET

// Motor speeds
#define SPEED_RIGHT                 ((int16_t)DEFAULT_SPEED_RIGHT)
#define SPEED_LEFT                  ((int16_t)DEFAULT_SPEED_LEFT)

// Dynamics
#define CORNER_BRAKE_COEFF          DEFAULT_CORNER_BRAKE_COEFF
#define DIFF_SPEED_COEFF            DEFAULT_DIFF_SPEED_COEFF
#define CONSECUTIVE_CURVE_TIME_MS   ((uint32_t)DEFAULT_CONSECUTIVE_CURVE_TIME_MS)
#define CONSECUTIVE_CURVE_SPEED     ((int16_t)DEFAULT_CONSECUTIVE_CURVE_SPEED)
#define MIN_CURVE_DURATION_MS       ((uint32_t)DEFAULT_MIN_CURVE_DURATION_MS)
#define MAX_SINGLE_CURVE_TIME_MS    ((uint32_t)DEFAULT_MAX_SINGLE_CURVE_TIME_MS)

// Detection
#define VERTICAL_MIN_SLOPE          DEFAULT_VERTICAL_MIN_SLOPE
#define DECAY_FACTOR                DEFAULT_DECAY_FACTOR
#define NO_LINE_MAX_LOST_FRAMES     ((uint32_t)DEFAULT_NO_LINE_MAX_LOST_FRAMES)

#endif /* WIFI_CONFIG_ENABLED */

#endif /* CONFIG_H */
