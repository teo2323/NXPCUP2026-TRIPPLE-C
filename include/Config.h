#ifndef CONFIG_H
#define CONFIG_H

#include "wifi.h"

/* ========================================================================= */
/* Default Values — used to initialise the global variables in wifi.c        */
/* These match the original hand-tuned constants exactly.                    */
/* ========================================================================= */

// 2-Zone Variable Steering PID control coefficients
#define DEFAULT_STEERING_KP_STRAIGHT        1.0
#define DEFAULT_STEERING_KD_STRAIGHT       10.0

#define DEFAULT_STEERING_KP_CURVE           2.0
#define DEFAULT_STEERING_KD_CURVE           2.0

// Dual Steering Error Deadbands (deg)
#define DEFAULT_STEERING_DEADBAND_STRAIGHT  2.5   // Wide: stops straight-line servo chatter
#define DEFAULT_STEERING_DEADBAND_CURVE     1.0   // Narrow: high precision along the arc

// Curve hysteresis thresholds
#define DEFAULT_CURVE_ENTER_ANGLE_DEG      35.0
#define DEFAULT_CURVE_EXIT_ANGLE_DEG        5.0   // good values = 10
#define DEFAULT_CURVE_EXIT_FRAMES           2.0   // Consecutive frames below EXIT threshold

// Camera perspective convergence offset (deg)
#define DEFAULT_PERSPECTIVE_ANGLE_OFFSET    5.0   // Inward tilt from vanishing-point perspective

// Physical steering limits (deg)
#define DEFAULT_STEERING_LIMIT_RIGHT       65.0
#define DEFAULT_STEERING_LIMIT_LEFT       -65.0

// Steering angle calibration offset (deg)
#define DEFAULT_STEERING_OFFSET            15.0

// Base wheel speeds (%)
#define DEFAULT_SPEED_RIGHT               100.0
#define DEFAULT_SPEED_LEFT                100.0

// Corner braking: how much BOTH wheels slow down in curves
// Higher = more overall speed reduction in turns
#define DEFAULT_CORNER_BRAKE_COEFF          1.7

// Differential drive: how much EXTRA the inner wheel brakes vs the outer
// Formula: outer = SPEED - |angle|*CORNER_BRAKE + |angle|*DIFF
//          inner = SPEED - |angle|*CORNER_BRAKE - |angle|*DIFF
#define DEFAULT_DIFF_SPEED_COEFF            0.38

// Minimum slope (|dy/dx|) for a vector to be classified as vertical
// Higher = stricter (only nearly-vertical lines pass)
// 0.375 means dy must be at least 0.375x dx
#define DEFAULT_VERTICAL_MIN_SLOPE          0.375

// Decay factor for steering angle when no lines are detected
#define DEFAULT_DECAY_FACTOR                0.5

// Maximum consecutive frames with no detected track lines before stopping motors
#define DEFAULT_NO_LINE_MAX_LOST_FRAMES     5.0

/* ========================================================================= */
/* Runtime Aliases — all existing code uses these names unchanged.           */
/* Each macro now resolves to a volatile global variable that can be         */
/* updated live over WiFi from the ESP32 web interface.                      */
/* ========================================================================= */

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

// Detection
#define VERTICAL_MIN_SLOPE          g_vertical_min_slope
#define DECAY_FACTOR                g_decay_factor
#define NO_LINE_MAX_LOST_FRAMES     ((uint32_t)g_no_line_max_lost_frames)

#endif /* CONFIG_H */