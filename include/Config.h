#ifndef CONFIG_H
#define CONFIG_H

#include "wifi.h"

/* ========================================================================= */
/* Default Configurations & Dynamic Variable Mappings                        */
/* ========================================================================= */

// Default 2-Zone Variable Steering PID control coefficients
#define DEFAULT_STEERING_KP_STRAIGHT       1.5
#define DEFAULT_STEERING_KD_STRAIGHT       0.5
#define DEFAULT_STEERING_KP_CURVE          3.0
#define DEFAULT_STEERING_KD_CURVE          1.2
#define DEFAULT_CURVE_ERROR_THRESHOLD      10.0

// Dynamic coefficients mapped to runtime global variables
#define STEERING_KP_STRAIGHT       g_steering_kp_straight
#define STEERING_KD_STRAIGHT       g_steering_kd_straight
#define STEERING_KP_CURVE          g_steering_kp_curve
#define STEERING_KD_CURVE          g_steering_kd_curve
#define CURVE_ERROR_THRESHOLD      g_curve_error_threshold

// Physical steering limits & calibration offset
#define DEFAULT_STEERING_LIMIT_RIGHT       65.0
#define DEFAULT_STEERING_LIMIT_LEFT        -60.0
#define DEFAULT_STEERING_OFFSET            0.0

#define STEERING_LIMIT_RIGHT       g_steering_limit_right
#define STEERING_LIMIT_LEFT        g_steering_limit_left
#define STEERING_OFFSET            g_steering_offset

// Base wheel speeds & dynamic sharp turn scaling
#define DEFAULT_MOTOR_SPEED                90.0
#define DEFAULT_SHARP_TURN_SPEED_COEFF     0.2
#define DEFAULT_SPEED_LEFT                 40.0
#define DEFAULT_SPEED_RIGHT                40.0

#define MOTOR_SPEED                g_motor_speed
#define SHARP_TURN_SPEED_COEFF     g_sharp_turn_speed_coeff
#define SPEED_LEFT                 g_speed_left
#define SPEED_RIGHT                g_speed_right

// Sharp turn angle detection threshold in degrees
#define DEFAULT_SHARP_TURN_ANGLE_THRESHOLD 30.0
#define SHARP_TURN_ANGLE_THRESHOLD g_sharp_turn_angle_threshold

// Decay factor for steering angle when no lines are detected
#define DEFAULT_DECAY_FACTOR               0.9
#define DECAY_FACTOR                       g_decay_factor

#endif /* CONFIG_H */
