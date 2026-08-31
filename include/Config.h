#ifndef CONFIG_H
#define CONFIG_H

#include "wifi.h"

// Default Steering control coefficients
#define DEFAULT_STEERING_P_RIGHT      0.4
#define DEFAULT_STEERING_P_LEFT       0.4

// Default Steering derivative gains (dampen rapid angle changes)
#define DEFAULT_STEERING_D_RIGHT      0.2
#define DEFAULT_STEERING_D_LEFT       0.2

// Dynamic Steering control coefficients (mapped to runtime global variables)
#define STEERING_P_RIGHT      g_steering_p_right
#define STEERING_P_LEFT       g_steering_p_left
#define STEERING_D_RIGHT      g_steering_d_right
#define STEERING_D_LEFT       g_steering_d_left

// Physical steering limits
#define STEERING_LIMIT_RIGHT  45
#define STEERING_LIMIT_LEFT  -60

// Steering angle offset
#define STEERING_OFFSET      -10

// Wheel speeds
#define SPEED_RIGHT           40
#define SPEED_LEFT            -40

#endif