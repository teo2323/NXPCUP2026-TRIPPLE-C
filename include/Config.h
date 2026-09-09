#ifndef CONFIG_H
#define CONFIG_H

#include "wifi.h"

// Default Steering control coefficients
#define DEFAULT_STEERING_P      0.8

// Default Steering derivative gains (dampen rapid angle changes)
#define DEFAULT_STEERING_D      0.2

// Dynamic Steering control coefficients (mapped to runtime global variables)
#define STEERING_P      g_steering_p
#define STEERING_D      g_steering_d

// Physical steering limits
#define STEERING_LIMIT_RIGHT  45
#define STEERING_LIMIT_LEFT  -45

// Wheel speeds
#define SPEED_RIGHT           40
#define SPEED_LEFT            -40

#define DECAY_FACTOR          0.9  // Decay factor for steering angle when no lines are detected

#endif