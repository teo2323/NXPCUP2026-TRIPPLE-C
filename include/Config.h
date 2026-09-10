#ifndef CONFIG_H
#define CONFIG_H

#include "wifi.h"

// Hardcoded Autonomous Vehicle Parameters (autov2)
#define AUTO_ENGINE_ENABLED    true
#define AUTO_MOTOR_SPEED       60.0

// Hardcoded Steering control coefficients
#define STEERING_P             0.80
#define STEERING_D             0.20

// Physical steering limits
#define STEERING_LIMIT_RIGHT   45.0
#define STEERING_LIMIT_LEFT   -45.0

// Wheel speeds
#define SPEED_RIGHT            40
#define SPEED_LEFT            -40

#define DECAY_FACTOR           0.90  // Decay factor for steering angle when no lines are detected

#endif