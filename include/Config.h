#ifndef CONFIG_H
#define CONFIG_H

// Steering control coefficients
#define STEERING_P_RIGHT      0.8  //poate fi schimbata
#define STEERING_P_LEFT       0.8

// Steering derivative gains (dampen rapid angle changes)
#define STEERING_D_RIGHT      0.2
#define STEERING_D_LEFT       0.2

// Physical steering limits
#define STEERING_LIMIT_RIGHT  50
#define STEERING_LIMIT_LEFT  -50

// Steering angle offset
#define STEERING_OFFSET      -10

// Wheel speeds
#define SPEED_RIGHT           40
#define SPEED_LEFT            -40

#endif
