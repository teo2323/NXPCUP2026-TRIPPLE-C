#ifndef CONFIG_H
#define CONFIG_H

// Default Steering control coefficients (PID gains)
#define STEERING_P_RIGHT      1.7
#define STEERING_P_LEFT       1.7

// Default Steering derivative gains (dampen rapid angle changes)
#define STEERING_D_RIGHT      0.2
#define STEERING_D_LEFT       0.2

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  67
#define STEERING_LIMIT_LEFT  -67

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      16

// Wheel speeds (Constant speed 60)
#define SPEED_RIGHT           50
#define SPEED_LEFT            50

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.9

#endif
