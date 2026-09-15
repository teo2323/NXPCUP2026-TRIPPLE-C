#ifndef CONFIG_H
#define CONFIG_H

// Unified Steering control coefficients (1 P and 1 D gain)
#define STEERING_KP           3
#define STEERING_KD           0

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  67
#define STEERING_LIMIT_LEFT  -67

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      16

// Base wheel speeds
#define SPEED_RIGHT           90
#define SPEED_LEFT            90
#define SHARP_TURN_SPEED_COEFF 0.2

// Sharp turn angle detection threshold in degrees
#define SHARP_TURN_ANGLE_THRESHOLD 30

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.9

#endif
