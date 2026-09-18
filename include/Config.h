#ifndef CONFIG_H
#define CONFIG_H

// 2-Zone Variable Steering PID control coefficients
// Zone 1: Straight line (|error| <= CURVE_ERROR_THRESHOLD)
#define STEERING_KP_STRAIGHT       1.5
#define STEERING_KD_STRAIGHT       0.9

// Zone 2: Curve / Sharp turn (|error| > CURVE_ERROR_THRESHOLD or sharp turn)
#define STEERING_KP_CURVE          4.5
#define STEERING_KD_CURVE          1.2

// Error threshold in degrees to transition between Straight and Curve zone
#define CURVE_ERROR_THRESHOLD      8.5

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  68
#define STEERING_LIMIT_LEFT  -63

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      13

// Base wheel speeds
#define SPEED_RIGHT           90
#define SPEED_LEFT            90
#define SHARP_TURN_SPEED_COEFF 0.2
// Sharp turn angle detection threshold in degrees
#define SHARP_TURN_ANGLE_THRESHOLD 25

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.9

#endif