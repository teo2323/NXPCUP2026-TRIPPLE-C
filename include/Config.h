#ifndef CONFIG_H
#define CONFIG_H

// 2-Zone Variable Steering PID control coefficients
// Zone 1: Straight line (|error| <= CURVE_ERROR_THRESHOLD)
//#define STEERING_KP_STRAIGHT       1.5
//#define STEERING_KD_STRAIGHT       0.9

// Zone 2: Curve / Sharp turn (|error| > CURVE_ERROR_THRESHOLD or sharp turn)
//#define STEERING_KP_CURVE          4.0
//#define STEERING_KD_CURVE          1.7

#define STEERING_KP_STRAIGHT       2.0
#define STEERING_KD_STRAIGHT       2.0

#define STEERING_KP_CURVE          STEERING_KP_STRAIGHT
#define STEERING_KD_CURVE          STEERING_KD_STRAIGHT

// Error threshold in degrees to transition between Straight and Curve zone
#define CURVE_ERROR_THRESHOLD      8.5

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  70
#define STEERING_LIMIT_LEFT  -65

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      13

// Base wheel speeds
#define SPEED_RIGHT           100
#define SPEED_LEFT            100
// Corner braking: how much BOTH wheels slow down in curves.
// Higher = more overall speed reduction in turns.
#define CORNER_BRAKE_COEFF     1.25

// Differential drive: how much EXTRA the inner wheel brakes vs the outer.
// Higher = tighter turns (bigger speed gap between wheels).
// Formula: outer = SPEED - |angle|*CORNER_BRAKE + |angle|*DIFF
//          inner = SPEED - |angle|*CORNER_BRAKE - |angle|*DIFF
// Example at angle=20: outer = 90 - 25 + 20 = 85, inner = 90 - 25 - 20 = 45
#define DIFF_SPEED_COEFF       1.0
// Sharp turn angle detection threshold in degrees
#define SHARP_TURN_ANGLE_THRESHOLD 25

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.95

#endif