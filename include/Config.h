#ifndef CONFIG_H
#define CONFIG_H

// 2-Zone Variable Steering PID control coefficients
// Zone 1: Straight line
#define STEERING_KP_STRAIGHT       1
#define STEERING_KD_STRAIGHT       10

// Zone 2: Curve (driven by CURVE PID)
#define STEERING_KP_CURVE          2.0
#define STEERING_KD_CURVE          2.0

// Dual Steering Error Deadbands
#define STEERING_DEADBAND_STRAIGHT   2.5   // Wide deadband: stops straight-line servo chatter (deg)
#define STEERING_DEADBAND_CURVE      1.0   // Narrow deadband: high precision along the arc (deg)

// Track line angle hysteresis thresholds in degrees for transitioning between Straight and Curve zone
#define CURVE_ENTER_ANGLE_DEG          15     // good values = 20;E_DEG      15
#define CURVE_EXIT_ANGLE_DEG       5 // good values = 10;
#define CURVE_EXIT_FRAMES          2     // Consecutive frames below EXIT threshold required to return to straight

// Camera 2 Look-Ahead & Racing Line parameters
#define ANTICIPATION_ENTER_ANGLE_DEG 15.0  // Threshold for cam2 to detect an approaching curve
#define RACING_LINE_OFFSET_PX        15.0  // Pixels to offset from center to stay near the outer line
#define PRE_CURVE_SPEED              70    // Speed to drop to when curve is anticipated


// Camera perspective convergence offset in degrees for single-line angle compensation
#define PERSPECTIVE_ANGLE_OFFSET   5.0   // Inward tilt caused by camera vanishing point perspective

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  65
#define STEERING_LIMIT_LEFT  -65

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      15
// Base wheel speeds
#define SPEED_RIGHT           100
#define SPEED_LEFT            100
// Corner braking: how much BOTH wheels slow down in curves.
// Higher = more overall speed reduction in turns.
#define CORNER_BRAKE_COEFF     1.50

// Differential drive: how much EXTRA the inner wheel brakes vs the outer.
// Formula: outer = SPEED - |angle|*CORNER_BRAKE + |angle|*DIFF
//          inner = SPEED - |angle|*CORNER_BRAKE - |angle|*DIFF
#define DIFF_SPEED_COEFF       0.38


// Minimum slope (|dy/dx|) for a vector to be classified as vertical.
// Higher = stricter (only nearly-vertical lines pass).
// 1.5 means dy must be at least 1.5x dx (~56° from horizontal).
#define VERTICAL_MIN_SLOPE     0.375

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.5

// Maximum consecutive frames with no detected track lines before stopping the motors (5 frames)
#define NO_LINE_MAX_LOST_FRAMES 5U

#endif