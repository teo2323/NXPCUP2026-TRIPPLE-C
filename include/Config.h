#ifndef CONFIG_H
#define CONFIG_H

// CAM 1 (dynamic status LED)
#define CAM1_COLOR_STRAIGHT_R        0
#define CAM1_COLOR_STRAIGHT_G        255
#define CAM1_COLOR_STRAIGHT_B        0

#define CAM1_COLOR_ANTICIPATE_R      255
#define CAM1_COLOR_ANTICIPATE_G      200
#define CAM1_COLOR_ANTICIPATE_B      0

#define CAM1_COLOR_CURVE_R           255
#define CAM1_COLOR_CURVE_G           0
#define CAM1_COLOR_CURVE_B           0

#define STEERING_KP_STRAIGHT       1.5
#define STEERING_KD_STRAIGHT       12

#define STEERING_KP_CURVE          4.5
#define STEERING_KD_CURVE          1.5

// Dual Steering Error Deadbands
#define STEERING_DEADBAND_STRAIGHT   1.0   // Wide deadband: stops straight-line servo chatter (deg)
#define STEERING_DEADBAND_CURVE      1.0   // Narrow deadband: high precision along the arc (deg)

// Track line angle hysteresis thresholds in degrees for transitioning between Straight and Curve zone
#define CURVE_ENTER_ANGLE_DEG      9.6       // good values = 20;E_DEG      15
#define CURVE_EXIT_ANGLE_DEG       8.7      // good values = 10;
#define CURVE_EXIT_FRAMES          2      // Consecutive frames below EXIT threshold required to return to straight

// Decay factor for steering angle when no lines are detected
#define DECAY_FACTOR          0.5

// Maximum consecutive frames with no detected track lines before stopping the motors (5 frames)
#define NO_LINE_MAX_LOST_FRAMES 5U

// CAM  2 (blue) Look-Ahead & Racing Line parameters
#define ANTICIPATION_ENTER_ANGLE_DEG 8  // Threshold for cam2 to detect an approaching curve
#define PRE_CURVE_SPEED 85              // Speed to drop to when curve is anticipated
// #define OFFSET_SMOOTH_ENTER          0.25   // Smooth gradual offset transition into curve
// #define OFFSET_SMOOTH_EXIT           0.28   // Smooth gradual offset transition outside curve

// CAM2 -> CAM1
#define RACING_LINE_OFFSET_PX       5  // Pixels to offset from center to stay near the outer line

// BOTH
// Camera perspective convergence offset in degrees for single-line angle compensation
#define PERSPECTIVE_ANGLE_OFFSET   5.0   // Inward tilt caused by camera vanishing point perspective

// Physical steering limits (KEPT FROM bootcampWorkspace)
#define STEERING_LIMIT_RIGHT  75
#define STEERING_LIMIT_LEFT  -75

// Steering angle offset (KEPT FROM bootcampWorkspace)
#define STEERING_OFFSET      15
// Base wheel speeds
#define SPEED_RIGHT           100
#define SPEED_LEFT            100
// Corner braking: how much BOTH wheels slow down in curves.
// Higher = more overall speed reduction in turns.
#define CORNER_BRAKE_COEFF     0.80
// Differential drive: how much EXTRA the inner wheel brakes vs the outer.
// Formula: outer = SPEED - |angle|*CORNER_BRAKE + |angle|*DIFF
//          inner = SPEED - |angle|*CORNER_BRAKE - |angle|*DIFF 
#define DIFF_SPEED_COEFF       0.85


// Minimum slope (|dy/dx|) for a vector to be classified as vertical.
// Higher = stricter (only nearly-vertical lines pass).
// 1.5 means dy must be at least 1.5x dx (~56° from horizontal).
#define VERTICAL_MIN_SLOPE     0.375

#endif