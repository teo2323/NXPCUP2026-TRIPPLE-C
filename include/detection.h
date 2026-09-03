#ifndef DETECTION_H_
#define DETECTION_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default Pixy2 Line Tracking Frame Dimensions */
#define PIXY_FRAME_WIDTH               78U
#define PIXY_FRAME_CENTER_X            39U

/* Default Detection & Track Constants */
#define DETECTION_MIN_DY_VERTICAL      8.0
#define DUAL_LINE_HALF_TRACK_DEFAULT   25.0  /* Half track width in pixels (~50px full track) */
#define DETECTION_MIN_DX_HORIZONTAL    15   /* Minimum |dx| in pixels for a vector to be considered horizontal */
#define TURN_TRACK_CLOSED_ANGLE        1.6  /* Raw angle output for closed turn */

/**
 * @brief Pixy Vector structure representing a single detected line vector.
 */
typedef struct {
    uint16_t x0;    /**< Tail / Start X coordinate */
    uint16_t y0;    /**< Tail / Start Y coordinate */
    uint16_t x1;    /**< Head / End X coordinate */
    uint16_t y1;    /**< Head / End Y coordinate */
    uint8_t  index; /**< Vector index or tracking ID */
    uint8_t  flags; /**< Vector status flags */
} pixy_vector_t;

/**
 * @brief Structure containing metrics for a single boundary line (Left or Right).
 */
typedef struct {
    pixy_vector_t vector;        /**< Primary vector for this line */
    bool          detected;      /**< True if line is detected */
    double        angle_deg;     /**< Vector angle in degrees */
    double        inverse_slope; /**< Inverse slope (dx / dy) */
    double        bottom_x;      /**< X coordinate at frame bottom */
    double        top_x;         /**< X coordinate at frame top */
    double        length;        /**< Vector length in pixels */
} line_track_t;

/**
 * @brief Comprehensive dual-line detection result structure.
 */
typedef struct {
    line_track_t  left_line;             /**< Left boundary line */
    line_track_t  right_line;            /**< Right boundary line */
    
    bool          both_lines_present;    /**< True if both left & right lines are detected */
    bool          left_line_present;     /**< True if left line is detected */
    bool          right_line_present;    /**< True if right line is detected */
    
    double        track_center_x;        /**< Estimated track center X at bottom of frame */
    double        center_offset;         /**< Track center deviation from frame center (39) */
    double        track_width;           /**< Measured track width in pixels */
    
    double        steering_angle;        /**< Final recommended steering angle/slope */
    double        avg_slope;             /**< Effective centerline slope */
    size_t        valid_vectors;         /**< Total valid vectors processed */
    
    bool          intersection_detected; /**< True if an intersection/fork was detected */
    bool          sharp_turn_detected;   /**< True if a sharp turn was detected */
} dual_line_detection_result_t;

/**
 * @brief Result structure for horizontal turn-track detection.
 */
typedef struct {
    bool   detected;        /**< True if a horizontal turn vector was found */
    double center_x;        /**< X midpoint of the best horizontal vector */
    double bottom_y;        /**< Y coordinate of the endpoint closest to the car (highest Y) */
    bool   turn_left;       /**< True if turn direction is LEFT, false if RIGHT */
    double steering_angle;  /**< Proportional steering angle based on vector offset from center */
} turn_track_result_t;

/* --- Main Detection API --- */

/**
 * @brief Process raw Pixy vectors to detect dual line tracks and compute steering recommendations.
 */
void detection_process_dual_lines(const uint16_t *raw_vectors,
                                  size_t num_vectors,
                                  dual_line_detection_result_t *result);

/**
 * @brief Detect turn tracks from raw horizontal vectors when no vertical line tracks are visible.
 */
bool detection_detect_turn_track(const uint16_t *raw_vectors,
                                 size_t num_vectors,
                                 turn_track_result_t *result);

/**
 * @brief Count horizontal vectors present in raw vector frame.
 */
size_t detection_count_horizontal_vectors(const uint16_t *raw_vectors,
                                         size_t num_vectors);

#ifdef __cplusplus
}
#endif

#endif /* DETECTION_H_ */
