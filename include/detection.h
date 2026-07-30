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
#define PIXY_FRAME_HEIGHT              51U
#define PIXY_FRAME_CENTER_X            39U

/* Default Detection & Track Constants */
#define DETECTION_MIN_DY_DEFAULT       8.0
#define DETECTION_MIN_LEN_DEFAULT      10.0
#define DUAL_LINE_HALF_TRACK_DEFAULT   20.0  /* Half track width in pixels (~40px full track) */
#define DUAL_LINE_STEERING_ANGLE_WEIGHT 1.0  /* Weight factor for line angle heading */
#define DUAL_LINE_OFFSET_WEIGHT        0.8  /* Weight factor for center offset correction */

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
 * @brief Identification of track line side.
 */
typedef enum {
    LINE_SIDE_UNKNOWN = 0,
    LINE_SIDE_LEFT    = 1,
    LINE_SIDE_RIGHT   = 2
} line_side_t;

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
    double        center_offset;         /**< Track center deviation from frame center (39.5) */
    double        track_width;           /**< Measured track width in pixels */
    
    double        steering_angle;        /**< Final recommended steering angle/slope */
    double        avg_slope;             /**< Effective centerline slope */
    size_t        valid_vectors;         /**< Total valid vectors processed */
    
    bool          intersection_detected; /**< True if an intersection/fork was detected */
    bool          sharp_turn_detected;   /**< True if a sharp turn was detected */
} dual_line_detection_result_t;

/**
 * @brief Single-result compatibility structure.
 */
typedef struct {
    double steering_angle;        /**< Calculated steering angle (-1.0 to +1.0 or degrees) */
    double line_offset;           /**< Horizontal offset from frame center (-39.5 to +39.5) */
    double avg_slope;             /**< Average inverse slope (dx / dy) */
    size_t valid_vectors;         /**< Count of vectors passing filter criteria */
    bool   intersection_detected; /**< True if an intersection/fork was detected */
    bool   sharp_turn_detected;   /**< True if a sharp turn was detected */
} detection_result_t;

/* --- Vector Parsing & Helper Functions --- */

size_t detection_parse_vectors(const uint16_t *raw_vectors,
                               size_t num_vectors,
                               pixy_vector_t *out_vectors,
                               size_t max_out);

double detection_vector_length(const pixy_vector_t *vec);
double detection_vector_angle_deg(const pixy_vector_t *vec);
void detection_vector_components(const pixy_vector_t *vec, int *dx, int *dy);
double detection_vector_inverse_slope(const pixy_vector_t *vec);
void detection_print_vector_details(const pixy_vector_t *vec, size_t index);

/* --- Filtering & Classification Algorithms --- */

size_t detection_filter_vertical(const pixy_vector_t *in,
                                 size_t in_count,
                                 pixy_vector_t *out,
                                 double min_dy_pixels);

size_t detection_filter_length(const pixy_vector_t *in,
                               size_t in_count,
                               pixy_vector_t *out,
                               double min_length);

size_t detection_filter_roi(const pixy_vector_t *in,
                            size_t in_count,
                            pixy_vector_t *out,
                            uint16_t min_y,
                            uint16_t max_y);

void detection_classify_left_right(const pixy_vector_t *vectors,
                                   size_t count,
                                   pixy_vector_t *left_vecs,
                                   size_t *left_count,
                                   pixy_vector_t *right_vecs,
                                   size_t *right_count);

bool detection_extract_left_line_track(const pixy_vector_t *vectors,
                                       size_t count,
                                       line_track_t *line);

bool detection_extract_right_line_track(const pixy_vector_t *vectors,
                                        size_t count,
                                        line_track_t *line);

bool detection_extract_line_track(const pixy_vector_t *vectors,
                                  size_t count,
                                  line_side_t side,
                                  line_track_t *line);

/* --- Single & Multi-Vector Line Algorithms --- */

bool detection_find_primary_vector(const pixy_vector_t *vectors,
                                   size_t count,
                                   pixy_vector_t *primary);

size_t detection_calculate_avg_slope_angle(const pixy_vector_t *vectors,
                                            size_t count,
                                            double min_dy,
                                            double *out_angle);

size_t detection_calculate_weighted_steering(const pixy_vector_t *vectors,
                                              size_t count,
                                              double *out_angle);

double detection_calculate_line_offset(const pixy_vector_t *vectors,
                                       size_t count);

bool detection_detect_intersection(const pixy_vector_t *vectors,
                                    size_t count);

bool detection_detect_sharp_turn(const pixy_vector_t *vectors,
                                 size_t count,
                                 double angle_threshold_deg);

/* --- Dual-Line Detection & Steering Algorithms --- */

void detection_calculate_dual_line_steering(const line_track_t *left,
                                             const line_track_t *right,
                                             double half_track_width,
                                             dual_line_detection_result_t *result);

void detection_process_dual_lines(const uint16_t *raw_vectors,
                                  size_t num_vectors,
                                  dual_line_detection_result_t *result);

void detection_process_vectors(const uint16_t *raw_vectors,
                               size_t num_vectors,
                               detection_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* DETECTION_H_ */
