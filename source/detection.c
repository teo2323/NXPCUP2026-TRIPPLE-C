#include "detection.h"
#include "fsl_debug_console.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

size_t detection_parse_vectors(const uint16_t *raw_vectors,
                               size_t num_vectors,
                               pixy_vector_t *out_vectors,
                               size_t max_out)
{
    if (raw_vectors == NULL || out_vectors == NULL) {
        return 0;
    }

    size_t count = (num_vectors < max_out) ? num_vectors : max_out;
    for (size_t i = 0; i < count; i++) {
        out_vectors[i].x0    = raw_vectors[4 * i + 0];
        out_vectors[i].y0    = raw_vectors[4 * i + 1];
        out_vectors[i].x1    = raw_vectors[4 * i + 2];
        out_vectors[i].y1    = raw_vectors[4 * i + 3];
        out_vectors[i].index = (uint8_t)i;
        out_vectors[i].flags = 0;
    }

    return count;
}

double detection_vector_length(const pixy_vector_t *vec)
{
    if (vec == NULL) return 0.0;

    int dx = (int)vec->x1 - (int)vec->x0;
    int dy = (int)vec->y1 - (int)vec->y0;
    return sqrt((double)(dx * dx + dy * dy));
}

double detection_vector_angle_deg(const pixy_vector_t *vec)
{
    if (vec == NULL) return 0.0;

    int dx = (int)vec->x1 - (int)vec->x0;
    int dy = (int)vec->y1 - (int)vec->y0;

    if (dy != 0) {
        return atan2((double)dx, (double)(-dy)) * 180.0 / M_PI;
    } else {
        return (dx >= 0) ? 90.0 : -90.0;
    }
}

double detection_vector_inverse_slope(const pixy_vector_t *vec)
{
    if (vec == NULL) return 0.0;

    double diff_y = (double)vec->y0 - (double)vec->y1;
    if (fabs(diff_y) < 1e-6) {
        return 0.0;
    }
    return ((double)vec->x0 - (double)vec->x1) / diff_y;
}

size_t detection_filter_vertical(const pixy_vector_t *in,
                                 size_t in_count,
                                 pixy_vector_t *out,
                                 double min_dy_pixels)
{
    if (in == NULL || out == NULL) return 0;

    size_t count = 0;
    for (size_t i = 0; i < in_count; i++) {
        double diff_y = (double)in[i].y0 - (double)in[i].y1;
        if (fabs(diff_y) >= min_dy_pixels) {
            out[count++] = in[i];
        }
    }
    return count;
}

/* Helper: X coordinate of the endpoint closest to the car (highest Y value) */
static double get_vector_bottom_x(const pixy_vector_t *vec)
{
    return (vec->y0 > vec->y1) ? (double)vec->x0 : (double)vec->x1;
}

/* Helper: Y coordinate of the endpoint closest to the car (highest Y value) */
static double get_vector_bottom_y(const pixy_vector_t *vec)
{
    return (vec->y0 > vec->y1) ? (double)vec->y0 : (double)vec->y1;
}

void detection_classify_left_right(const pixy_vector_t *vectors,
                                   size_t count,
                                   pixy_vector_t *left_vecs,
                                   size_t *left_count,
                                   pixy_vector_t *right_vecs,
                                   size_t *right_count)
{
    if (left_count)  *left_count = 0;
    if (right_count) *right_count = 0;

    if (vectors == NULL || count == 0 || left_vecs == NULL || right_vecs == NULL) {
        return;
    }

    size_t l_cnt = 0;
    size_t r_cnt = 0;

    for (size_t i = 0; i < count; i++) {
        double bot_x = get_vector_bottom_x(&vectors[i]);
        if (bot_x < PIXY_FRAME_CENTER_X) {
            left_vecs[l_cnt++] = vectors[i];
        } else {
            right_vecs[r_cnt++] = vectors[i];
        }
    }

    if (left_count)  *left_count  = l_cnt;
    if (right_count) *right_count = r_cnt;
}

bool detection_extract_left_line_track(const pixy_vector_t *vectors,
                                       size_t count,
                                       line_track_t *line)
{
    if (line == NULL) return false;

    if (vectors == NULL || count == 0) {
        line->detected      = false;
        line->angle_deg     = 0.0;
        line->inverse_slope = 0.0;
        line->bottom_x      = 0.0;
        line->top_x         = 0.0;
        line->length        = 0.0;
        return false;
    }

    size_t best_idx  = 0;
    double max_score = -1.0;

    for (size_t i = 0; i < count; i++) {
        double bot_x = get_vector_bottom_x(&vectors[i]);

        /* Score left line vector: higher bottom X is closer to frame center (inner left boundary) */
        double score = bot_x;

        if (score > max_score) {
            max_score = score;
            best_idx  = i;
        }
    }

    const pixy_vector_t *best_vec = &vectors[best_idx];
    double len = detection_vector_length(best_vec);

    line->vector        = *best_vec;
    line->detected      = true;
    line->length        = len;
    line->angle_deg     = detection_vector_angle_deg(best_vec);
    line->inverse_slope = detection_vector_inverse_slope(best_vec);

    if (best_vec->y0 >= best_vec->y1) {
        line->bottom_x = (double)best_vec->x0;
        line->top_x    = (double)best_vec->x1;
    } else {
        line->bottom_x = (double)best_vec->x1;
        line->top_x    = (double)best_vec->x0;
    }

    return true;
}

bool detection_extract_right_line_track(const pixy_vector_t *vectors,
                                        size_t count,
                                        line_track_t *line)
{
    if (line == NULL) return false;

    if (vectors == NULL || count == 0) {
        line->detected      = false;
        line->angle_deg     = 0.0;
        line->inverse_slope = 0.0;
        line->bottom_x      = 0.0;
        line->top_x         = 0.0;
        line->length        = 0.0;
        return false;
    }

    size_t best_idx  = 0;
    double max_score = -1.0;

    for (size_t i = 0; i < count; i++) {
        double bot_x = get_vector_bottom_x(&vectors[i]);

        /* Score right line vector: lower bottom X is closer to frame center (inner right boundary) */
        double score = (double)PIXY_FRAME_WIDTH - bot_x;

        if (score > max_score) {
            max_score = score;
            best_idx  = i;
        }
    }

    const pixy_vector_t *best_vec = &vectors[best_idx];
    double len = detection_vector_length(best_vec);

    line->vector        = *best_vec;
    line->detected      = true;
    line->length        = len;
    line->angle_deg     = detection_vector_angle_deg(best_vec);
    line->inverse_slope = detection_vector_inverse_slope(best_vec);

    if (best_vec->y0 >= best_vec->y1) {
        line->bottom_x = (double)best_vec->x0;
        line->top_x    = (double)best_vec->x1;
    } else {
        line->bottom_x = (double)best_vec->x1;
        line->top_x    = (double)best_vec->x0;
    }

    return true;
}

void detection_calculate_dual_line_steering(const line_track_t *left,
                                             const line_track_t *right,
                                             double half_track_width,
                                             dual_line_detection_result_t *result)
{
    if (result == NULL) return;

    bool has_left  = (left != NULL && left->detected);
    bool has_right = (right != NULL && right->detected);

    result->left_line_present  = has_left;
    result->right_line_present = has_right;
    result->both_lines_present = (has_left && has_right);

    if (has_left)  result->left_line  = *left;
    if (has_right) result->right_line = *right;

    if (has_left && has_right) {
        /* CASE 1: Both lines detected -> Track Centerline */
        result->track_width    = right->bottom_x - left->bottom_x;
        result->track_center_x = (left->bottom_x + right->bottom_x) / 2.0;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->avg_slope      = (left->inverse_slope + right->inverse_slope) / 2.0;

        /* Combined steering: inverse slope heading + center offset correction */
        result->steering_angle = (-1.0 * result->avg_slope) + (result->center_offset * 0.25);
    }
    else if (!has_left && has_right) {
        /* CASE 2: Left line MISSING -> virtual center = right_x - half_track_width */
        result->track_center_x = right->bottom_x - half_track_width;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->track_width    = 2.0 * half_track_width;
        result->avg_slope      = right->inverse_slope;

        result->steering_angle = (-1.0 * result->avg_slope) + (result->center_offset * 0.25);
    }
    else if (has_left && !has_right) {
        /* CASE 3: Right line MISSING -> virtual center = left_x + half_track_width */
        result->track_center_x = left->bottom_x + half_track_width;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->track_width    = 2.0 * half_track_width;
        result->avg_slope      = left->inverse_slope;

        result->steering_angle = (-1.0 * result->avg_slope) + (result->center_offset * 0.25);
    }
    else {
        /* CASE 4: Neither line detected */
        result->track_center_x = PIXY_FRAME_CENTER_X;
        result->center_offset  = 0.0;
        result->track_width    = 0.0;
        result->avg_slope      = 0.0;
        result->steering_angle = 0.0;
    }
}

void detection_process_dual_lines(const uint16_t *raw_vectors,
                                  size_t num_vectors,
                                  dual_line_detection_result_t *result)
{
    if (result == NULL) return;

    /* Reset result */
    result->both_lines_present    = false;
    result->left_line_present     = false;
    result->right_line_present    = false;
    result->track_center_x        = PIXY_FRAME_CENTER_X;
    result->center_offset         = 0.0;
    result->track_width           = 0.0;
    result->steering_angle        = 0.0;
    result->avg_slope             = 0.0;
    result->valid_vectors         = 0;
    result->intersection_detected = false;
    result->sharp_turn_detected   = false;

    if (raw_vectors == NULL || num_vectors == 0) return;

    pixy_vector_t parsed[16];
    size_t parsed_count = detection_parse_vectors(raw_vectors, num_vectors, parsed, 16);

    pixy_vector_t filtered[16];
    size_t filtered_count = detection_filter_vertical(parsed, parsed_count, filtered, DETECTION_MIN_DY_VERTICAL);

    result->valid_vectors = filtered_count;
    if (filtered_count == 0) return;

    pixy_vector_t left_vecs[16], right_vecs[16];
    size_t left_cnt = 0, right_cnt = 0;

    detection_classify_left_right(filtered, filtered_count,
                                  left_vecs, &left_cnt,
                                  right_vecs, &right_cnt);

    line_track_t left_line, right_line;
    detection_extract_left_line_track(left_vecs, left_cnt, &left_line);
    detection_extract_right_line_track(right_vecs, right_cnt, &right_line);

    detection_calculate_dual_line_steering(&left_line, &right_line,
                                            DUAL_LINE_HALF_TRACK_DEFAULT,
                                            result);

    if (left_line.detected && right_line.detected) {
        if (fabs(left_line.angle_deg - right_line.angle_deg) > 35.0) {
            result->intersection_detected = true;
        }
    }

    if ((left_line.detected && fabs(left_line.angle_deg) > 35.0) ||
        (right_line.detected && fabs(right_line.angle_deg) > 35.0)) {
        result->sharp_turn_detected = true;
    }
}

bool detection_detect_turn_track(const uint16_t *raw_vectors,
                                 size_t num_vectors,
                                 turn_track_result_t *result)
{
    if (result == NULL) return false;

    /* Reset result */
    result->detected       = false;
    result->center_x       = PIXY_FRAME_CENTER_X;
    result->bottom_y       = 0.0;
    result->turn_left      = false;
    result->steering_angle = 0.0;

    if (raw_vectors == NULL || num_vectors == 0) return false;

    /* Step 1: Parse raw vectors into structured form */
    pixy_vector_t parsed[16];
    size_t parsed_count = detection_parse_vectors(raw_vectors, num_vectors, parsed, 16);
    if (parsed_count == 0) return false;

    /* Step 2: Filter for horizontal-oriented vectors.
     * A vector is horizontal when |dx| > |dy| AND |dx| >= DETECTION_MIN_DX_HORIZONTAL.
     * This rejects vertical track lines and short noise vectors. */
    pixy_vector_t horiz[16];
    size_t horiz_count = 0;

    for (size_t i = 0; i < parsed_count; i++) {
        int dx     = (int)parsed[i].x1 - (int)parsed[i].x0;
        int dy     = (int)parsed[i].y1 - (int)parsed[i].y0;
        int abs_dx = dx < 0 ? -dx : dx;
        int abs_dy = dy < 0 ? -dy : dy;

        /* Must be predominantly horizontal and long enough to be meaningful */
        if (abs_dx > abs_dy && abs_dx >= DETECTION_MIN_DX_HORIZONTAL) {
            horiz[horiz_count++] = parsed[i];
        }
    }

    if (horiz_count == 0) return false;

    /* Step 3: Score each horizontal vector and find the best one.
     * Score = bottom_y: the y-coordinate of the endpoint closest to the car.
     * Higher bottom_y = closer to car = more relevant for turn decision.
     *
     * IMPROVEMENT: A secondary score factor could be vector length (abs_dx),
     * preferring longer, more reliable horizontal lines over short ones. */
    size_t best_idx   = 0;
    double best_score = -1.0;

    for (size_t i = 0; i < horiz_count; i++) {
        double bot_y = get_vector_bottom_y(&horiz[i]);
        double score = bot_y;   /* Primary: highest y = closest to car */

        if (score > best_score) {
            best_score = score;
            best_idx   = i;
        }
    }

    const pixy_vector_t *best = &horiz[best_idx];

    /* Step 4: Extract geometry of the best horizontal vector */
    int dx_raw = (int)best->x1 - (int)best->x0;
    int dy_raw = (int)best->y1 - (int)best->y0;
    double center_x = ((double)best->x0 + (double)best->x1) / 2.0;
    double bot_y    = get_vector_bottom_y(best);

    /* Step 5: Determine turn direction from the slope (dy/dx) of the horizontal vector.
     *
     * The Pixy2 frame has y=0 at the top (far from car) and y=51 at the bottom (close to car).
     * At a track turn, the horizontal end-of-track vector is tilted by the curve:
     *
     *   RIGHT turn: the right side of the vector is closer to the car (higher y).
     *               Going left-to-right (dx > 0), y increases (dy > 0) -> same sign -> RIGHT.
     *
     *   LEFT  turn: the left side of the vector is closer to the car (higher y).
     *               Going left-to-right (dx > 0), y decreases (dy < 0) -> opposite sign -> LEFT.
     *
     * Rule: turn_left = (dx and dy have OPPOSITE signs), i.e. dx * dy < 0.
     * If the vector is perfectly horizontal (dy == 0), fall back to midpoint comparison. */
    bool turn_left;
    if (dy_raw != 0 && dx_raw != 0) {
        turn_left = ((dx_raw > 0 && dy_raw < 0) || (dx_raw < 0 && dy_raw > 0));
    } else {
        /* Fallback: perfectly horizontal vector -> use midpoint vs frame center */
        turn_left = (center_x < (double)PIXY_FRAME_CENTER_X);
    }

    /* Step 6: Output a closed (committed) turn angle toward the detected direction.
     * Rather than a proportional correction, the car commits to a full steering
     * angle in the turn direction so it can recover the track lines after the curve.
     * TURN_TRACK_CLOSED_ANGLE (1.6) x STEERING_P (30) = 48 -> clamped to +-45 in main.c. */
    double steering_angle = turn_left ? -TURN_TRACK_CLOSED_ANGLE : +TURN_TRACK_CLOSED_ANGLE;

    /* Populate result */
    result->detected       = true;
    result->center_x       = center_x;
    result->bottom_y       = bot_y;
    result->turn_left      = turn_left;
    result->steering_angle = steering_angle;

    return true;
}

bool detection_detect_double_horizontal_lines(const uint16_t *raw_vectors, 
                                             size_t num_vectors, 
                                             uint16_t y_tolerance)
{
    if (raw_vectors == NULL || num_vectors < 2) return false;

    pixy_vector_t parsed[16];
    size_t parsed_count = detection_parse_vectors(raw_vectors, num_vectors, parsed, 16);

    pixy_vector_t horiz[16];
    size_t horiz_count = 0;

    /* Filter for horizontal vectors (|dx| > |dy| and |dx| >= DETECTION_MIN_DX_HORIZONTAL) */
    for (size_t i = 0; i < parsed_count; i++) {
        int dx = (int)parsed[i].x1 - (int)parsed[i].x0;
        int dy = (int)parsed[i].y1 - (int)parsed[i].y0;
        int abs_dx = dx < 0 ? -dx : dx;
        int abs_dy = dy < 0 ? -dy : dy;

        if (abs_dx > abs_dy && abs_dx >= (int)DETECTION_MIN_DX_HORIZONTAL) {
            horiz[horiz_count++] = parsed[i];
        }
    }

    if (horiz_count < 2) return false;

    /* Check for any pair of horizontal vectors on the same horizontal line */
    for (size_t i = 0; i < horiz_count; i++) {
        double y_mid1 = ((double)horiz[i].y0 + (double)horiz[i].y1) / 2.0;
        double x_mid1 = ((double)horiz[i].x0 + (double)horiz[i].x1) / 2.0;

        for (size_t j = i + 1; j < horiz_count; j++) {
            double y_mid2 = ((double)horiz[j].y0 + (double)horiz[j].y1) / 2.0;
            double x_mid2 = ((double)horiz[j].x0 + (double)horiz[j].x1) / 2.0;

            double dy_diff = fabs(y_mid1 - y_mid2);
            double dx_diff = fabs(x_mid1 - x_mid2);

            /* Both vectors must be on similar Y level and separated in X */
            if (dy_diff <= (double)y_tolerance && dx_diff >= 10.0) {
                return true;
            }
        }
    }

    return false;
}

void detection_debug_vectors(const uint16_t *raw_vectors, size_t num_vectors)
{
    PRINTF("\r\n=== [Detection Debug] Raw vectors: %u ===\r\n", (unsigned)num_vectors);

    if (raw_vectors == NULL || num_vectors == 0) {
        PRINTF("  No raw vectors to process.\r\n");
        return;
    }

    /* Parse all raw vectors */
    pixy_vector_t parsed[16];
    size_t parsed_count = detection_parse_vectors(raw_vectors, num_vectors, parsed, 16);

    /* Counters for the summary */
    size_t cnt_vertical_left  = 0;
    size_t cnt_vertical_right = 0;
    size_t cnt_horizontal     = 0;
    size_t cnt_rejected       = 0;

    for (size_t i = 0; i < parsed_count; i++) {
        const pixy_vector_t *v = &parsed[i];

        int dx     = (int)v->x1 - (int)v->x0;
        int dy     = (int)v->y1 - (int)v->y0;
        int abs_dx = dx < 0 ? -dx : dx;
        int abs_dy = dy < 0 ? -dy : dy;

        /* Replicate vertical filter: |dy| >= DETECTION_MIN_DY_VERTICAL (8px) */
        bool passes_vertical = (abs_dy >= (int)DETECTION_MIN_DY_VERTICAL);

        /* Replicate horizontal filter: |dx| > |dy| AND |dx| >= DETECTION_MIN_DX_HORIZONTAL (8px) */
        bool passes_horizontal = (abs_dx > abs_dy && abs_dx >= DETECTION_MIN_DX_HORIZONTAL);

        /* Determine bottom_x for left/right classification (highest Y endpoint) */
        double bot_x = (v->y0 > v->y1) ? (double)v->x0 : (double)v->x1;

        PRINTF("  [%u] (%u,%u)->(%u,%u) | dx=%d dy=%d | ",
               (unsigned)i,
               (unsigned)v->x0, (unsigned)v->y0,
               (unsigned)v->x1, (unsigned)v->y1,
               dx, dy);

        if (passes_vertical) {
            /* Classify as left or right based on bottom_x vs frame center */
            if (bot_x < PIXY_FRAME_CENTER_X) {
                PRINTF("VERTICAL -> LEFT  (bot_x=%d)\r\n", (int)bot_x);
                cnt_vertical_left++;
            } else {
                PRINTF("VERTICAL -> RIGHT (bot_x=%d)\r\n", (int)bot_x);
                cnt_vertical_right++;
            }
        } else if (passes_horizontal) {
            double center_x = ((double)v->x0 + (double)v->x1) / 2.0;
            const char *dir = (center_x < PIXY_FRAME_CENTER_X) ? "LEFT" : "RIGHT";
            PRINTF("HORIZONTAL -> TURN %s (center_x=%d)\r\n", dir, (int)center_x);
            cnt_horizontal++;
        } else {
            PRINTF("REJECTED   (|dx|=%d |dy|=%d - too short or diagonal)\r\n",
                   abs_dx, abs_dy);
            cnt_rejected++;
        }
    }

    PRINTF("--- Summary: LEFT=%u RIGHT=%u HORIZ=%u REJECTED=%u ---\r\n\r\n",
           (unsigned)cnt_vertical_left,
           (unsigned)cnt_vertical_right,
           (unsigned)cnt_horizontal,
           (unsigned)cnt_rejected);
}
