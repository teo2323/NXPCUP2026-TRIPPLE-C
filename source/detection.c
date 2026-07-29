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

void detection_vector_components(const pixy_vector_t *vec, int *dx, int *dy)
{
    if (vec == NULL) return;
    if (dx) *dx = (int)vec->x1 - (int)vec->x0;
    if (dy) *dy = (int)vec->y1 - (int)vec->y0;
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

void detection_print_vector_details(const pixy_vector_t *vec, size_t index)
{
    if (vec == NULL) return;

    int dx = (int)vec->x1 - (int)vec->x0;
    int dy = (int)vec->y1 - (int)vec->y0;

    double length    = detection_vector_length(vec);
    double angle_deg = detection_vector_angle_deg(vec);

    int len_int  = (int)length;
    int len_frac = (int)((length - (double)len_int) * 100.0);
    if (len_frac < 0) len_frac = -len_frac;

    int ang_int  = (int)angle_deg;
    int ang_frac = (int)((angle_deg - (double)ang_int) * 100.0);
    if (ang_frac < 0) ang_frac = -ang_frac;

    PRINTF("Vector [%u] details:\r\n", (unsigned)index);
    PRINTF("  Start: (%u, %u) -> End: (%u, %u)\r\n",
           (unsigned)vec->x0, (unsigned)vec->y0,
           (unsigned)vec->x1, (unsigned)vec->y1);
    PRINTF("  dx: %d, dy: %d\r\n", dx, dy);
    PRINTF("  Length: %d.%02d px\r\n", len_int, len_frac);
    PRINTF("  Angle: %d.%02d deg\r\n", ang_int, ang_frac);
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

size_t detection_filter_length(const pixy_vector_t *in,
                               size_t in_count,
                               pixy_vector_t *out,
                               double min_length)
{
    if (in == NULL || out == NULL) return 0;

    size_t count = 0;
    for (size_t i = 0; i < in_count; i++) {
        if (detection_vector_length(&in[i]) >= min_length) {
            out[count++] = in[i];
        }
    }
    return count;
}

size_t detection_filter_roi(const pixy_vector_t *in,
                            size_t in_count,
                            pixy_vector_t *out,
                            uint16_t min_y,
                            uint16_t max_y)
{
    if (in == NULL || out == NULL) return 0;

    size_t count = 0;
    for (size_t i = 0; i < in_count; i++) {
        if (in[i].y0 >= min_y && in[i].y0 <= max_y &&
            in[i].y1 >= min_y && in[i].y1 <= max_y) {
            out[count++] = in[i];
        }
    }
    return count;
}

/* Helper to get bottom X coordinate of a vector (point closest to car) */
static double get_vector_bottom_x(const pixy_vector_t *vec)
{
    return (vec->y0 > vec->y1) ? (double)vec->x0 : (double)vec->x1;
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

bool detection_extract_line_track(const pixy_vector_t *vectors,
                                  size_t count,
                                  line_side_t side,
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

        /* Score vector: base weight on bottom X coordinate (without considering vector length) */
        double score = bot_x;

        /* Apply side preference weighting if side is explicitly specified */
        if (side == LINE_SIDE_LEFT && bot_x < PIXY_FRAME_CENTER_X) {
            score *= 1.2;
        } else if (side == LINE_SIDE_RIGHT && bot_x >= PIXY_FRAME_CENTER_X) {
            score *= 1.2;
        }

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

bool detection_find_primary_vector(const pixy_vector_t *vectors,
                                   size_t count,
                                   pixy_vector_t *primary)
{
    if (vectors == NULL || count == 0 || primary == NULL) return false;

    double max_score = -1.0;
    size_t best_idx  = 0;

    for (size_t i = 0; i < count; i++) {
        double len = detection_vector_length(&vectors[i]);
        uint16_t max_y = (vectors[i].y0 > vectors[i].y1) ? vectors[i].y0 : vectors[i].y1;
        double score   = len * (1.0 + ((double)max_y / (double)PIXY_FRAME_HEIGHT));

        if (score > max_score) {
            max_score = score;
            best_idx  = i;
        }
    }

    *primary = vectors[best_idx];
    return true;
}

size_t detection_calculate_avg_slope_angle(const pixy_vector_t *vectors,
                                            size_t count,
                                            double min_dy,
                                            double *out_angle)
{
    if (vectors == NULL || out_angle == NULL) return 0;

    double angle_sum   = 0.0;
    size_t valid_count = 0;

    for (size_t i = 0; i < count; i++) {
        double diff_y = (double)vectors[i].y0 - (double)vectors[i].y1;
        if (fabs(diff_y) < min_dy) {
            continue;
        }
        double m = ((double)vectors[i].x0 - (double)vectors[i].x1) / diff_y;
        angle_sum += m;
        valid_count++;
    }

    if (valid_count > 0) {
        angle_sum /= (double)valid_count;
    }
    angle_sum *= -1.0;

    *out_angle = angle_sum;
    return valid_count;
}

size_t detection_calculate_weighted_steering(const pixy_vector_t *vectors,
                                              size_t count,
                                              double *out_angle)
{
    if (vectors == NULL || out_angle == NULL) return 0;

    double weighted_angle_sum = 0.0;
    double total_weight        = 0.0;
    size_t valid_count         = 0;

    for (size_t i = 0; i < count; i++) {
        double diff_y = (double)vectors[i].y0 - (double)vectors[i].y1;
        if (fabs(diff_y) < DETECTION_MIN_DY_DEFAULT) {
            continue;
        }

        double inv_slope = ((double)vectors[i].x0 - (double)vectors[i].x1) / diff_y;
        double length    = detection_vector_length(&vectors[i]);

        uint16_t max_y = (vectors[i].y0 > vectors[i].y1) ? vectors[i].y0 : vectors[i].y1;
        double weight  = length * (1.0 + ((double)max_y / (double)PIXY_FRAME_HEIGHT));

        weighted_angle_sum += inv_slope * weight;
        total_weight       += weight;
        valid_count++;
    }

    if (total_weight > 1e-6) {
        *out_angle = -1.0 * (weighted_angle_sum / total_weight);
    } else {
        *out_angle = 0.0;
    }

    return valid_count;
}

double detection_calculate_line_offset(const pixy_vector_t *vectors,
                                       size_t count)
{
    if (vectors == NULL || count == 0) return 0.0;

    double x_sum       = 0.0;
    size_t valid_count = 0;

    for (size_t i = 0; i < count; i++) {
        uint16_t bottom_x = (vectors[i].y0 > vectors[i].y1) ? vectors[i].x0 : vectors[i].x1;
        x_sum += (double)bottom_x;
        valid_count++;
    }

    if (valid_count == 0) return 0.0;
    double avg_x = x_sum / (double)valid_count;

    return avg_x - PIXY_FRAME_CENTER_X;
}

bool detection_detect_intersection(const pixy_vector_t *vectors,
                                    size_t count)
{
    if (vectors == NULL || count < 2) return false;

    for (size_t i = 0; i < count; i++) {
        double angle_i = detection_vector_angle_deg(&vectors[i]);
        for (size_t j = i + 1; j < count; j++) {
            double angle_j = detection_vector_angle_deg(&vectors[j]);
            if (fabs(angle_i - angle_j) > 30.0) {
                return true;
            }
        }
    }

    return false;
}

bool detection_detect_sharp_turn(const pixy_vector_t *vectors,
                                 size_t count,
                                 double angle_threshold_deg)
{
    if (vectors == NULL || count == 0) return false;

    for (size_t i = 0; i < count; i++) {
        double angle = fabs(detection_vector_angle_deg(&vectors[i]));
        if (angle >= angle_threshold_deg) {
            return true;
        }
    }

    return false;
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
        /* CASE 1: Both Left and Right lines detected -> Track Centerline */
        result->track_width    = right->bottom_x - left->bottom_x;
        result->track_center_x = (left->bottom_x + right->bottom_x) / 2.0;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->avg_slope      = (left->inverse_slope + right->inverse_slope) / 2.0;

        /* Combined steering angle = slope heading + offset correction */
        result->steering_angle = -1.0 * (result->avg_slope + (result->center_offset * 0.05));
    }
    else if (!has_left && has_right) {
        /* CASE 2: Left line is MISSING (only Right line detected).
         * Steer LEFT (negative angle) in the direction of the missing left line until it reappears. */
        result->track_center_x = right->bottom_x - half_track_width;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->track_width    = 2.0 * half_track_width;
        result->avg_slope      = right->inverse_slope;

        /* Negative slope/steering value forces steering LEFT towards missing line */
        double recover_steer   = (right->inverse_slope < -0.2) ? right->inverse_slope : -0.6;
        result->steering_angle = recover_steer;
    }
    else if (has_left && !has_right) {
        /* CASE 3: Right line is MISSING (only Left line detected).
         * Steer RIGHT (positive angle) in the direction of the missing right line until it reappears. */
        result->track_center_x = left->bottom_x + half_track_width;
        result->center_offset  = result->track_center_x - PIXY_FRAME_CENTER_X;
        result->track_width    = 2.0 * half_track_width;
        result->avg_slope      = left->inverse_slope;

        /* Positive slope/steering value forces steering RIGHT towards missing line */
        double recover_steer   = (left->inverse_slope > 0.2) ? left->inverse_slope : 0.6;
        result->steering_angle = recover_steer;
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
    size_t filtered_count = detection_filter_vertical(parsed, parsed_count, filtered, DETECTION_MIN_DY_DEFAULT);

    result->valid_vectors = filtered_count;
    if (filtered_count == 0) return;

    pixy_vector_t left_vecs[16], right_vecs[16];
    size_t left_cnt = 0, right_cnt = 0;

    detection_classify_left_right(filtered, filtered_count,
                                  left_vecs, &left_cnt,
                                  right_vecs, &right_cnt);

    line_track_t left_line, right_line;
    detection_extract_line_track(left_vecs, left_cnt, LINE_SIDE_LEFT, &left_line);
    detection_extract_line_track(right_vecs, right_cnt, LINE_SIDE_RIGHT, &right_line);

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

void detection_process_vectors(const uint16_t *raw_vectors,
                               size_t num_vectors,
                               detection_result_t *result)
{
    if (result == NULL) return;

    dual_line_detection_result_t dual_res;
    detection_process_dual_lines(raw_vectors, num_vectors, &dual_res);

    result->steering_angle        = dual_res.steering_angle;
    result->line_offset           = dual_res.center_offset;
    result->avg_slope             = dual_res.avg_slope;
    result->valid_vectors         = dual_res.valid_vectors;
    result->intersection_detected = dual_res.intersection_detected;
    result->sharp_turn_detected   = dual_res.sharp_turn_detected;
}
