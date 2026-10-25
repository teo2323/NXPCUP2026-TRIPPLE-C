#ifndef WIFI_H_
#define WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ========================================================================= */
/* Dynamic Tuning Variables (Tunable via ESP32 Web Interface)                */
/* ========================================================================= */

// Steering 2-Zone PID coefficients
extern volatile double g_steering_kp_straight;
extern volatile double g_steering_kd_straight;
extern volatile double g_steering_kp_curve;
extern volatile double g_steering_kd_curve;

// Dual Steering Error Deadbands
extern volatile double g_steering_deadband_straight;
extern volatile double g_steering_deadband_curve;

// Curve hysteresis thresholds
extern volatile double g_curve_enter_angle_deg;
extern volatile double g_curve_exit_angle_deg;
extern volatile double g_curve_exit_frames;

// Camera perspective offset
extern volatile double g_perspective_angle_offset;

// Physical steering limits & calibration offset
extern volatile double g_steering_limit_right;
extern volatile double g_steering_limit_left;
extern volatile double g_steering_offset;

// Motor speeds & dynamics
extern volatile double g_speed_right;
extern volatile double g_speed_left;
extern volatile double g_corner_brake_coeff;
extern volatile double g_diff_speed_coeff;
extern volatile double g_consecutive_curve_time_ms;
extern volatile double g_consecutive_curve_speed;
extern volatile double g_min_curve_duration_ms;
extern volatile double g_max_single_curve_time_ms;

// Detection thresholds & decay
extern volatile double g_vertical_min_slope;
extern volatile double g_decay_factor;
extern volatile double g_no_line_max_lost_frames;

// Engine safety & drive state
extern volatile bool   g_engine_enabled;
extern volatile bool   g_engine_state_changed;

/* ========================================================================= */
/* Diagnostic Metrics                                                        */
/* ========================================================================= */
extern volatile uint32_t g_wifi_tx_count;
extern volatile uint32_t g_wifi_rx_count;
extern volatile uint32_t g_wifi_rx_packet_count;
extern volatile uint32_t g_wifi_last_pong_seq;
extern char              g_wifi_last_rx_cmd[64];

/* ========================================================================= */
/* Parameter Registry Descriptor                                             */
/* ========================================================================= */
typedef struct {
    const char      *name;        /**< Exact parameter name received via UART, e.g. "STEERING_KP_STRAIGHT" */
    volatile double *val_ptr;     /**< Pointer to the global variable in RAM */
    double           min_val;     /**< Minimum allowable value (for safety clamping) */
    double           max_val;     /**< Maximum allowable value (for safety clamping) */
    double           default_val; /**< Default initial value */
} config_param_entry_t;

/* ========================================================================= */
/* Function Prototypes                                                       */
/* ========================================================================= */

/**
 * @brief Initialize the Wi-Fi UART module (LP_FLEXCOMM7 on P3_2/P3_3).
 */
void Wifi_Init(void);

/**
 * @brief Process incoming characters from the RX ring buffer.
 *        Poll this regularly inside the main control loop.
 */
void Wifi_Process_Rx(void);

/**
 * @brief Parse and execute a command received via UART ("PARAM=VALUE").
 * @param cmd Null-terminated string containing the command.
 */
void Wifi_ParseCommand(const char *cmd);

/**
 * @brief Send a null-terminated string to the ESP32 module over UART.
 */
void Wifi_SendString(const char *str);

/**
 * @brief Synchronous or non-blocking flush helper for TX buffer.
 */
void Wifi_Flush_Tx(void);

/**
 * @brief Send a ping message with sequence number to verify bidirectional communication.
 */
void Wifi_SendPing(uint32_t seq);

/**
 * @brief Send a compact telemetry frame to the ESP32 over UART.
 */
void Wifi_SendTelemetry(uint8_t line_count,
                        const char *which_lines,
                        size_t num_vectors,
                        uint32_t horiz_count,
                        double steering_angle,
                        int lx0, int ly0, int lx1, int ly1,
                        int rx0, int ry0, int rx1, int ry1);

#endif /* WIFI_H_ */
