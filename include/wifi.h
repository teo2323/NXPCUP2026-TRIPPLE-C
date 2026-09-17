#ifndef WIFI_H_
#define WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Dynamic PID & Motor variables (tunable via ESP32 Web Server) */
extern volatile double g_steering_kp_straight;
extern volatile double g_steering_kd_straight;
extern volatile double g_steering_kp_curve;
extern volatile double g_steering_kd_curve;
extern volatile double g_curve_error_threshold;
extern volatile double g_motor_speed;
extern volatile bool   g_engine_enabled;
extern volatile double g_decay_factor;
extern volatile double g_sharp_turn_speed_coeff;

/**
 * @brief Inițializează modulul Wi-Fi (LPUART7 pe P3_2 / P3_3).
 */
void Wifi_Init(void);

/**
 * @brief Procesează datele primite de la modulul Wi-Fi prin LPUART.
 *        Ar trebui apelată periodic în bucla principală.
 */
void Wifi_Process_Rx(void);

/**
 * @brief Trimite un șir de caractere către modulul Wi-Fi.
 */
void Wifi_SendString(const char *str);

/**
 * @brief Evacuează datele din bufferul circular TX către UART fără blocare.
 */
void Wifi_Flush_Tx(void);

/**
 * @brief Trimite un pachet de telemetrie restrâns către ESP32 peste UART.
 */
void Wifi_SendTelemetry(uint8_t line_count,
                        const char *which_lines,
                        size_t num_vectors,
                        uint32_t horiz_count,
                        double steering_angle,
                        int lx0, int ly0, int lx1, int ly1,
                        int rx0, int ry0, int rx1, int ry1);

/* Dual-duplex communication test diagnostics */
extern volatile uint32_t g_wifi_tx_count;
extern volatile uint32_t g_wifi_rx_count;
extern volatile uint32_t g_wifi_last_pong_seq;
extern char              g_wifi_last_rx_cmd[64];

/**
 * @brief Trimite un mesaj de test PING către ESP32 peste UART.
 */
void Wifi_SendPing(uint32_t seq);



#endif /* WIFI_H_ */