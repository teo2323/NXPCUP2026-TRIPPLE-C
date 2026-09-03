#ifndef WIFI_H_
#define WIFI_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Variabile dinamice globale pentru parametri de direcție PID (actualizați din Web Server via ESP32)
 */
extern volatile double g_steering_p_right;
extern volatile double g_steering_p_left;
extern volatile double g_steering_d_right;
extern volatile double g_steering_d_left;

/**
 * @brief Variabile de control viteză și stare motor (actualizate din Web Server)
 */
extern volatile double g_motor_speed;
extern volatile bool g_engine_enabled;
extern volatile double g_decay_factor;


/**
 * @brief Inițializează modulul Wi-Fi.
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



#endif /* WIFI_H_ */