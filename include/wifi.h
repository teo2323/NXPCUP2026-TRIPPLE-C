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
 * @brief Parsează comanda primită sub forma "NUME_PARAMETRU = VAL" și actualizează parametrul corespunzător.
 */
void Wifi_ParseCommand(const char *cmd);

/**
 * @brief Trimite un pachet de telemetrie către ESP32 peste UART.
 */
void Wifi_SendTelemetry(uint8_t line_count,
                        const char *which_lines,
                        size_t num_vectors,
                        double steering_angle,
                        double motor_speed,
                        bool engine_enabled,
                        double battery_volts);

#endif /* WIFI_H_ */