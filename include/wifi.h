#ifndef WIFI_H_
#define WIFI_H_

#include <stdbool.h>
#include <stddef.h>

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
 * @brief Parsează comanda primită sub forma "NUME_PARAMETRU = VAL" și actualizează parametrul corespunzător.
 */
void Wifi_ParseCommand(const char *cmd);

#endif /* WIFI_H_ */