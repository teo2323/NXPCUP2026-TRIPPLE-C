#ifndef WIFI_H_
#define WIFI_H_

#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Inițializează modulul Wi-Fi (trimite comenzi AT inițiale).
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

#endif /* WIFI_H_ */