#include "wifi.h"
#include "Config.h"
#include "hbridge.h"
#include "servo.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RX_BUF_SIZE 128

// Definition of global PID parameters initialized with default values
volatile double g_steering_p_right = DEFAULT_STEERING_P_RIGHT;
volatile double g_steering_p_left  = DEFAULT_STEERING_P_LEFT;
volatile double g_steering_d_right = DEFAULT_STEERING_D_RIGHT;
volatile double g_steering_d_left  = DEFAULT_STEERING_D_LEFT;

// Motor speed and engine state variables (blocked on boot for safety)
volatile double g_motor_speed   = 70.0;
volatile bool   g_engine_enabled = false;

static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;

static void LPUART_SendChar(char c)
{
    // Așteaptă până când registrul de transmisie este liber
    while (!(LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL) & kLPUART_TxDataRegEmptyFlag))
    {
    }
    LPUART_WriteByte(LP_FLEXCOMM3_PERIPHERAL, (uint8_t)c);
}

void Wifi_Init(void)
{
    PRINTF("[Wi-Fi] Initializat receptie comenzi tuning PID & Control Viteza\r\n");
}

void Wifi_SendString(const char *str)
{
    while (*str)
    {
        LPUART_SendChar(*str++);
    }
}

void Wifi_ParseCommand(const char *cmd)
{
    char buf[128];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *eq = strchr(buf, '=');
    if (!eq) return;

    *eq = '\0'; // Separa cheia de valoare
    char *key = buf;
    char *val_str = eq + 1;

    // Eliminare spatii de la cheie
    while (*key == ' ' || *key == '\t' || *key == '\r' || *key == '\n') key++;
    char *k_end = key + strlen(key) - 1;
    while (k_end > key && (*k_end == ' ' || *k_end == '\t' || *k_end == '\r' || *k_end == '\n')) {
        *k_end = '\0';
        k_end--;
    }

    // Eliminare spatii de la valoare
    while (*val_str == ' ' || *val_str == '\t' || *val_str == '\r' || *val_str == '\n') val_str++;
    float val = (float)atof(val_str);
    int val_i = (int)val;
    int val_f = (int)((val - (float)val_i) * 1000.0f);
    if (val_f < 0) val_f = -val_f;


    if (strcmp(key, "STEERING_P_RIGHT") == 0) {
        g_steering_p_right = (double)val;
        PRINTF("[PID Update] STEERING_P_RIGHT setat la %d.%03d\r\n", val_i, val_f);
        Wifi_SendString("ACK: STEERING_P_RIGHT = ");
    }
    else if (strcmp(key, "STEERING_P_LEFT") == 0) {
        g_steering_p_left = (double)val;
        PRINTF("[PID Update] STEERING_P_LEFT setat la %d.%03d\r\n", val_i, val_f);
        Wifi_SendString("ACK: STEERING_P_LEFT = ");
    }
    else if (strcmp(key, "STEERING_D_RIGHT") == 0) {
        g_steering_d_right = (double)val;
        PRINTF("[PID Update] STEERING_D_RIGHT setat la %d.%03d\r\n", val_i, val_f);
        Wifi_SendString("ACK: STEERING_D_RIGHT = ");
    }
    else if (strcmp(key, "STEERING_D_LEFT") == 0) {
        g_steering_d_left = (double)val;
        PRINTF("[PID Update] STEERING_D_LEFT setat la %d.%03d\r\n", val_i, val_f);
        Wifi_SendString("ACK: STEERING_D_LEFT = ");
    }
    else if (strcmp(key, "MOTOR_SPEED") == 0) {
        g_motor_speed = (double)val;
        PRINTF("[MOTOARE] Viteza actualizata la %d%%\r\n", (int)g_motor_speed);
        Wifi_SendString("ACK: MOTOR_SPEED = ");
    }
    else if (strcmp(key, "ENGINE_ENABLED") == 0) {
        g_engine_enabled = (val > 0.5f);
        if (g_engine_enabled) {
            PRINTF("[MOTOARE] Comanda Web: MOTOARE PORNITE (Viteza: %d%%)\r\n", (int)g_motor_speed);
        } else {
            PRINTF("[MOTOARE] Comanda Web: MOTOARE OPRITE\r\n");
        }
        Wifi_SendString("ACK: ENGINE_ENABLED = ");
    }
    else if (strcmp(key, "EMERGENCY_STOP") == 0) {
        g_engine_enabled = false;
        HbridgeSpeed(&g_hbridge, 0, 0);
        Steer(0.0);
        PRINTF("[MOTOARE] EMERGENCY STOP ACTIVAT! Motoare oprite instant!\r\n");
        Wifi_SendString("ACK: EMERGENCY_STOP = 1\r\n");
        return;
    }
    else {
        PRINTF("[PID Update] Parametru necunoscut: '%s'\r\n", key);
        return;
    }

    char ack_buf[32];
    snprintf(ack_buf, sizeof(ack_buf), "%d.%03d\r\n", val_i, val_f);
    Wifi_SendString(ack_buf);

}


void Wifi_Process_Rx(void)
{
    uint32_t flags = LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL);

    // 1. Curățare erori de linie LPUART
    if (flags & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))
    {
        LPUART_ClearStatusFlags(LP_FLEXCOMM3_PERIPHERAL,
                                kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    }

    // 2. Citire caractere din buffer-ul hardware
    while (LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL) & kLPUART_RxDataRegFullFlag)
    {
        char c = (char)LPUART_ReadByte(LP_FLEXCOMM3_PERIPHERAL);

        if (c == ';' || c == '\n' || c == '\r')
        {
            if (rx_idx > 0U)
            {
                rx_buf[rx_idx] = '\0';
                PRINTF("Mesaj receptionat: %s\r\n", (const char *)rx_buf);
                Wifi_ParseCommand((const char *)rx_buf);
                rx_idx = 0U;
            }
        }
        else if ((uint8_t)c >= 32U && rx_idx < (uint8_t)(sizeof(rx_buf) - 1U))
        {
            rx_buf[rx_idx++] = c;
        }
    }
}