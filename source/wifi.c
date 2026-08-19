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
#include <math.h>

#define RX_BUF_SIZE 128

// Definition of global PID parameters initialized with default values
volatile double g_steering_p_right = DEFAULT_STEERING_P_RIGHT;
volatile double g_steering_p_left  = DEFAULT_STEERING_P_LEFT;
volatile double g_steering_d_right = DEFAULT_STEERING_D_RIGHT;
volatile double g_steering_d_left  = DEFAULT_STEERING_D_LEFT;

// Motor speed and engine state variables (blocked on boot for safety)
volatile double g_motor_speed   = 70.0;
volatile bool   g_engine_enabled = false;

#define TX_BUF_SIZE 512

static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;

static volatile char tx_buf[TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;

/**
 * @brief Servicii non-blocante pentru golirea buffer-ului circular TX în registrul hardware LPUART.
 */
void Wifi_Flush_Tx(void)
{
    while ((tx_head != tx_tail) &&
           (LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL) & kLPUART_TxDataRegEmptyFlag))
    {
        char c = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1U) % TX_BUF_SIZE;
        LPUART_WriteByte(LP_FLEXCOMM3_PERIPHERAL, (uint8_t)c);
    }
}

static void LPUART_SendChar_NonBlocking(char c)
{
    uint16_t next_head = (tx_head + 1U) % TX_BUF_SIZE;
    if (next_head != tx_tail)
    {
        tx_buf[tx_head] = c;
        tx_head = next_head;
    }
    Wifi_Flush_Tx();
}

void Wifi_Init(void)
{
    tx_head = 0;
    tx_tail = 0;
    rx_idx = 0;
}

void Wifi_SendString(const char *str)
{
    while (*str)
    {
        LPUART_SendChar_NonBlocking(*str++);
    }
}

void Wifi_SendTelemetry(uint8_t line_count,
                        const char *which_lines,
                        size_t num_vectors,
                        double steering_angle,
                        double motor_speed,
                        bool engine_enabled,
                        double battery_volts)
{
    char buf[128];
    int steer_i = (int)steering_angle;
    int steer_f = (int)(fabs(steering_angle - (double)steer_i) * 100.0);
    if (steer_f < 0) steer_f = -steer_f;

    int batt_i = (int)battery_volts;
    int batt_f = (int)(fabs(battery_volts - (double)batt_i) * 100.0);
    if (batt_f < 0) batt_f = -batt_f;

    snprintf(buf, sizeof(buf),
             "TELEM:lines=%u|which=%s|num_vec=%u|steer=%d.%02d|speed=%d|eng=%d|batt=%d.%02d\r\n",
             (unsigned int)line_count,
             which_lines ? which_lines : "NONE",
             (unsigned int)num_vectors,
             steer_i, steer_f,
             (int)motor_speed,
             engine_enabled ? 1 : 0,
             batt_i, batt_f);

    Wifi_SendString(buf);
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
        Wifi_SendString("ACK: STEERING_P_RIGHT = ");
    }
    else if (strcmp(key, "STEERING_P_LEFT") == 0) {
        g_steering_p_left = (double)val;
        Wifi_SendString("ACK: STEERING_P_LEFT = ");
    }
    else if (strcmp(key, "STEERING_D_RIGHT") == 0) {
        g_steering_d_right = (double)val;
        Wifi_SendString("ACK: STEERING_D_RIGHT = ");
    }
    else if (strcmp(key, "STEERING_D_LEFT") == 0) {
        g_steering_d_left = (double)val;
        Wifi_SendString("ACK: STEERING_D_LEFT = ");
    }
    else if (strcmp(key, "MOTOR_SPEED") == 0) {
        g_motor_speed = (double)val;
        Wifi_SendString("ACK: MOTOR_SPEED = ");
    }
    else if (strcmp(key, "ENGINE_ENABLED") == 0) {
        g_engine_enabled = (val > 0.5f);
        Wifi_SendString("ACK: ENGINE_ENABLED = ");
    }
    else if (strcmp(key, "EMERGENCY_STOP") == 0) {
        g_engine_enabled = false;
        HbridgeSpeed(&g_hbridge, 0, 0);
        Steer(0.0);
        Wifi_SendString("ACK: EMERGENCY_STOP = 1\r\n");
        return;
    }
    else if (strcmp(key, "HEARTBEAT") == 0) {
        Wifi_SendString("ACK: HEARTBEAT = 1\r\n");
        return;
    }
    else {
        return;
    }

    char ack_buf[32];
    snprintf(ack_buf, sizeof(ack_buf), "%d.%03d\r\n", val_i, val_f);
    Wifi_SendString(ack_buf);

}


void Wifi_Process_Rx(void)
{
    // Always flush non-blocking UART TX ring buffer
    Wifi_Flush_Tx();

    uint32_t flags = LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL);

    // 1. Curățare erori de linie LPUART
    if (flags & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))
    {
        LPUART_ClearStatusFlags(LP_FLEXCOMM3_PERIPHERAL,
                                kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
        // Reset buffer index on line error/overrun to prevent corrupted string accumulation
        rx_idx = 0U;
    }

    // 2. Citire caractere din buffer-ul hardware
    while (((LPUART_GetStatusFlags(LP_FLEXCOMM3_PERIPHERAL) & kLPUART_RxDataRegFullFlag) != 0U) ||
           (LPUART_GetRxFifoCount(LP_FLEXCOMM3_PERIPHERAL) > 0U))
    {
        char c = (char)LPUART_ReadByte(LP_FLEXCOMM3_PERIPHERAL);

        if (c == ';' || c == '\n' || c == '\r')
        {
            if (rx_idx > 0U)
            {
                rx_buf[rx_idx] = '\0';
                Wifi_ParseCommand((const char *)rx_buf);
                rx_idx = 0U;
            }
        }
        else if ((uint8_t)c >= 32U)
        {
            if (rx_idx < (uint8_t)(sizeof(rx_buf) - 1U))
            {
                rx_buf[rx_idx++] = c;
            }
            else
            {
                // Buffer overflow protection: reset on overflow to avoid string mangling
                rx_idx = 0U;
            }
        }
    }
}