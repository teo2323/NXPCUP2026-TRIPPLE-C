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
volatile double g_steering_kp_straight   = DEFAULT_STEERING_KP_STRAIGHT;
volatile double g_steering_kd_straight   = DEFAULT_STEERING_KD_STRAIGHT;
volatile double g_steering_kp_curve      = DEFAULT_STEERING_KP_CURVE;
volatile double g_steering_kd_curve      = DEFAULT_STEERING_KD_CURVE;
volatile double g_curve_error_threshold  = DEFAULT_CURVE_ERROR_THRESHOLD;

// Motor speed and engine state variables
volatile double g_motor_speed            = DEFAULT_MOTOR_SPEED;
volatile bool   g_engine_enabled         = false;
volatile double g_decay_factor           = DEFAULT_DECAY_FACTOR;
volatile double g_sharp_turn_speed_coeff = DEFAULT_SHARP_TURN_SPEED_COEFF;

/* Dual-duplex communication test diagnostics */
volatile uint32_t g_wifi_tx_count = 0;
volatile uint32_t g_wifi_rx_count = 0;
volatile uint32_t g_wifi_last_pong_seq = 0;
char              g_wifi_last_rx_cmd[64] = "NONE";

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
           (LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_TxDataRegEmptyFlag))
    {
        char c = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1U) % TX_BUF_SIZE;
        LPUART_WriteByte(LP_FLEXCOMM7_PERIPHERAL, (uint8_t)c);
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

#define WIFI_RX_RING_SIZE 256

static volatile char s_wifi_rx_ring[WIFI_RX_RING_SIZE];
static volatile uint16_t s_wifi_rx_head = 0;
static volatile uint16_t s_wifi_rx_tail = 0;

void LP_FLEXCOMM7_IRQHandler(void)
{
    uint32_t flags = LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL);

    if (flags & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))
    {
        LPUART_ClearStatusFlags(LP_FLEXCOMM7_PERIPHERAL,
                                kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    }

    while (((LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_RxDataRegFullFlag) != 0U) ||
           (LPUART_GetRxFifoCount(LP_FLEXCOMM7_PERIPHERAL) > 0U))
    {
        char c = (char)LPUART_ReadByte(LP_FLEXCOMM7_PERIPHERAL);
        uint16_t next_head = (s_wifi_rx_head + 1U) % WIFI_RX_RING_SIZE;
        if (next_head != s_wifi_rx_tail)
        {
            s_wifi_rx_ring[s_wifi_rx_head] = c;
            s_wifi_rx_head = next_head;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

void Wifi_Init(void)
{
    tx_head = 0;
    tx_tail = 0;
    rx_idx = 0;
    s_wifi_rx_head = 0;
    s_wifi_rx_tail = 0;

    LPUART_EnableInterrupts(LP_FLEXCOMM7_PERIPHERAL, kLPUART_RxDataRegFullInterruptEnable | kLPUART_RxOverrunInterruptEnable);
    EnableIRQ(LP_FLEXCOMM7_IRQn);
}

void Wifi_SendString(const char *str)
{
    g_wifi_tx_count++;
    while (*str)
    {
        LPUART_SendChar_NonBlocking(*str++);
    }
}

void Wifi_SendPing(uint32_t seq)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "PING=%lu\r\n", (unsigned long)seq);
    Wifi_SendString(buf);
}

void Wifi_SendTelemetry(uint8_t line_count,
                        const char *which_lines,
                        size_t num_vectors,
                        uint32_t horiz_count,
                        double steering_angle,
                        int lx0, int ly0, int lx1, int ly1,
                        int rx0, int ry0, int rx1, int ry1)
{
    char buf[256];
    int steer_i = (int)steering_angle;
    int steer_f = (int)(fabs(steering_angle - (double)steer_i) * 100.0);
    if (steer_f < 0) steer_f = -steer_f;

    snprintf(buf, sizeof(buf),
             "TELEM:lines=%u|which=%s|num_vec=%u|horiz_cnt=%u|steer=%d.%02d|lx0=%d|ly0=%d|lx1=%d|ly1=%d|rx0=%d|ry0=%d|rx1=%d|ry1=%d\r\n",
             (unsigned int)line_count,
             which_lines ? which_lines : "NONE",
             (unsigned int)num_vectors,
             (unsigned int)horiz_count,
             steer_i, steer_f,
             lx0, ly0, lx1, ly1,
             rx0, ry0, rx1, ry1);

    Wifi_SendString(buf);
}

void Wifi_ParseCommand(const char *cmd)
{
    char buf[128];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // Store last received message and increment RX counter
    strncpy(g_wifi_last_rx_cmd, cmd, sizeof(g_wifi_last_rx_cmd) - 1);
    g_wifi_last_rx_cmd[sizeof(g_wifi_last_rx_cmd) - 1] = '\0';
    g_wifi_rx_count++;

    char *eq = strchr(buf, '=');
    if (!eq) {
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: \"%s\"\r\n", cmd);
        return;
    }

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

    if (strcmp(key, "PING") == 0) {
        char resp[32];
        snprintf(resp, sizeof(resp), "PONG=%d\r\n", val_i);
        Wifi_SendString(resp);
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: PING=%d -> Replying PONG=%d\r\n", val_i, val_i);
        return;
    }
    else if (strcmp(key, "PONG") == 0) {
        g_wifi_last_pong_seq = (uint32_t)val_i;
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: PONG=%d (Round-trip confirmed!)\r\n", val_i);
        return;
    }
    else if (strcmp(key, "HEARTBEAT") == 0) {
        Wifi_SendString("ACK: HEARTBEAT = 1\r\n");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: HEARTBEAT=%d -> Sent ACK\r\n", val_i);
        return;
    }
    else if (strcmp(key, "KP_STRAIGHT") == 0 || strcmp(key, "STEERING_KP_STRAIGHT") == 0) {
        g_steering_kp_straight = (double)val;
        Wifi_SendString("ACK: KP_STRAIGHT = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: KP_STRAIGHT=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "KD_STRAIGHT") == 0 || strcmp(key, "STEERING_KD_STRAIGHT") == 0) {
        g_steering_kd_straight = (double)val;
        Wifi_SendString("ACK: KD_STRAIGHT = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: KD_STRAIGHT=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "KP_CURVE") == 0 || strcmp(key, "STEERING_KP_CURVE") == 0) {
        g_steering_kp_curve = (double)val;
        Wifi_SendString("ACK: KP_CURVE = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: KP_CURVE=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "KD_CURVE") == 0 || strcmp(key, "STEERING_KD_CURVE") == 0) {
        g_steering_kd_curve = (double)val;
        Wifi_SendString("ACK: KD_CURVE = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: KD_CURVE=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "CURVE_THRES") == 0 || strcmp(key, "CURVE_ERROR_THRESHOLD") == 0) {
        g_curve_error_threshold = (double)val;
        Wifi_SendString("ACK: CURVE_THRES = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: CURVE_THRES=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "STEERING_P") == 0) {
        g_steering_kp_straight = (double)val;
        g_steering_kp_curve = (double)val * 2.0;
        Wifi_SendString("ACK: STEERING_P = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: STEERING_P=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "STEERING_D") == 0) {
        g_steering_kd_straight = (double)val;
        g_steering_kd_curve = (double)val * 2.4;
        Wifi_SendString("ACK: STEERING_D = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: STEERING_D=%.3f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "MOTOR_SPEED") == 0 || strcmp(key, "SPEED") == 0) {
        g_motor_speed = (double)val;
        Wifi_SendString("ACK: MOTOR_SPEED = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: MOTOR_SPEED=%.1f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "SHARP_COEFF") == 0 || strcmp(key, "SHARP_TURN_SPEED_COEFF") == 0) {
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        g_sharp_turn_speed_coeff = (double)val;
        Wifi_SendString("ACK: SHARP_COEFF = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: SHARP_COEFF=%.2f (ACK sent)\r\n", (double)val);
    }
    else if (strcmp(key, "ENGINE_ENABLED") == 0) {
        g_engine_enabled = (val > 0.5f);
        Wifi_SendString("ACK: ENGINE_ENABLED = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: ENGINE_ENABLED=%d (ACK sent)\r\n", g_engine_enabled ? 1 : 0);
    }
    else if (strcmp(key, "EMERGENCY_STOP") == 0) {
        g_engine_enabled = false;
        HbridgeSpeed(&g_hbridge, 0, 0);
        Steer(0.0);
        Wifi_SendString("ACK: EMERGENCY_STOP = 1\r\n");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: EMERGENCY_STOP (Engines halted, ACK sent)\r\n");
        return;
    }
    else if (strcmp(key, "DECAY_FACTOR") == 0) {
        g_decay_factor = (double)val;
        Wifi_SendString("ACK: DECAY_FACTOR = ");
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: DECAY_FACTOR=%.2f (ACK sent)\r\n", (double)val);
    }
    else {
        PRINTF("[COMM DUPLEX TEST] RX from ESP32: unknown command \"%s\"\r\n", cmd);
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

    // Process all bytes received in s_wifi_rx_ring by ISR
    while (s_wifi_rx_head != s_wifi_rx_tail)
    {
        char c = s_wifi_rx_ring[s_wifi_rx_tail];
        s_wifi_rx_tail = (s_wifi_rx_tail + 1U) % WIFI_RX_RING_SIZE;

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
                rx_idx = 0U;
            }
        }
    }
}