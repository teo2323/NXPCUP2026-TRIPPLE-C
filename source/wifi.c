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

/* ========================================================================= */
/* Global Parameter Definitions (Initialized to Config.h Defaults)           */
/* ========================================================================= */

// Steering 2-Zone PID coefficients
volatile double g_steering_kp_straight        = DEFAULT_STEERING_KP_STRAIGHT;
volatile double g_steering_kd_straight        = DEFAULT_STEERING_KD_STRAIGHT;
volatile double g_steering_kp_curve           = DEFAULT_STEERING_KP_CURVE;
volatile double g_steering_kd_curve           = DEFAULT_STEERING_KD_CURVE;
volatile double g_curve_error_threshold       = DEFAULT_CURVE_ERROR_THRESHOLD;

// Steering limits and geometry offset
volatile double g_steering_limit_right        = DEFAULT_STEERING_LIMIT_RIGHT;
volatile double g_steering_limit_left         = DEFAULT_STEERING_LIMIT_LEFT;
volatile double g_steering_offset             = DEFAULT_STEERING_OFFSET;

// Motor speeds and dynamics
volatile double g_motor_speed                 = DEFAULT_MOTOR_SPEED;
volatile double g_sharp_turn_speed_coeff      = DEFAULT_SHARP_TURN_SPEED_COEFF;
volatile double g_speed_left                  = DEFAULT_SPEED_LEFT;
volatile double g_speed_right                 = DEFAULT_SPEED_RIGHT;

// Track detection and decay thresholds
volatile double g_sharp_turn_angle_threshold  = DEFAULT_SHARP_TURN_ANGLE_THRESHOLD;
volatile double g_decay_factor                = DEFAULT_DECAY_FACTOR;

// Engine safety & drive state (starts disabled for safety)
volatile bool   g_engine_enabled              = false;

/* ========================================================================= */
/* Diagnostics                                                               */
/* ========================================================================= */
volatile uint32_t g_wifi_tx_count        = 0;
volatile uint32_t g_wifi_rx_count        = 0;
volatile uint32_t g_wifi_rx_packet_count = 0;
volatile uint32_t g_wifi_last_pong_seq   = 0;
char              g_wifi_last_rx_cmd[64] = "NONE";

/* ========================================================================= */
/* Modular Parameter Registry Table                                          */
/* Easily add or change parameters for any algorithm on any branch here!     */
/* ========================================================================= */
static const config_param_entry_t g_config_param_table[] = {
    // 2-Zone Steering PID
    {"STEERING_KP_STRAIGHT",        &g_steering_kp_straight,        0.0,    10.0,  DEFAULT_STEERING_KP_STRAIGHT},
    {"STEERING_KD_STRAIGHT",        &g_steering_kd_straight,        0.0,     5.0,  DEFAULT_STEERING_KD_STRAIGHT},
    {"STEERING_KP_CURVE",           &g_steering_kp_curve,           0.0,    15.0,  DEFAULT_STEERING_KP_CURVE},
    {"STEERING_KD_CURVE",           &g_steering_kd_curve,           0.0,    10.0,  DEFAULT_STEERING_KD_CURVE},
    {"CURVE_ERROR_THRESHOLD",       &g_curve_error_threshold,       1.0,    45.0,  DEFAULT_CURVE_ERROR_THRESHOLD},

    // Steering Limits & Geometry
    {"STEERING_LIMIT_RIGHT",        &g_steering_limit_right,        0.0,    90.0,  DEFAULT_STEERING_LIMIT_RIGHT},
    {"STEERING_LIMIT_LEFT",         &g_steering_limit_left,       -90.0,     0.0,  DEFAULT_STEERING_LIMIT_LEFT},
    {"STEERING_OFFSET",             &g_steering_offset,           -30.0,    30.0,  DEFAULT_STEERING_OFFSET},

    // Motor Speeds & Sharp Turn Scaling
    {"MOTOR_SPEED",                 &g_motor_speed,                 0.0,   100.0,  DEFAULT_MOTOR_SPEED},
    {"SHARP_TURN_SPEED_COEFF",      &g_sharp_turn_speed_coeff,      0.0,     1.0,  DEFAULT_SHARP_TURN_SPEED_COEFF},
    {"SPEED_LEFT",                  &g_speed_left,                  0.0,   100.0,  DEFAULT_SPEED_LEFT},
    {"SPEED_RIGHT",                 &g_speed_right,                 0.0,   100.0,  DEFAULT_SPEED_RIGHT},

    // Track Detection & Decay Thresholds
    {"SHARP_TURN_ANGLE_THRESHOLD",  &g_sharp_turn_angle_threshold,  5.0,    60.0,  DEFAULT_SHARP_TURN_ANGLE_THRESHOLD},
    {"DECAY_FACTOR",                &g_decay_factor,                0.1,     1.0,  DEFAULT_DECAY_FACTOR},
};
#define CONFIG_PARAM_COUNT (sizeof(g_config_param_table) / sizeof(g_config_param_table[0]))

/* ========================================================================= */
/* Interrupt-Driven UART RX Ring Buffer                                      */
/* ========================================================================= */
#define RX_RING_BUFFER_SIZE 256
static volatile char s_rxRing[RX_RING_BUFFER_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;

void LP_FLEXCOMM7_IRQHandler(void)
{
    uint32_t status = LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL);

    /* Clear overrun, framing, or noise errors if present */
    if (status & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag | kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))
    {
        LPUART_ClearStatusFlags(LP_FLEXCOMM7_PERIPHERAL,
                                kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    }

    /* Read incoming byte into ring buffer */
    while ((LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_RxDataRegFullFlag) != 0U)
    {
        char c = (char)LPUART_ReadByte(LP_FLEXCOMM7_PERIPHERAL);
        uint16_t nextHead = (s_rxHead + 1U) % RX_RING_BUFFER_SIZE;
        if (nextHead != s_rxTail)
        {
            s_rxRing[s_rxHead] = c;
            s_rxHead = nextHead;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

void Wifi_Init(void)
{
    s_rxHead = 0;
    s_rxTail = 0;
    g_wifi_rx_packet_count = 0;
    g_wifi_rx_count = 0;
    g_wifi_tx_count = 0;

    /* Enable RX Interrupts and configure NVIC priority */
    LPUART_EnableInterrupts(LP_FLEXCOMM7_PERIPHERAL, kLPUART_RxDataRegFullInterruptEnable | kLPUART_RxOverrunInterruptEnable);
    NVIC_SetPriority(LP_FLEXCOMM7_IRQn, 3);
    EnableIRQ(LP_FLEXCOMM7_IRQn);
}

void Wifi_Flush_Tx(void)
{
    /* Handled synchronously by direct hardware register polling */
}

void Wifi_SendString(const char *str)
{
    if (str == NULL) return;
    g_wifi_tx_count++;
    while (*str)
    {
        while ((LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_TxDataRegEmptyFlag) == 0U) {}
        LPUART_WriteByte(LP_FLEXCOMM7_PERIPHERAL, (uint8_t)(*str++));
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

/* ========================================================================= */
/* Command Parser (Table-Driven)                                             */
/* Format expected: "NAME_OF_THE_PARAMETER=NEW_VALUE"                        */
/* ========================================================================= */
void Wifi_ParseCommand(const char *cmd)
{
    if (cmd == NULL) return;

    char buf[128];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *eq = strchr(buf, '=');
    if (!eq) {
        return;
    }

    *eq = '\0';
    char *key = buf;
    char *val_str = eq + 1;

    // Trim whitespace from key
    while (*key == ' ' || *key == '\t' || *key == '\r' || *key == '\n') key++;
    char *k_end = key + strlen(key) - 1;
    while (k_end > key && (*k_end == ' ' || *k_end == '\t' || *k_end == '\r' || *k_end == '\n')) {
        *k_end = '\0';
        k_end--;
    }

    // Trim whitespace from val_str
    while (*val_str == ' ' || *val_str == '\t' || *val_str == '\r' || *val_str == '\n') val_str++;
    char *v_end = val_str + strlen(val_str) - 1;
    while (v_end > val_str && (*v_end == ' ' || *v_end == '\t' || *v_end == '\r' || *v_end == '\n')) {
        *v_end = '\0';
        v_end--;
    }

    float val = (float)atof(val_str);

    // 1. Diagnostic Ping / Pong
    if (strcmp(key, "PING") == 0) {
        char resp[32];
        snprintf(resp, sizeof(resp), "PONG=%d\r\n", (int)val);
        Wifi_SendString(resp);
        PRINTF("[FRDM-UART] Replied PONG=%d to ESP32\r\n", (int)val);
        return;
    }
    else if (strcmp(key, "PONG") == 0) {
        g_wifi_last_pong_seq = (uint32_t)val;
        PRINTF("[FRDM-UART] RX PONG=%d from ESP32 (Round-trip OK!)\r\n", (int)val);
        return;
    }

    // 2. Safety & Engine Controls
    if (strcmp(key, "EMERGENCY_STOP") == 0) {
        g_engine_enabled = false;
        HbridgeSpeed(&g_hbridge, 0, 0);
        Steer(0.0 + STEERING_OFFSET);
        Wifi_SendString("ACK:EMERGENCY_STOP=1\r\n");
        PRINTF("\r\n[FRDM-UART] >>> EMERGENCY STOP ACTIVATED! Motors halted, steering centered <<<\r\n\r\n");
        return;
    }
    else if (strcmp(key, "ENGINE_ENABLED") == 0) {
        g_engine_enabled = (val > 0.5f);
        if (!g_engine_enabled) {
            HbridgeSpeed(&g_hbridge, 0, 0);
        }
        char ack[64];
        snprintf(ack, sizeof(ack), "ACK:ENGINE_ENABLED=%d\r\n", g_engine_enabled ? 1 : 0);
        Wifi_SendString(ack);
        PRINTF("[FRDM-UART] Engine state changed: %s\r\n", g_engine_enabled ? "RUNNING" : "STOPPED");
        return;
    }

    // 3. Search Parameter Registry Table
    bool found = false;
    for (size_t i = 0; i < CONFIG_PARAM_COUNT; i++) {
        if (strcmp(key, g_config_param_table[i].name) == 0) {
            double clamped = (double)val;
            if (clamped < g_config_param_table[i].min_val) clamped = g_config_param_table[i].min_val;
            if (clamped > g_config_param_table[i].max_val) clamped = g_config_param_table[i].max_val;

            *g_config_param_table[i].val_ptr = clamped;

            char ack[64];
            snprintf(ack, sizeof(ack), "ACK:%s=%.4f\r\n", g_config_param_table[i].name, clamped);
            Wifi_SendString(ack);

            PRINTF("[FRDM-UART] Parameter updated: %s = %.4f (range [%.1f, %.1f])\r\n",
                   g_config_param_table[i].name, clamped,
                   g_config_param_table[i].min_val, g_config_param_table[i].max_val);
            found = true;
            break;
        }
    }

    if (!found) {
        PRINTF("[FRDM-UART] Warning: Unrecognized parameter key: \"%s\"\r\n", key);
        char err[64];
        snprintf(err, sizeof(err), "ERR:UNKNOWN_PARAM=%s\r\n", key);
        Wifi_SendString(err);
    }
}

/* ========================================================================= */
/* Main RX Processing Loop (Polled in Main)                                  */
/* ========================================================================= */
void Wifi_Process_Rx(void)
{
    static char packetBuffer[128];
    static uint8_t packetIdx = 0;

    while (s_rxHead != s_rxTail)
    {
        char c = s_rxRing[s_rxTail];
        s_rxTail = (s_rxTail + 1U) % RX_RING_BUFFER_SIZE;

        if (c == '\n' || c == '\r' || c == ';')
        {
            if (packetIdx > 0)
            {
                packetBuffer[packetIdx] = '\0';
                g_wifi_rx_packet_count++;
                g_wifi_rx_count = g_wifi_rx_packet_count;

                strncpy(g_wifi_last_rx_cmd, packetBuffer, sizeof(g_wifi_last_rx_cmd) - 1);
                g_wifi_last_rx_cmd[sizeof(g_wifi_last_rx_cmd) - 1] = '\0';

                PRINTF("[FRDM-UART] RX Packet #%lu: \"%s\"\r\n",
                       (unsigned long)g_wifi_rx_packet_count, packetBuffer);

                // Parse and execute parameter update
                Wifi_ParseCommand(packetBuffer);

                packetIdx = 0;
            }
        }
        else if ((uint8_t)c >= 32U && packetIdx < (sizeof(packetBuffer) - 1))
        {
            packetBuffer[packetIdx++] = c;
        }
    }
}