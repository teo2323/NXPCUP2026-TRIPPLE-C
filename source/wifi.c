/*
 * wifi.c — ESP32 UART Parameter Tuning Interface
 *
 * Handles bidirectional UART communication between the FRDM-MCXN947 and
 * an ESP32 module acting as a WiFi parameter tuning server.
 *
 * Hardware: LP_FLEXCOMM7 (LPUART) @ 115200 8N1
 *   FRDM RX: P3_2  (J9 Pin 20 / J7 Pin 8)  ← ESP32 GPIO17 (TX2)
 *   FRDM TX: P3_3  (J9 Pin 19 / J7 Pin 6)  → ESP32 GPIO16 (RX2)
 *   Shared GND
 *
 * Protocol:
 *   ESP → FRDM:  "NAME_OF_THE_PARAMETER=NEW_VALUE\n"
 *   FRDM → ESP:  "ACK:NAME_OF_THE_PARAMETER=NEW_VALUE\r\n"
 */

#include "Config.h"

#if WIFI_CONFIG_ENABLED

#include "wifi.h"
#include "peripherals.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "hbridge.h"
#include "servo.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ========================================================================= */
/* Global Variable Definitions (defaults match Config.h original values)     */
/* ========================================================================= */

// Steering 2-Zone PID coefficients
volatile double g_steering_kp_straight      = 1.0;
volatile double g_steering_kd_straight      = 10.0;
volatile double g_steering_kp_curve         = 2.0;
volatile double g_steering_kd_curve         = 2.0;

// Dual Steering Error Deadbands
volatile double g_steering_deadband_straight = 2.5;
volatile double g_steering_deadband_curve    = 1.0;

// Curve hysteresis thresholds
volatile double g_curve_enter_angle_deg     = 35.0;
volatile double g_curve_exit_angle_deg      = 5.0;
volatile double g_curve_exit_frames         = 2.0;

// Camera perspective offset
volatile double g_perspective_angle_offset  = 5.0;

// Physical steering limits & calibration offset
volatile double g_steering_limit_right      = 65.0;
volatile double g_steering_limit_left       = -65.0;
volatile double g_steering_offset           = 15.0;

// Motor speeds & dynamics
volatile double g_speed_right               = 100.0;
volatile double g_speed_left                = 100.0;
volatile double g_corner_brake_coeff        = 1.7;
volatile double g_diff_speed_coeff          = 0.38;
volatile double g_consecutive_curve_time_ms = 500.0;
volatile double g_consecutive_curve_speed   = 70.0;
volatile double g_min_curve_duration_ms     = 100.0;
volatile double g_max_single_curve_time_ms  = 450.0;

// Detection thresholds & decay
volatile double g_vertical_min_slope        = 0.375;
volatile double g_decay_factor              = 0.5;
volatile double g_no_line_max_lost_frames   = 5.0;

// Engine safety
volatile bool   g_engine_enabled            = false;
volatile bool   g_engine_state_changed      = false;

// Diagnostics
volatile uint32_t g_wifi_tx_count           = 0U;
volatile uint32_t g_wifi_rx_count           = 0U;
volatile uint32_t g_wifi_rx_packet_count    = 0U;
volatile uint32_t g_wifi_last_pong_seq      = 0U;
char              g_wifi_last_rx_cmd[64]    = "";

/* ========================================================================= */
/* Parameter Registry Table (table-driven command dispatcher)                */
/* ========================================================================= */
static const config_param_entry_t g_config_param_table[] = {
    /* name                          val_ptr                          min     max    default */
    {"STEERING_KP_STRAIGHT",      &g_steering_kp_straight,           0.0,   10.0,    1.0  },
    {"STEERING_KD_STRAIGHT",      &g_steering_kd_straight,           0.0,   50.0,   10.0  },
    {"STEERING_KP_CURVE",         &g_steering_kp_curve,              0.0,   15.0,    2.0  },
    {"STEERING_KD_CURVE",         &g_steering_kd_curve,              0.0,   10.0,    2.0  },
    {"STEERING_DEADBAND_STRAIGHT",&g_steering_deadband_straight,      0.0,   15.0,    2.5  },
    {"STEERING_DEADBAND_CURVE",   &g_steering_deadband_curve,        0.0,   10.0,    1.0  },
    {"CURVE_ENTER_ANGLE_DEG",     &g_curve_enter_angle_deg,          5.0,   70.0,   35.0  },
    {"CURVE_EXIT_ANGLE_DEG",      &g_curve_exit_angle_deg,           1.0,   40.0,    5.0  },
    {"CURVE_EXIT_FRAMES",         &g_curve_exit_frames,              1.0,   10.0,    2.0  },
    {"PERSPECTIVE_ANGLE_OFFSET",  &g_perspective_angle_offset,       0.0,   20.0,    5.0  },
    {"STEERING_LIMIT_RIGHT",      &g_steering_limit_right,           0.0,   90.0,   65.0  },
    {"STEERING_LIMIT_LEFT",       &g_steering_limit_left,          -90.0,    0.0,  -65.0  },
    {"STEERING_OFFSET",           &g_steering_offset,              -30.0,   30.0,   15.0  },
    {"SPEED_RIGHT",               &g_speed_right,                    0.0,  100.0,  100.0  },
    {"SPEED_LEFT",                &g_speed_left,                     0.0,  100.0,  100.0  },
    {"CORNER_BRAKE_COEFF",        &g_corner_brake_coeff,             0.0,    5.0,    1.7  },
    {"DIFF_SPEED_COEFF",          &g_diff_speed_coeff,               0.0,    2.0,    0.38 },
    {"CONSECUTIVE_CURVE_TIME_MS", &g_consecutive_curve_time_ms,      0.0, 3000.0,  500.0  },
    {"CONSECUTIVE_CURVE_SPEED",   &g_consecutive_curve_speed,        0.0,  100.0,   70.0  },
    {"MIN_CURVE_DURATION_MS",     &g_min_curve_duration_ms,          0.0, 1000.0,  100.0  },
    {"MAX_SINGLE_CURVE_TIME_MS",  &g_max_single_curve_time_ms,     100.0, 5000.0,  450.0  },
    {"VERTICAL_MIN_SLOPE",        &g_vertical_min_slope,             0.1,    3.0,    0.375},
    {"DECAY_FACTOR",              &g_decay_factor,                   0.0,    1.0,    0.5  },
    {"NO_LINE_MAX_LOST_FRAMES",   &g_no_line_max_lost_frames,        1.0,   30.0,    5.0  },
};
#define CONFIG_PARAM_COUNT (sizeof(g_config_param_table) / sizeof(g_config_param_table[0]))

/* ========================================================================= */
/* RX Ring Buffer (filled by ISR, drained by Wifi_Process_Rx)               */
/* ========================================================================= */
#define RX_RING_BUFFER_SIZE 256U
static volatile char     s_rxRing[RX_RING_BUFFER_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;

/* ========================================================================= */
/* LPUART7 Receive ISR                                                       */
/* ========================================================================= */
void LP_FLEXCOMM7_IRQHandler(void)
{
    uint32_t status = LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL);

    /* Clear overrun, framing, noise, or parity errors */
    if (status & (kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                  kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag))
    {
        LPUART_ClearStatusFlags(LP_FLEXCOMM7_PERIPHERAL,
                                kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    }

    /* Drain the hardware RX FIFO into the ring buffer */
    while ((LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_RxDataRegFullFlag) != 0U)
    {
        char c = (char)LPUART_ReadByte(LP_FLEXCOMM7_PERIPHERAL);
        uint16_t nextHead = (uint16_t)((s_rxHead + 1U) % RX_RING_BUFFER_SIZE);
        if (nextHead != s_rxTail)   /* drop on overflow */
        {
            s_rxRing[s_rxHead] = c;
            s_rxHead = nextHead;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

/* ========================================================================= */
/* Wifi_Init                                                                 */
/* ========================================================================= */
void Wifi_Init(void)
{
    s_rxHead             = 0U;
    s_rxTail             = 0U;
    g_wifi_rx_packet_count = 0U;
    g_wifi_rx_count      = 0U;
    g_wifi_tx_count      = 0U;

    /* Enable RX data-ready and overrun interrupts on LPUART7 */
    LPUART_EnableInterrupts(LP_FLEXCOMM7_PERIPHERAL,
                            kLPUART_RxDataRegFullInterruptEnable |
                            kLPUART_RxOverrunInterruptEnable);
    NVIC_SetPriority(LP_FLEXCOMM7_IRQn, 3U);
    EnableIRQ(LP_FLEXCOMM7_IRQn);
}

/* ========================================================================= */
/* Wifi_Flush_Tx                                                             */
/* ========================================================================= */
void Wifi_Flush_Tx(void)
{
    /* TX is handled synchronously via polling — nothing to flush */
}

/* ========================================================================= */
/* Wifi_SendString                                                           */
/* ========================================================================= */
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

/* ========================================================================= */
/* Wifi_SendPing                                                             */
/* ========================================================================= */
void Wifi_SendPing(uint32_t seq)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "PING=%lu\r\n", (unsigned long)seq);
    Wifi_SendString(buf);
}

/* ========================================================================= */
/* Wifi_SendTelemetry                                                        */
/* ========================================================================= */
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
/* Wifi_ParseCommand                                                         */
/* Format: "KEY=VALUE"  (whitespace trimmed automatically)                   */
/* ========================================================================= */
void Wifi_ParseCommand(const char *cmd)
{
    if (cmd == NULL) return;

    char buf[128];
    strncpy(buf, cmd, sizeof(buf) - 1U);
    buf[sizeof(buf) - 1U] = '\0';

    char *eq = strchr(buf, '=');
    if (!eq) return;

    *eq = '\0';
    char *key     = buf;
    char *val_str = eq + 1;

    /* Trim whitespace from key */
    while (*key == ' ' || *key == '\t' || *key == '\r' || *key == '\n') key++;
    char *k_end = key + strlen(key) - 1;
    while (k_end > key && (*k_end == ' ' || *k_end == '\t' || *k_end == '\r' || *k_end == '\n'))
    {
        *k_end = '\0';
        k_end--;
    }

    /* Trim whitespace from value */
    while (*val_str == ' ' || *val_str == '\t' || *val_str == '\r' || *val_str == '\n') val_str++;
    char *v_end = val_str + strlen(val_str) - 1;
    while (v_end > val_str && (*v_end == ' ' || *v_end == '\t' || *v_end == '\r' || *v_end == '\n'))
    {
        *v_end = '\0';
        v_end--;
    }

    float val = (float)atof(val_str);

    /* ------------------------------------------------------------------ */
    /* 1. Diagnostic ping / pong                                           */
    /* ------------------------------------------------------------------ */
    if (strcmp(key, "PING") == 0)
    {
        char resp[32];
        snprintf(resp, sizeof(resp), "PONG=%d\r\n", (int)val);
        Wifi_SendString(resp);
        // PRINTF("[FRDM-UART] Replied PONG=%d to ESP32\r\n", (int)val);
        return;
    }
    else if (strcmp(key, "PONG") == 0)
    {
        g_wifi_last_pong_seq = (uint32_t)val;
        // PRINTF("[FRDM-UART] RX PONG=%d from ESP32 (Round-trip OK!)\r\n", (int)val);
        return;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Safety & engine controls                                         */
    /* ------------------------------------------------------------------ */
    if (strcmp(key, "EMERGENCY_STOP") == 0)
    {
        g_engine_enabled = false;
        g_engine_state_changed = true;
        HbridgeSpeed(&g_hbridge, 0, 0);
        Steer(0.0 + (double)STEERING_OFFSET);
        Wifi_SendString("ACK:EMERGENCY_STOP=1\r\n");
        // PRINTF("\r\n[FRDM-UART] >>> EMERGENCY STOP ACTIVATED! Motors halted, steering centered <<<\r\n\r\n");
        return;
    }
    else if (strcmp(key, "ENGINE_ENABLED") == 0)
    {
        g_engine_enabled = (val > 0.5f);
        g_engine_state_changed = true;
        if (!g_engine_enabled)
        {
            HbridgeSpeed(&g_hbridge, 0, 0);
        }
        char ack[64];
        snprintf(ack, sizeof(ack), "ACK:ENGINE_ENABLED=%d\r\n", g_engine_enabled ? 1 : 0);
        Wifi_SendString(ack);
        // PRINTF("[FRDM-UART] Engine state changed: %s\r\n", g_engine_enabled ? "RUNNING" : "STOPPED");
        return;
    }

    /* ------------------------------------------------------------------ */
    /* 3. Search parameter registry table                                  */
    /* ------------------------------------------------------------------ */
    bool found = false;
    for (size_t i = 0U; i < CONFIG_PARAM_COUNT; i++)
    {
        if (strcmp(key, g_config_param_table[i].name) == 0)
        {
            double clamped = (double)val;
            if (clamped < g_config_param_table[i].min_val) clamped = g_config_param_table[i].min_val;
            if (clamped > g_config_param_table[i].max_val) clamped = g_config_param_table[i].max_val;

            *g_config_param_table[i].val_ptr = clamped;

            char ack[64];
            snprintf(ack, sizeof(ack), "ACK:%s=%.4f\r\n", g_config_param_table[i].name, clamped);
            Wifi_SendString(ack);

            // PRINTF("[FRDM-UART] Parameter updated: %s = %.4f (range [%.2f, %.2f])\r\n",
            //        g_config_param_table[i].name, clamped,
            //        g_config_param_table[i].min_val,
            //        g_config_param_table[i].max_val);
            found = true;
            break;
        }
    }

    if (!found)
    {
        // PRINTF("[FRDM-UART] Warning: Unrecognized parameter key: \"%s\"\r\n", key);
        char err[64];
        snprintf(err, sizeof(err), "ERR:UNKNOWN_PARAM=%s\r\n", key);
        Wifi_SendString(err);
    }
}

/* ========================================================================= */
/* Wifi_Process_Rx — drain ring buffer, assemble lines, dispatch parser      */
/* ========================================================================= */
void Wifi_Process_Rx(void)
{
    static char    packetBuffer[128];
    static uint8_t packetIdx = 0U;

    while (s_rxHead != s_rxTail)
    {
        char c = s_rxRing[s_rxTail];
        s_rxTail = (uint16_t)((s_rxTail + 1U) % RX_RING_BUFFER_SIZE);

        if (c == '\n' || c == '\r' || c == ';')
        {
            if (packetIdx > 0U)
            {
                packetBuffer[packetIdx] = '\0';
                g_wifi_rx_packet_count++;
                g_wifi_rx_count = g_wifi_rx_packet_count;

                strncpy(g_wifi_last_rx_cmd, packetBuffer, sizeof(g_wifi_last_rx_cmd) - 1U);
                g_wifi_last_rx_cmd[sizeof(g_wifi_last_rx_cmd) - 1U] = '\0';

                // PRINTF("[FRDM-UART] RX Packet #%lu: \"%s\"\r\n",
                //        (unsigned long)g_wifi_rx_packet_count, packetBuffer);

                Wifi_ParseCommand(packetBuffer);

                packetIdx = 0U;
            }
        }
        else if ((uint8_t)c >= 32U && packetIdx < (sizeof(packetBuffer) - 1U))
        {
            packetBuffer[packetIdx++] = c;
        }
    }
}

#endif /* WIFI_CONFIG_ENABLED */
