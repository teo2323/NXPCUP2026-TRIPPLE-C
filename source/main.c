#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"
#include "detection.h"
#include "wifi.h"
#include <math.h>

/* Set to 1 to run the isolated ESP32 <-> FRDM UART test (bypasses camera & motors).
 * Set to 0 to run the complete robot competition software with live Wi-Fi parameter tuning. */
#define TEST_ESP_UART_STANDALONE 0

#define MAX_VECTORS 10

static inline double compute_variable_pid(double error, double *previous_error, bool is_sharp_turn)
{
    /* D term: calculated from error difference */
    double derivative = error - *previous_error;
    *previous_error   = error;

    /* 2-Zone Gain Selection: Straight vs Curve */
    double kp, kd;
    if (fabs(error) > CURVE_ERROR_THRESHOLD || is_sharp_turn) {
        kp = STEERING_KP_CURVE;
        kd = STEERING_KD_CURVE;
    } else {
        kp = STEERING_KP_STRAIGHT;
        kd = STEERING_KD_STRAIGHT;
    }

    /* Variable P + D output */
    double steer_angle = (kp * error) + (kd * derivative);

    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

    return steer_angle;
}

int main(void)
{
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

#if TEST_ESP_UART_STANDALONE
    PRINTF("\r\n======================================================\r\n");
    PRINTF("  FRDM-MCXN947 Minimal UART Sniffer & Connectivity Test\r\n");
    PRINTF("  Pins: P3_2 (J9[20] / J7[8]) = RX | P3_3 (J9[19] / J7[6]) = TX @ 115200\r\n");
    PRINTF("  LED: Green LED (P0_27) toggles on EVERY byte received\r\n");
    PRINTF("  Listening for incoming raw bytes from ESP32...\r\n");
    PRINTF("======================================================\r\n\r\n");

    /* Disable interrupts so sniffer directly reads the hardware register */
    DisableIRQ(LP_FLEXCOMM7_IRQn);
    LPUART_DisableInterrupts(LP_FLEXCOMM7_PERIPHERAL, (uint32_t)kLPUART_AllInterruptEnable);

    uint32_t byte_count = 0U;
    uint32_t heartbeat_counter = 0U;

    while (1)
    {
        uint32_t status = LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL);

        /* Error detection & reporting */
        if (status & (kLPUART_RxOverrunFlag | kLPUART_FramingErrorFlag | kLPUART_NoiseErrorFlag | kLPUART_ParityErrorFlag))
        {
            PRINTF("[UART ERROR] STAT=0x%08lX -> FE(Framing)=%d | OR(Overrun)=%d | NF(Noise)=%d | PF(Parity)=%d\r\n",
                   (unsigned long)status,
                   (status & kLPUART_FramingErrorFlag) ? 1 : 0,
                   (status & kLPUART_RxOverrunFlag) ? 1 : 0,
                   (status & kLPUART_NoiseErrorFlag) ? 1 : 0,
                   (status & kLPUART_ParityErrorFlag) ? 1 : 0);

            LPUART_ClearStatusFlags(LP_FLEXCOMM7_PERIPHERAL,
                                    kLPUART_RxOverrunFlag | kLPUART_FramingErrorFlag |
                                    kLPUART_NoiseErrorFlag | kLPUART_ParityErrorFlag);
        }

        /* Check if byte arrived in hardware register */
        if ((status & kLPUART_RxDataRegFullFlag) != 0U)
        {
            uint8_t ch = LPUART_ReadByte(LP_FLEXCOMM7_PERIPHERAL);
            byte_count++;

            /* Toggle onboard Green LED (P0_27) */
            GPIO_PortToggle(BOARD_INITPINS_LED_GREEN_GPIO, BOARD_INITPINS_LED_GREEN_PIN_MASK);

            if (ch >= 32U && ch <= 126U)
            {
                PRINTF("[RX SUCCESS #%lu] Byte: '%c' (0x%02X)\r\n",
                       (unsigned long)byte_count, (char)ch, (unsigned int)ch);
            }
            else if (ch == '\n')
            {
                PRINTF("[RX SUCCESS #%lu] Byte: '\\n' (0x0A) [NEWLINE]\r\n", (unsigned long)byte_count);
            }
            else if (ch == '\r')
            {
                PRINTF("[RX SUCCESS #%lu] Byte: '\\r' (0x0D)\r\n", (unsigned long)byte_count);
            }
            else
            {
                PRINTF("[RX SUCCESS #%lu] Byte: (0x%02X)\r\n", (unsigned long)byte_count, (unsigned int)ch);
            }

            /* Echo byte back out on TX (P3_3) so ESP32 receives confirmation */
            while ((LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL) & kLPUART_TxDataRegEmptyFlag) == 0U) {}
            LPUART_WriteByte(LP_FLEXCOMM7_PERIPHERAL, ch);
        }

        /* Periodic heartbeat every ~2 seconds */
        if (++heartbeat_counter >= 3000000U)
        {
            heartbeat_counter = 0U;
            PRINTF("[SNIFFER ACTIVE] Bytes RX so far: %lu | STAT: 0x%08lX\r\n",
                   (unsigned long)byte_count,
                   (unsigned long)LPUART_GetStatusFlags(LP_FLEXCOMM7_PERIPHERAL));
        }
    }
#else
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    /* Initialize H-bridge with bootcampWorkspace pins for new car */
    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 0, 255, 0); // Green LED indicates active mode

    Wifi_Init();

    /* Initial base drive speed and straight steering */
    int16_t initial_speed = g_engine_enabled ? (int16_t)g_motor_speed : 0;
    HbridgeSpeed(&g_hbridge, initial_speed, initial_speed);
    Steer(0 + STEERING_OFFSET);

    double last_steering_angle = 0.0;
    double previous_error      = 0.0;  // D term: stores last frame's error

    while (1)
    {
        Wifi_Process_Rx();

        /* Periodic Dual-Duplex Communication Test between FRDM and ESP32 (~1 Hz) */
        static uint32_t comm_test_tick = 0U;
        static uint32_t ping_seq = 0U;
        if (++comm_test_tick >= 60U)
        {
            comm_test_tick = 0U;
            ping_seq++;
            Wifi_SendPing(ping_seq);

            PRINTF("\r\n======================================================\r\n");
            PRINTF(" [DUAL-DUPLEX COMM TEST] FRDM <-> ESP32 (LPUART7 @ 115200)\r\n");
            PRINTF("  FRDM TX -> ESP32: %lu packets sent (Sent PING #%lu)\r\n",
                   (unsigned long)g_wifi_tx_count, (unsigned long)ping_seq);
            PRINTF("  FRDM RX <- ESP32: %lu packets received\r\n",
                   (unsigned long)g_wifi_rx_count);
            if (g_wifi_rx_count > 0U) {
                PRINTF("  >>> STATUS: [FULL-DUPLEX ACTIVE - BIDIRECTIONAL OK] <<<\r\n");
                PRINTF("  Last Msg from ESP32: \"%s\"\r\n", g_wifi_last_rx_cmd);
            } else {
                PRINTF("  >>> STATUS: [WAITING FOR ESP32 RX - TX ONLY] <<<\r\n");
                PRINTF("  Check: ESP32 TX (GPIO17) -> FRDM RX (P3_2), Shared GND\r\n");
            }
            PRINTF("======================================================\r\n\r\n");
        }

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            Wifi_Process_Rx();

            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            bool is_sharp_turn = false;

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                if (det.sharp_turn_detected) {
                    is_sharp_turn = true;
                }

                /* 2-Zone Variable PID steering control */
                double steer_angle = compute_variable_pid(det.steering_angle, &previous_error, is_sharp_turn);

                Steer(steer_angle + STEERING_OFFSET);
                last_steering_angle = steer_angle;
            }
            else {
                /* 0 track lines detected -> search for horizontal turn-track fallback vector */
                turn_track_result_t turn;
                if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
                    /* Horizontal fallback vector indicates a sharp turn in progress */
                    is_sharp_turn = true;

                    /* 2-Zone Variable PID steering control */
                    double steer_angle = compute_variable_pid(turn.steering_angle, &previous_error, is_sharp_turn);

                    Steer(steer_angle + STEERING_OFFSET);
                    last_steering_angle = steer_angle;
                }
                else {
                    /* No vectors detected -> gently decay angle using DECAY_FACTOR (0.9) */
                    last_steering_angle *= DECAY_FACTOR;
                    if (fabs(last_steering_angle) < 1.0) {
                        last_steering_angle = 0.0;
                    }
                    Steer(last_steering_angle + STEERING_OFFSET);
                }
            }

            /* Dynamic speed control based on engine state and sharp turn detection */
            int16_t nominal_speed = g_engine_enabled ? (int16_t)g_motor_speed : 0;
            int16_t current_speed_left  = nominal_speed;
            int16_t current_speed_right = nominal_speed;

            if (is_sharp_turn && g_engine_enabled) {
                current_speed_left  = (int16_t)((double)nominal_speed * SHARP_TURN_SPEED_COEFF);
                current_speed_right = (int16_t)((double)nominal_speed * SHARP_TURN_SPEED_COEFF);
            }

            HbridgeSpeed(&g_hbridge, current_speed_left, current_speed_right);

            /* Send Telemetry to ESP32 over UART at ~10 Hz rate (every 6 camera frames at 60FPS) */
            static uint32_t telemetry_tick = 0U;
            if (++telemetry_tick >= 6U)
            {
                telemetry_tick = 0U;

                uint8_t line_cnt = 0U;
                const char *which_str = "NONE";

                if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present))
                {
                    if (det.both_lines_present) {
                        line_cnt = 2U;
                        which_str = "BOTH";
                    } else if (det.left_line_present) {
                        line_cnt = 1U;
                        which_str = "LEFT";
                    } else {
                        line_cnt = 1U;
                        which_str = "RIGHT";
                    }
                }
                else
                {
                    turn_track_result_t turn_t;
                    if (detection_detect_turn_track(vectors, num_vectors, &turn_t)) {
                        line_cnt = 0U;
                        which_str = turn_t.turn_left ? "TURN_LEFT" : "TURN_RIGHT";
                    } else {
                        line_cnt = 0U;
                        which_str = "NONE";
                    }
                }

                int lx0 = 0, ly0 = 0, lx1 = 0, ly1 = 0;
                int rx0 = 0, ry0 = 0, rx1 = 0, ry1 = 0;

                if (det.left_line_present) {
                    lx0 = (int)det.left_line.vector.x0;
                    ly0 = (int)det.left_line.vector.y0;
                    lx1 = (int)det.left_line.vector.x1;
                    ly1 = (int)det.left_line.vector.y1;
                }
                if (det.right_line_present) {
                    rx0 = (int)det.right_line.vector.x0;
                    ry0 = (int)det.right_line.vector.y0;
                    rx1 = (int)det.right_line.vector.x1;
                    ry1 = (int)det.right_line.vector.y1;
                }

                Wifi_SendTelemetry(line_cnt, which_str, num_vectors, 0U, last_steering_angle, lx0, ly0, lx1, ly1, rx0, ry0, rx1, ry1);
            }
        } else {
            SDK_DelayAtLeastUs(16000U, SystemCoreClock);
        }
    }
#endif
}
