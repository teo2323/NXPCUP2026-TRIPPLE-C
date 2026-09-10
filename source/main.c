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
#include <math.h>
#include "wifi.h"
#include "ultrasonic.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define MAX_VECTORS          10
#define AUTOMATED_BASE_SPEED 40
#define OBSTACLE_STOP_DIST_CM 20.0f
#define OBSTACLE_CONFIRM_COUNT 2 

static pixy_t cam1;
static volatile bool g_obstacle_brake = false;

static void vSafetyTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t obstacle_detected_count = 0;

    for (;;)
    {
        float distance_cm = Ultrasonic_ReadDistanceCm();

        if (distance_cm >= 2.0f && distance_cm <= OBSTACLE_STOP_DIST_CM) {
            if (++obstacle_detected_count >= OBSTACLE_CONFIRM_COUNT) {
                obstacle_detected_count = OBSTACLE_CONFIRM_COUNT;
                g_obstacle_brake = true;
                HbridgeBrake(&g_hbridge);
                pixy_set_led(&cam1, 255, 0, 0); // Pixy Red LED
            }
        } else {
            if (obstacle_detected_count >= OBSTACLE_CONFIRM_COUNT) {
                pixy_set_led(&cam1, 0, 255, 0); // Pixy Green LED
            }
            obstacle_detected_count = 0;
            g_obstacle_brake = false;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

static void vTelemetryTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        Wifi_Process_Rx();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

static void vVisionTask(void *pvParameters)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t num_vectors;
    double last_steering_angle = 0.0;
    double previous_error = 0.0;
    bool was_tracking = false;

    static uint32_t g_horizontal_vector_count = 0U;
    static bool last_engine_state = false;
    static bool g_horiz_delay_in_progress = false;
    static uint32_t g_horiz_delay_start_cycles = 0U;
    static bool g_horiz_speed_reduced = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (g_obstacle_brake) {
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
            continue;
        }

        if (g_engine_enabled && !last_engine_state) {
            g_horizontal_vector_count = 0U;
            g_horiz_delay_in_progress = false;
            g_horiz_speed_reduced = false;
            pixy_set_led(&cam1, 0, 255, 0);
        }
        last_engine_state = g_engine_enabled;

        if (g_horiz_delay_in_progress) {
            uint32_t now_cycles = MSDK_GetCpuCycleCount();
            if ((now_cycles - g_horiz_delay_start_cycles) >= SystemCoreClock) {
                g_horiz_delay_in_progress = false;
                g_horiz_speed_reduced = true;
                pixy_set_led(&cam1, 255, 255, 0);
            }
        }

        int current_speed = 0;
        if (g_horiz_speed_reduced) {
            current_speed = g_engine_enabled ? ((int)g_motor_speed / 2) : 0;
        } else {
            current_speed = g_engine_enabled ? (int)g_motor_speed : 0;
        }
        HbridgeSpeed(&g_hbridge, current_speed, current_speed);

        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            uint32_t horiz_in_frame = (uint32_t)detection_count_horizontal_vectors(vectors, num_vectors);
            if (horiz_in_frame > 0) {
                g_horizontal_vector_count += horiz_in_frame;
                if (g_engine_enabled && g_horizontal_vector_count > 2U && !g_horiz_delay_in_progress && !g_horiz_speed_reduced) {
                    g_horiz_delay_in_progress = true;
                    g_horiz_delay_start_cycles = MSDK_GetCpuCycleCount();
                    pixy_set_led(&cam1, 0, 255, 255);
                }
            }

            dual_line_detection_result_t det;
            detection_process_dual_lines(vectors, num_vectors, &det);

            if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                double error = det.steering_angle;
                if (!was_tracking) {
                    previous_error = error;
                    was_tracking = true;
                }
                double derivative = error - previous_error;
                previous_error = error;

                double steer_angle = (STEERING_P * error) + (STEERING_D * derivative);
                if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                Steer(steer_angle);
                last_steering_angle = steer_angle;
            } else {
                was_tracking = false;
                turn_track_result_t turn;
                if (detection_detect_turn_track(vectors, num_vectors, &turn)) {
                    double error = turn.steering_angle;
                    double derivative = error - previous_error;
                    previous_error = error;

                    double steer_angle = (STEERING_P * error) + (STEERING_D * derivative);
                    if (steer_angle > STEERING_LIMIT_RIGHT) steer_angle = STEERING_LIMIT_RIGHT;
                    if (steer_angle < STEERING_LIMIT_LEFT)  steer_angle = STEERING_LIMIT_LEFT;

                    Steer(steer_angle);
                    last_steering_angle = steer_angle;
                } else {
                    last_steering_angle *= g_decay_factor;
                    if (fabs(last_steering_angle) < 1.0) {
                        last_steering_angle = 0.0;
                    }
                    Steer(last_steering_angle);
                }
            }

            static uint32_t telemetry_tick = 0U;
            if (++telemetry_tick >= 6U) {
                telemetry_tick = 0U;
                uint8_t line_cnt = 0U;
                const char *which_str = "NONE";

                if (det.valid_vectors > 0 && (det.left_line_present || det.right_line_present)) {
                    line_cnt = det.both_lines_present ? 2U : 1U;
                    which_str = det.both_lines_present ? "BOTH" : (det.left_line_present ? "LEFT" : "RIGHT");
                } else {
                    turn_track_result_t turn_t;
                    if (detection_detect_turn_track(vectors, num_vectors, &turn_t)) {
                        line_cnt = 0U;
                        which_str = turn_t.turn_left ? "TURN_LEFT" : "TURN_RIGHT";
                    }
                }

                int lx0 = det.left_line_present ? (int)det.left_line.vector.x0 : 0;
                int ly0 = det.left_line_present ? (int)det.left_line.vector.y0 : 0;
                int lx1 = det.left_line_present ? (int)det.left_line.vector.x1 : 0;
                int ly1 = det.left_line_present ? (int)det.left_line.vector.y1 : 0;
                int rx0 = det.right_line_present ? (int)det.right_line.vector.x0 : 0;
                int ry0 = det.right_line_present ? (int)det.right_line.vector.y0 : 0;
                int rx1 = det.right_line_present ? (int)det.right_line.vector.x1 : 0;
                int ry1 = det.right_line_present ? (int)det.right_line.vector.y1 : 0;

                Wifi_SendTelemetry(line_cnt, which_str, num_vectors, g_horizontal_vector_count, last_steering_angle, lx0, ly0, lx1, ly1, rx0, ry0, rx1, ry1);
            }
        }

        /* Always actuate steering servo on every RTOS 16ms cycle, regardless of camera I2C frame drops */
        Steer(last_steering_angle);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(16));
    }
}

int main(void)
{
    BOARD_InitHardware();
    BOARD_InitBootClocks();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 27U,
                GPIO0, 26U,
                GPIO0, 28U,
                GPIO0, 31U
    );
    extern uint32_t SystemCoreClock;

    CTIMER_StartTimer(CTIMER0_PERIPHERAL);

    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 0, 255, 0);

    HbridgeSpeed(&g_hbridge, 0, 0);
    Steer(0.0);

    Wifi_Init();
    Ultrasonic_Init();

    /* Create FreeRTOS Tasks */
    xTaskCreate(vSafetyTask,    "Safety",    configMINIMAL_STACK_SIZE + 256, NULL, 4, NULL);
    xTaskCreate(vTelemetryTask, "Telemetry", configMINIMAL_STACK_SIZE + 384, NULL, 3, NULL);
    xTaskCreate(vVisionTask,    "Vision",    configMINIMAL_STACK_SIZE + 896, NULL, 3, NULL);

    /* Start FreeRTOS Scheduler */
    vTaskStartScheduler();

    while (1)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    /* Emergency brake on stack overflow */
    HbridgeBrake(&g_hbridge);
    pixy_set_led(&cam1, 255, 0, 0); // Pixy Solid Red LED
    PRINTF("\r\n[CRITICAL ERROR] FreeRTOS Stack Overflow in Task: %s!\r\n", pcTaskName ? pcTaskName : "UNKNOWN");

    for (;;)
    {
        /* Trap CPU safely */
    }
}