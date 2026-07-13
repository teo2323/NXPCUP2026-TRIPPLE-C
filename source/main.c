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
#include <math.h>
#include "fsl_lpuart.h"
#include "fsl_clock.h"
#include "fsl_reset.h"

// Define a noinit section variable for the magic word
__attribute__((section(".noinit"))) volatile uint32_t boot_magic;

#define BOOTLOADER_MAGIC_WORD 0x5A5A5A5AU

static const char ota_trigger_str[] = "OTA_UPDATE_START";
static size_t ota_trigger_idx = 0;

void parse_ota_byte(uint8_t byte)
{
    if (byte == ota_trigger_str[ota_trigger_idx]) {
        ota_trigger_idx++;
        if (ota_trigger_idx == sizeof(ota_trigger_str) - 1) {
            // Trigger detected!
            PRINTF("OTA Trigger Detected! Rebooting into ROM bootloader...\r\n");
            
            // Transmit ACK
            const char ack_str[] = "ACK_BOOTLOADER";
            for (size_t i = 0; i < sizeof(ack_str) - 1; i++) {
                while (!(LPUART_GetStatusFlags(LPUART3) & kLPUART_TxDataRegEmptyFlag));
                LPUART_WriteByte(LPUART3, ack_str[i]);
            }
            
            // Wait for completion of transmission
            while (!(LPUART_GetStatusFlags(LPUART3) & kLPUART_TransmissionCompleteFlag));
            
            // Write magic word to SRAM
            boot_magic = BOOTLOADER_MAGIC_WORD;
            
            // Execute system reset
            NVIC_SystemReset();
        }
    } else {
        if (byte == ota_trigger_str[0]) {
            ota_trigger_idx = 1;
        } else {
            ota_trigger_idx = 0;
        }
    }
}

void LPUART3_Init(void)
{
    // 1. Enable clock gate for LP_FLEXCOMM3
    CLOCK_EnableClock(kCLOCK_LPFlexComm3);
    
    // 2. Set clock divider for FLEXCOMM3 to 1
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom3Clk, 1U);
    
    // 3. Attach FRO12M clock to FLEXCOMM3
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    
    // 4. Clear peripheral reset for FLEXCOMM3
    RESET_ClearPeripheralReset(kFC3_RST_SHIFT_RSTn);
    
    // 5. Initialize LPUART3
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    
    LPUART_Init(LPUART3, &config, 12000000U);
}

#define MAX_VECTORS 10

void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;

    // Calculate vector length (magnitude)
    double length = sqrt((double)(dx * dx + dy * dy));

    // Calculate direction angle in degrees relative to the vertical axis (-dy).
    // Straight forward is 0 degrees. Left is negative, right is positive.
    double angle_deg = 0.0;
    if (dy != 0) {
        angle_deg = atan2((double)dx, (double)(-dy)) * 180.0 / 3.141592653589793;
    } else {
        angle_deg = (dx >= 0) ? 90.0 : -90.0;
    }

    // Split into integer and fractional parts for printing (avoiding %f format)
    int len_int = (int)length;
    int len_frac = (int)((length - len_int) * 100.0);
    if (len_frac < 0) len_frac = -len_frac;

    int ang_int = (int)angle_deg;
    int ang_frac = (int)((angle_deg - (double)ang_int) * 100.0);
    if (ang_frac < 0) ang_frac = -ang_frac;

    PRINTF("Vector [%u] details:\r\n", (unsigned)index);
    PRINTF("  Start: (%u, %u) -> End: (%u, %u)\r\n", (unsigned)x0, (unsigned)y0, (unsigned)x1, (unsigned)y1);
    PRINTF("  dx: %d, dy: %d\r\n", dx, dy);
    PRINTF("  Length: %d.%02d px\r\n", len_int, len_frac);
    PRINTF("  Angle: %d.%02d deg\r\n", ang_int, ang_frac);
}

int main(void)
{
    // Check for OTA bootloader trigger
    if (boot_magic == BOOTLOADER_MAGIC_WORD) {
        boot_magic = 0; // Clear it to avoid infinite loop
        
        // Define ROM API structure
        typedef struct {
            const uint32_t version;
            const char *copyright;
            void (*runBootloader)(void *arg);
        } bootloader_api_entry_t;

        #define g_bootloaderTree (*(bootloader_api_entry_t **)0x1303FC34)
        
        // Disable interrupts to ensure a clean jump
        __disable_irq();
        
        // Jump to bootloader
        g_bootloaderTree->runBootloader(NULL);
    }

    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    // Initialize LPUART3 for ESP32 Wi-Fi update interface
    LPUART3_Init();

    /* Servo Test: 3 seconds right, 7 seconds left */
    TestServoRightLeft();

//    HbridgeInit(&g_hbridge,
//                CTIMER0_PERIPHERAL,
//                CTIMER0_PWM_PERIOD_CH,
//                CTIMER0_PWM_1_CHANNEL,
//                CTIMER0_PWM_2_CHANNEL,
//                GPIO0, 24U,
//                GPIO0, 27U);
//    HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);

//    Esc esc1, esc2;
//    EscInit(&esc1, CTIMER2_PERIPHERAL, CTIMER2_PWM_PERIOD_CH, kCTIMER_Match_1);
//    EscInit(&esc2, CTIMER2_PERIPHERAL, CTIMER2_PWM_PERIOD_CH, kCTIMER_Match_3);
//    EscSetSpeed(&esc1, 79.0);
//    EscBrake(&esc1);
//    EscSetSpeed(&esc2, 59.0);
//    EscBrake(&esc2);

//    TestServo();
    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 255, 0, 0);

   volatile double steer = 0;
    while (1)
    {
        // Check for incoming data from ESP32 on LPUART3
        if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3)) {
            uint8_t byte = LPUART_ReadByte(LPUART3);
            parse_ota_byte(byte);
        }

    	if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
    	        double angle = 0.0;
    	        for (size_t i = 0; i < num_vectors; i++) {
    	            uint16_t x0 = vectors[4*i + 0];
    	            uint16_t y0 = vectors[4*i + 1];
    	            uint16_t x1 = vectors[4*i + 2];
    	            uint16_t y1 = vectors[4*i + 3];

    	            // Print vector details
    	            print_vector_details(x0, y0, x1, y1, i);

    	            // Calculate slope, protecting against division by zero
    	            double diff_y = (double)y0 - (double)y1;
    	            if (diff_y != 0.0) {
    	                double m = ((double)x0 - (double)x1) / diff_y;
    	                angle += m;
    	            } else {
    	                // If dy is zero, slope is infinite. Assign a large value based on direction of dx
    	                angle += ((double)x0 - (double)x1 >= 0) ? 999.0 : -999.0;
    	            }
    	        }
    	        angle *= -1;

    	        // Safe print representation of double (since %f is not enabled)
    	        if (angle < 0) {
    	            int ang_int = (int)(-angle);
    	            int ang_frac = (int)((-angle - ang_int) * 100.0);
    	            PRINTF("Calculated Steering Angle: -%d.%02d\r\n", ang_int, ang_frac);
    	        } else {
    	            int ang_int = (int)angle;
    	            int ang_frac = (int)((angle - ang_int) * 100.0);
    	            PRINTF("Calculated Steering Angle: %d.%02d\r\n", ang_int, ang_frac);
    	        }

    	        if(angle > 0)
    	        	angle *= STEERING_P_RIGHT;
    	        else{
    	        	angle *= STEERING_P_LEFT;
    	        }
    	        if (angle > STEERING_LIMIT_RIGHT){
    	        	angle = STEERING_LIMIT_RIGHT;
    	        }
    	        if (angle < STEERING_LIMIT_LEFT){
					angle = STEERING_LIMIT_LEFT;
				}
    	        if(num_vectors !=0)
    	        	Steer(angle + STEERING_OFFSET);
    	    }

//        HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);

    }
}
