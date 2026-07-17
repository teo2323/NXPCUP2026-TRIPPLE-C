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
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootClocks();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* Servo Test: 3 seconds right, 7 seconds left */
    TestServoRightLeft();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 27U,
                GPIO0, 26U);
    //HbridgeSpeed(&g_hbridge, SPEED_LEFT, SPEED_RIGHT);
    CTIMER_StartTimer(CTIMER0_PERIPHERAL);
    HbridgeSpeed(&g_hbridge, 75);

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