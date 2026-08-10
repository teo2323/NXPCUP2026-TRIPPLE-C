#include "wifi.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include "fsl_debug_console.h"

#define RX_BUF_SIZE 128

static volatile char rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_idx = 0;
static volatile bool rx_complete = false; 

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
    // example: send AT command to check if the module is responsive
    Wifi_SendString("AT\r\n");
}

void Wifi_SendString(const char *str)
{
    while (*str)
    {
        LPUART_SendChar(*str++);
    }
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
                PRINTF("Received message: %s\r\n", (const char *)rx_buf); // Procesează sau afișează mesajul
                rx_idx = 0U;
            }
        }
        else if ((uint8_t)c >= 32U && rx_idx < (uint8_t)(sizeof(rx_buf) - 1U))
        {
            rx_buf[rx_idx++] = c;
        }
    }
}