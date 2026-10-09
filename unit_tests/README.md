# Unit Tests & Hardware Diagnostic Tools

This directory contains standalone test programs and diagnostic sketches for verifying peripherals, communication buses, and hardware modules.

## Directories

### `esp_uart_test/`
* **File**: `esp32_uart_test.ino`
* **Purpose**: Minimal sniffer and UART loopback/echo test between ESP32 and FRDM-MCXN947 board at 115200 baud.
* **Pin Connections**:
  * ESP32 GND $\leftrightarrow$ FRDM J9 Pin 22 (GND) or J7 Pin 10
  * ESP32 GPIO17 (TX2) $\rightarrow$ FRDM J9 Pin 20 (P3_2 / RX) or J7 Pin 8
  * ESP32 GPIO16 (RX2) $\leftarrow$ FRDM J9 Pin 19 (P3_3 / TX) or J7 Pin 6
* **FRDM-side Equivalent**:
  * In `source/main.c`, set `#define TEST_ESP_UART_STANDALONE 1` to run the hardware register sniffer and toggle the green LED on every received byte.
