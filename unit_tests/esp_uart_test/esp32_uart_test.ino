/*
 * ESP32 to FRDM-MCXN947 Minimal UART Sniffer Test
 *
 * Hardware Connections (Factory Header J9 recommended):
 *   ESP32 GND          <---> FRDM J9 Pin 22 (GND) or J7 Pin 10
 *   ESP32 GPIO17 (TX2) ----> FRDM J9 Pin 20 (P3_2 / RX) or J7 Pin 8
 *   ESP32 GPIO16 (RX2) <---- FRDM J9 Pin 19 (P3_3 / TX) or J7 Pin 6 (Optional for echo)
 */

#define RX2_PIN 16
#define TX2_PIN 17
#define UART_BAUD 115200

unsigned long lastSendTime = 0;
uint32_t sendCount = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n==================================================");
    Serial.println("  ESP32 Minimal UART Transmitter (Sniffer Test)   ");
    Serial.println("==================================================");

    Serial2.begin(UART_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
    Serial.printf("[ESP32] Serial2 initialized on RX=GPIO%d, TX=GPIO%d at %d baud.\n", 
                  RX2_PIN, TX2_PIN, UART_BAUD);
    Serial.println("[ESP32] Sending byte 'U' (0x55) every 500 ms...\n");
}

void loop() {
    unsigned long currentMillis = millis();

    // 1. Send test byte 'U' (0x55) every 500 ms
    if (currentMillis - lastSendTime >= 500) {
        lastSendTime = currentMillis;
        sendCount++;

        // Send raw byte 'U' (0x55 produces alternating 01010101 square wave)
        Serial2.write('U');
        Serial2.flush();

        Serial.printf("[ESP32 TX -> FRDM] Sent byte #%u: 'U' (0x55)\n", sendCount);
    }

    // 2. Read any echoed bytes returned by the FRDM board
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.printf("  >>> [ESP32 RX <- FRDM ECHO] Received: '%c' (0x%02X)\n", c, (uint8_t)c);
    }
}
