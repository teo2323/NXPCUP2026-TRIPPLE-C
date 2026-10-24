/*
 * ESP32 Diagnostic & Hello World Test
 * 
 * Instructions:
 * 1. Open this file in Arduino IDE.
 * 2. Select your board (e.g., "ESP32 Dev Module" or your specific ESP32 variant).
 * 3. Select the COM port corresponding to your ESP32.
 * 4. Open the Serial Monitor (Tools -> Serial Monitor) and set the baud rate to 115200.
 * 5. Upload the sketch.
 */

#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // GPIO2 is the standard onboard LED on most ESP32 DevKit boards
#endif

uint32_t counter = 0;

void setup() {
  // Initialize USB Serial at 115200 baud
  Serial.begin(115200);
  delay(1500); // Allow time for USB CDC / Serial Monitor to connect

  // Configure onboard LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Print diagnostic header
  Serial.println("\n==========================================");
  Serial.println("       ESP32 HELLO WORLD & HEALTH CHECK   ");
  Serial.println("==========================================");
  Serial.print("Chip Model:        "); Serial.println(ESP.getChipModel());
  Serial.print("Chip Revision:     "); Serial.println(ESP.getChipRevision());
  Serial.print("CPU Frequency:     "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.print("Flash Chip Size:   "); Serial.print(ESP.getFlashChipSize() / (1024 * 1024)); Serial.println(" MB");
  Serial.print("Free Heap Memory:  "); Serial.print(ESP.getFreeHeap() / 1024); Serial.println(" KB");
  Serial.println("==========================================\n");
}

void loop() {
  counter++;

  // Toggle onboard LED
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // Print Hello World message with counter and uptime
  Serial.printf("[%lu s] Hello World from ESP32! (Heartbeat #%lu)\n", 
                millis() / 1000, 
                counter);
}
