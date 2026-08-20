#include <Arduino.h>
// CRSF protocol for communicating with ELRS transmitter module
#include "crsf.h"

// For ESP32-S2, we need to create a HardwareSerial object
// ESP32-S2 has UART0 (Serial), UART1 available for general use
HardwareSerial crsfUart(1); // Use UART1

// CRSF Serial interface for ELRS TX module
// Alternative approach: Use separate TX/RX pins but wire them together externally
// This avoids ESP32-S2 pin management issues
// GPIO5 = TX, GPIO6 = RX (wire together externally for half-duplex)
CrsfSerial crsf(crsfUart, 6, 5, true); // RX=6, TX=5, half-duplex mode

// ELRS TX module expects 400kbaud by default, but can auto-detect other rates
#define CRSF_BAUDRATE 400000
#define CRSF_RC_CHANNEL_COUNT 16
#define CRSF_UPDATE_INTERVAL 14 // milliseconds, for ~70Hz update rate

// Simulation variables
unsigned long lastUpdate = 0;
float time_seconds = 0.0;

// Channel simulation variables
float ch1_normalized = 0.0; // Simulated joint angle 1
float ch2_normalized = 0.0; // Simulated joint angle 2
float ch3_normalized = 0.0; // Simulated joint angle 3
float ch4_normalized = 0.0; // Simulated joint angle 4
bool pwr_bool = true;        // Power switch state

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  delay(2000); // Give time for serial monitor to connect

  Serial.println("=== CRSF TX Half-Duplex Test ===");
  Serial.print("ESP32-S2 Chip Model: ");
  Serial.println(ESP.getChipModel());
  Serial.println("Using GPIO5(TX) and GPIO6(RX) - wire together for half-duplex");

  // Initialize CRSF communication with ELRS TX module
  crsf.begin(CRSF_BAUDRATE);
  Serial.println("CRSF initialized for ELRS TX module");

  Serial.println("Starting CRSF channel transmission...");
  Serial.print("Update interval: ");
  Serial.print(CRSF_UPDATE_INTERVAL);
  Serial.println("ms");
}

void loop() {
  // Update timing for simulation
  unsigned long currentTime = millis();
  if (currentTime - lastUpdate >= CRSF_UPDATE_INTERVAL) {
    time_seconds = currentTime / 1000.0;
    
    // Simulate channels 1-4 with different patterns
    // Channel 1: Slow sine wave (-1.0 to 1.0)
    ch1_normalized = sin(time_seconds * 0.5) * 0.8;
    
    // Channel 2: Faster cosine wave
    ch2_normalized = cos(time_seconds * 1.2) * 0.6;
    
    // Channel 3: Triangle wave
    float triangle_phase = fmod(time_seconds * 0.8, 2.0);
    if (triangle_phase < 1.0) {
      ch3_normalized = (triangle_phase * 2.0 - 1.0) * 0.7; // -0.7 to 0.7
    } else {
      ch3_normalized = (3.0 - triangle_phase * 2.0) * 0.7; // 0.7 to -0.7
    }
    
    // Channel 4: Square wave alternating every 2 seconds
    ch4_normalized = (fmod(time_seconds, 4.0) < 2.0) ? 0.5 : -0.5;
    
    // Toggle power switch every 10 seconds for demonstration
    pwr_bool = (fmod(time_seconds, 20.0) < 10.0);
    
    // Debug output every 2 seconds
    if (fmod(time_seconds, 2.0) < 0.02) {
      Serial.print("Ch1:");
      Serial.print(ch1_normalized, 3);
      Serial.print(" Ch2:");
      Serial.print(ch2_normalized, 3);
      Serial.print(" Ch3:");
      Serial.print(ch3_normalized, 3);
      Serial.print(" Ch4:");
      Serial.print(ch4_normalized, 3);
      Serial.print(" Pwr:");
      Serial.println(pwr_bool ? "ON" : "OFF");
    }
    
    lastUpdate = currentTime;
  }

  // Set channel values
  crsf.setChannelFloat(1, ch1_normalized);
  crsf.setChannelFloat(2, ch2_normalized);
  crsf.setChannelFloat(3, ch3_normalized);
  crsf.setChannelFloat(4, ch4_normalized);
  
  // Channel 5: Power switch (explicit high/low)
  crsf.setChannelUs(5, pwr_bool ? 2000 : 1000);
  
  // Channels 6-16: Set to neutral/center position
  for (int i = 6; i <= 16; i++) {
    crsf.setChannelFloat(i, 0.0);
  }

  // Update CRSF transmission (this maintains the proper frame rate)
  crsf.update();
}