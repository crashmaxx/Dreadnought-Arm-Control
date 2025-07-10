#include <Arduino.h>
#include <SimpleFOC.h>

// Define the PWM pin for the encoder
#define ENCODER_PWM_PIN 4

// Create the PWM sensor object (30us = 0 deg, 1360us = 359 deg)
MagneticSensorPWM encoder = MagneticSensorPWM(ENCODER_PWM_PIN, 30, 1360);
void doPWM() {encoder.handlePWM();}

void setup() {
  Serial.begin(115200);
  encoder.init();
  encoder.enableInterrupt(doPWM); // Enable the interrupt handler for the PWM signal
  Serial.println("Encoder initialized. Waiting for PWM signal...");
  _delay(1000); // Allow some time for the encoder to initialize
}

void loop() {
  static unsigned long lastPrint = 0;
  encoder.update();
  float angle = encoder.getAngle(); // in radians
  float pos_deg = angle * 180.0 / PI;

  unsigned long now = millis();
  if (now - lastPrint >= 500) { // Print every 100ms
    Serial.print("Encoder angle: ");
    Serial.print(pos_deg);
    Serial.println(" deg");
    lastPrint = now;
  }
  // No blocking delay here
}