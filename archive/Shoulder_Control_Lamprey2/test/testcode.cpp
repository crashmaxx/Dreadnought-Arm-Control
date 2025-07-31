#include <Arduino.h>
#include <SimpleFOC.h>

// Define the PWM pin for the encoder
#define ENCODER_PWM_PIN 4

volatile unsigned long pwmRiseTime = 0;
volatile unsigned long pwmPulseWidth = 0;

void IRAM_ATTR handlePwmInterrupt() {
  if (digitalRead(ENCODER_PWM_PIN) == HIGH) {
    pwmRiseTime = micros();
  } else {
    pwmPulseWidth = micros() - pwmRiseTime;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PWM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PWM_PIN), handlePwmInterrupt, CHANGE);
}

void loop() {
  static unsigned long lastPrint = 0;
  static unsigned long lastPulse = 0;
  unsigned long now = millis();
  unsigned long pulseWidth;
  noInterrupts();
  pulseWidth = pwmPulseWidth;
  interrupts();
  if (now - lastPrint >= 500) { // Print every 500ms
    Serial.print("PWM pulse: ");
    Serial.print(pulseWidth);
    Serial.println(" us");
    lastPrint = now;
  }
  // No blocking delay here
}