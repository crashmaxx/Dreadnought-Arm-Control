#include "encoder_handler.h"

// PWM sensor interrupt handler
void doPWM() { encoderPWM.handlePWM(); }

// Quadrature encoder interrupt handlers
void doEncoderA() { encoder.handleA(); }
void doEncoderB() { encoder.handleB(); }

// Initialize encoders
void initializeEncoders() {
  // Initialize PWM sensor
  encoderPWM.init();
  encoderPWM.enableInterrupt(doPWM);

  // Initialize quadrature encoder
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doEncoderA, doEncoderB);

  // Read initial PWM sensor angle
  encoderPWM.update();
  anglePWM = encoderPWM.getAngle();
  pos_deg_PWM = anglePWM * 180.0 / PI;
  Serial.print("PWM Sensor initialized. Angle: ");
  Serial.println(pos_deg_PWM);
}

// Update encoder readings (call this in main loop)
void updateEncoders() {
  encoder.update();
  angleAB = encoder.getAngle();
  angleAB = angleAB + anglePWM; // Combine quadrature and PWM if needed
  pos_deg = angleAB * 180.0 / PI;
}
