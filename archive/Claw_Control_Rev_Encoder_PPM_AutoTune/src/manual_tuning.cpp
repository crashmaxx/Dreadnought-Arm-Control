#include "manual_tuning.h"
#include "shared_globals.h"

// Manual tuning state
bool manualTuningEnabled = false;
float testKp = 1.0;
float testKi = 0.0;
float testKd = 0.1;
unsigned long tuningStartTime = 0;
float startPosition = 0;
float testTarget = 0;

// Start manual step response test
void startManualTuning() {
  if (!manualTuningEnabled) {
    manualTuningEnabled = true;
    tuningStartTime = millis();
    startPosition = pos_deg;
    
    // Set test target 45 degrees away
    testTarget = startPosition + 45.0;
    if (testTarget > max_deg) testTarget = startPosition - 45.0;
    
    // Start with conservative values
    testKp = 2.0;
    testKi = 0.1;
    testKd = 0.3;
    
    myPID.SetTunings(testKp, testKi, testKd);
    target_deg = testTarget;
    
    Serial.println("=== MANUAL TUNING STARTED ===");
    Serial.printf("Start Position: %.1f deg\n", startPosition);
    Serial.printf("Test Target: %.1f deg\n", testTarget);
    Serial.printf("Test PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", testKp, testKi, testKd);
    Serial.println("Watch for overshoot, settling time, and steady-state error");
    Serial.println("Commands: 'kp+', 'kp-', 'ki+', 'ki-', 'kd+', 'kd-', 'test', 'apply', 'cancel'");
  }
}

// Stop manual tuning
void stopManualTuning() {
  if (manualTuningEnabled) {
    manualTuningEnabled = false;
    Serial.println("=== MANUAL TUNING STOPPED ===");
  }
}

// Apply current test values
void applyManualTuning() {
  if (manualTuningEnabled) {
    Kp = testKp;
    Ki = testKi;
    Kd = testKd;
    myPID.SetTunings(Kp, Ki, Kd);
    
    Serial.println("=== TUNING APPLIED ===");
    Serial.printf("New PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
    
    stopManualTuning();
  }
}

// Run a new step test
void runStepTest() {
  if (manualTuningEnabled) {
    tuningStartTime = millis();
    startPosition = pos_deg;
    
    // Alternate target
    if (abs(target_deg - testTarget) < 5.0) {
      // Switch to opposite direction
      testTarget = startPosition + (testTarget > startPosition ? -45.0 : 45.0);
      if (testTarget > max_deg) testTarget = max_deg - 10;
      if (testTarget < min_deg) testTarget = min_deg + 10;
    }
    
    target_deg = testTarget;
    myPID.SetTunings(testKp, testKi, testKd);
    
    Serial.println("=== NEW STEP TEST ===");
    Serial.printf("Start: %.1f deg -> Target: %.1f deg\n", startPosition, testTarget);
    Serial.printf("Current PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", testKp, testKi, testKd);
  }
}

// Adjust PID parameters
void adjustKp(bool increase) {
  if (manualTuningEnabled) {
    double step = testKp > 1.0 ? 0.2 : 0.1;
    testKp += increase ? step : -step;
    if (testKp < 0.1) testKp = 0.1;
    if (testKp > 10.0) testKp = 10.0;
    
    myPID.SetTunings(testKp, testKi, testKd);
    Serial.printf("Kp adjusted to: %.2f\n", testKp);
  }
}

void adjustKi(bool increase) {
  if (manualTuningEnabled) {
    double step = testKi > 0.1 ? 0.02 : 0.01;
    testKi += increase ? step : -step;
    if (testKi < 0.0) testKi = 0.0;
    if (testKi > 1.0) testKi = 1.0;
    
    myPID.SetTunings(testKp, testKi, testKd);
    Serial.printf("Ki adjusted to: %.3f\n", testKi);
  }
}

void adjustKd(bool increase) {
  if (manualTuningEnabled) {
    double step = testKd > 0.5 ? 0.05 : 0.02;
    testKd += increase ? step : -step;
    if (testKd < 0.0) testKd = 0.0;
    if (testKd > 2.0) testKd = 2.0;
    
    myPID.SetTunings(testKp, testKi, testKd);
    Serial.printf("Kd adjusted to: %.3f\n", testKd);
  }
}

// Print tuning status
void printTuningStatus() {
  if (manualTuningEnabled) {
    double error = target_deg - pos_deg;
    unsigned long elapsed = (millis() - tuningStartTime) / 1000;
    
    Serial.printf("Time: %lus, Position: %.1f deg, Target: %.1f deg, Error: %.1f deg\n", 
                 elapsed, pos_deg, target_deg, error);
    Serial.printf("Current PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", testKp, testKi, testKd);
  }
}
