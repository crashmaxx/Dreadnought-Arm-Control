#include "smart_tuning.h"
#include "shared_globals.h"
#include "config_manager.h"

// Forward declarations
void startStepTest();
void completeStepAnalysis();
void adjustKpBasedOnResponse(double overshoot, bool reached, double ssError);
void adjustKdBasedOnResponse(double overshoot, unsigned long settling);
void adjustKiBasedOnResponse(double ssError);
bool shouldContinuePhase();
void advanceToNextPhase();
bool checkIfMoving();
void waitForMotorToSettle();

// Smart tuning state
bool smartTuningEnabled = false;
bool smartWaitingForArm = false;
double smartKp = 1.0;
double smartKi = 0.0;
double smartKd = 0.1;
unsigned long stepStartTime = 0;
unsigned long smartStartTime = 0;
double stepStartPosition = 0;
double stepTarget = 0;
int tuningPhase = 0;  // 0=Kp, 1=Kd, 2=Ki
int stepCount = 0;
bool stepActive = false;

// Response analysis variables
double maxOvershoot = 0;
double settlingTime = 0;
double steadyStateError = 0;
unsigned long settleStartTime = 0;
bool hasSettled = false;
double peakPosition = 0;
bool reachedTarget = false;

// Improved settling detection
const int POSITION_HISTORY_SIZE = 1000;
double positionHistory[POSITION_HISTORY_SIZE];
int historyIndex = 0;
bool historyFull = false;
const double SETTLE_RANGE = 0.2;  // degrees

// Tuning parameters
const double KP_START = 1.0;    // Much more aggressive starting Kp for 45° movements
const double KP_STEP = 0.5;     // Larger steps for faster convergence
const double KD_STEP = 0.1;     // Larger Kd steps
const double KI_STEP = 0.02;    // Larger Ki steps
const double TARGET_STEP_SIZE = 45.0;
const unsigned long STEP_TIMEOUT = 8000;  // 8 seconds per step
const unsigned long SETTLE_TIME = 2000;   // 2 seconds to settle
const double SETTLE_TOLERANCE = 2.0;      // ±2 degrees

// Start smart auto-tuning
void startSmartTuning() {
  if (!smartTuningEnabled) {
    Serial.println("Smart tuning requested - waiting for system to be armed...");
    Serial.println("Please enable arm switch (Channel 5 > 1700) to begin Smart Tuning");
    
    smartTuningEnabled = true;
    smartWaitingForArm = true;
    smartStartTime = millis();
    tuningPhase = 0;  // Start with Kp tuning
    stepCount = 0;
    
    // Start with more aggressive values for 45° movements
    smartKp = KP_START;  // 2.0 instead of 0.5
    smartKi = 0.0;  // Start with no integral
    smartKd = 0.2; // Higher derivative to start
    
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Don't start step test yet - wait until armed
  }
}

// Stop smart tuning
void stopSmartTuning() {
  if (smartTuningEnabled) {
    smartTuningEnabled = false;
    smartWaitingForArm = false;
    stepActive = false;
    Serial.println("=== SMART TUNING STOPPED ===");
    Serial.printf("Final values: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", smartKp, smartKi, smartKd);
  }
}

// Apply smart tuning results
void applySmartTuning() {
  if (smartTuningEnabled) {
    Kp = smartKp;
    Ki = smartKi;
    Kd = smartKd;
    myPID.SetTunings(Kp, Ki, Kd);
    
    Serial.println("=== SMART TUNING APPLIED ===");
    Serial.printf("New PID values: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
    
    // Save to config
    saveConfig();
    stopSmartTuning();
  }
}

// Start a new step test
void startStepTest() {
  if (smartTuningEnabled) {
    Serial.printf("Starting new step test - Current position: %.1f°\n", pos_deg);
    
    // Wait for motor to settle before starting
    waitForMotorToSettle();
    
    stepStartTime = millis();
    stepStartPosition = pos_deg;  // Use current actual position as start
    stepActive = true;
    hasSettled = false;
    reachedTarget = false;
    
    // Set target based on CURRENT position (not original step start position)
    if (stepCount % 2 == 0) {
      stepTarget = pos_deg + TARGET_STEP_SIZE;  // Use current position
    } else {
      stepTarget = pos_deg - TARGET_STEP_SIZE;  // Use current position
    }
    
    target_deg = stepTarget;
    
    // Reset analysis variables
    maxOvershoot = 0;
    settlingTime = 0;
    steadyStateError = 0;
    peakPosition = stepStartPosition;  // Now matches current position
    
    stepCount++;
    
    Serial.printf("Step %d: %.1f° -> %.1f° (Phase %d: %s) - Movement: %.1f°\n", 
                 stepCount, stepStartPosition, stepTarget,
                 tuningPhase + 1,
                 tuningPhase == 0 ? "Kp" : (tuningPhase == 1 ? "Kd" : "Ki"),
                 abs(stepTarget - stepStartPosition));
    Serial.printf("Current PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", smartKp, smartKi, smartKd);
    Serial.printf("Target error: %.1f° - expecting significant movement!\n", abs(stepTarget - pos_deg));
  }
}

// Analyze step response and update PID constants
void analyzeStepResponse() {
  if (!stepActive) return;
  
  unsigned long elapsed = millis() - stepStartTime;
  double error = stepTarget - pos_deg;
  double response = pos_deg - stepStartPosition;
  double targetResponse = stepTarget - stepStartPosition;
  
  // Debug output every 2 seconds during step
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    Serial.printf("Step progress - Error: %.1f°, Response: %.1f°, PID Output: %.1f\n", 
                 error, response, pid_output);
    lastDebug = millis();
  }
  
  // Track peak response for overshoot calculation
  if (abs(response) > abs(peakPosition - stepStartPosition)) {
    peakPosition = pos_deg;
  }
  
  // Check if we've reached close to target
  if (abs(error) < SETTLE_TOLERANCE && !reachedTarget) {
    reachedTarget = true;
    settleStartTime = millis();
  }
  
  // Check for settling
  if (reachedTarget && !hasSettled) {
    if (millis() - settleStartTime > SETTLE_TIME) {
      hasSettled = true;
      settlingTime = settleStartTime - stepStartTime;
      steadyStateError = abs(error);
    } else if (abs(error) > SETTLE_TOLERANCE) {
      // Reset settle timer if we move away from target
      reachedTarget = false;
    }
  }
  
  // Step timeout or completion
  if (elapsed > STEP_TIMEOUT || hasSettled) {
    completeStepAnalysis();
  }
}

// Complete step analysis and adjust PID
void completeStepAnalysis() {
  stepActive = false;
  
  double targetResponse = stepTarget - stepStartPosition;
  double actualPeak = peakPosition - stepStartPosition;
  double overshoot = 0;
  
  if (targetResponse != 0) {
    overshoot = ((actualPeak - targetResponse) / targetResponse) * 100;
  }
  
  Serial.printf("Results: Overshoot=%.1f%%, Settling=%.1fs, Error=%.1f°\n", 
               overshoot, settlingTime/1000.0, steadyStateError);
  
  // Store the current stepActive state before calling adjustment functions
  bool wasStepActive = stepActive;
  
  // Adjust PID based on current tuning phase
  if (tuningPhase == 0) {  // Kp tuning phase
    adjustKpBasedOnResponse(overshoot, reachedTarget, steadyStateError);
  } else if (tuningPhase == 1) {  // Kd tuning phase
    adjustKdBasedOnResponse(overshoot, settlingTime);
  } else if (tuningPhase == 2) {  // Ki tuning phase
    adjustKiBasedOnResponse(steadyStateError);
  }
  
  // Only continue with phase logic if adjustment functions didn't start a new step
  if (!stepActive) {
    // Decide next action
    if (shouldContinuePhase()) {
      // Continue current phase with new step
      Serial.println("Continuing current phase with new step");
      Serial.println("Waiting for motor to settle before next step...");
      delay(1000);  // Brief pause between steps
      waitForMotorToSettle();  // Wait for motor to stop moving
      startStepTest();
    } else {
      // Move to next phase or complete
      advanceToNextPhase();
    }
  }
}

// Adjust Kp based on response
void adjustKpBasedOnResponse(double overshoot, bool reached, double ssError) {
  if (!reached && ssError > 10.0) {
    // Too slow, increase Kp
    smartKp += KP_STEP;
    Serial.printf("Too slow, increasing Kp to %.2f\n", smartKp);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Kp
    Serial.println("Starting new step test with increased Kp");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else if (overshoot > 15.0) {
    // Too much overshoot, decrease Kp
    smartKp -= KP_STEP * 0.5;
    if (smartKp < 0.1) smartKp = 0.1;
    Serial.printf("Overshoot %.1f%%, decreasing Kp to %.2f\n", overshoot, smartKp);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Kp
    Serial.println("Starting new step test with decreased Kp");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else if (overshoot < 5.0 && ssError < 5.0 && reached) {
    // Good response, maybe try slightly higher
    smartKp += KP_STEP * 0.3;
    Serial.printf("Good response, fine-tuning Kp to %.2f\n", smartKp);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Kp
    Serial.println("Starting new step test with fine-tuned Kp");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else {
    Serial.printf("Kp=%.2f looks good, moving to Kd tuning\n", smartKp);
    return;  // Kp is good, ready for next phase
  }
}

// Adjust Kd based on response
void adjustKdBasedOnResponse(double overshoot, unsigned long settling) {
  if (overshoot > 10.0 || settling > 4000) {
    // Need more damping
    smartKd += KD_STEP;
    Serial.printf("Need damping, increasing Kd to %.3f\n", smartKd);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Kd
    Serial.println("Starting new step test with increased Kd");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else if (overshoot < 2.0 && settling < 2000) {
    // Maybe over-damped, try less Kd
    smartKd -= KD_STEP * 0.5;
    if (smartKd < 0.01) smartKd = 0.01;
    Serial.printf("Well damped, fine-tuning Kd to %.3f\n", smartKd);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Kd
    Serial.println("Starting new step test with decreased Kd");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else {
    Serial.printf("Kd=%.3f looks good, moving to Ki tuning\n", smartKd);
    return;  // Kd is good
  }
}

// Adjust Ki based on response
void adjustKiBasedOnResponse(double ssError) {
  if (ssError > 3.0) {
    // Need integral action
    smartKi += KI_STEP;
    Serial.printf("Steady-state error %.1f°, increasing Ki to %.3f\n", ssError, smartKi);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Ki
    Serial.println("Starting new step test with increased Ki");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else if (ssError < 1.0 && smartKi > 0) {
    // Maybe too much integral
    smartKi -= KI_STEP * 0.5;
    if (smartKi < 0) smartKi = 0;
    Serial.printf("Low error, fine-tuning Ki to %.3f\n", smartKi);
    myPID.SetTunings(smartKp, smartKi, smartKd);
    
    // Start new step test to evaluate new Ki
    Serial.println("Starting new step test with decreased Ki");
    Serial.println("Waiting for motor to settle before next step...");
    delay(500);  // Brief pause for PID to update
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  } else {
    Serial.printf("Ki=%.3f looks good, tuning complete!\n", smartKi);
    return;  // Ki is good
  }
}

// Check if we should continue current phase
bool shouldContinuePhase() {
  // Limit steps per phase
  int stepsInPhase = (stepCount - 1) % 4 + 1;
  return stepsInPhase < 4;  // Max 4 steps per phase
}

// Advance to next tuning phase
void advanceToNextPhase() {
  tuningPhase++;
  
  if (tuningPhase >= 3) {
    // All phases complete
    Serial.println("=== SMART TUNING COMPLETE ===");
    Serial.printf("Optimized PID: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", smartKp, smartKi, smartKd);
    Serial.println("Type 'apply' to save these values or 'cancel' to discard");
  } else {
    // Start next phase
    const char* phaseNames[] = {"Kp", "Kd", "Ki"};
    Serial.printf("Phase %d: Optimizing %s\n", tuningPhase + 1, phaseNames[tuningPhase]);
    Serial.println("Waiting for motor to settle before starting new phase...");
    delay(2000);  // Pause between phases
    waitForMotorToSettle();  // Wait for motor to stop moving
    startStepTest();
  }
}

// Handle smart tuning process (call from main loop)
void handleSmartTuning() {
  if (smartTuningEnabled) {
    // Check if we're waiting for the arm switch to be enabled
    if (smartWaitingForArm) {
      int armswitch = crsf.getChannel(5);
      if (armswitch >= 1700) {
        // System is now armed, start the actual smart tuning
        Serial.println("System armed - Starting Smart Auto-Tuning...");
        Serial.println("=== SMART AUTO-TUNING STARTED ===");
        Serial.println("Phase 1: Finding optimal Kp (proportional gain)");
        Serial.printf("Starting values: Kp=%.2f, Ki=%.3f, Kd=%.3f\n", smartKp, smartKi, smartKd);
        
        smartWaitingForArm = false;
        smartStartTime = millis(); // Reset timer for actual tuning
        
        // Start first step test
        startStepTest();
      } else {
        // Still waiting - check for timeout (30 seconds to arm)
        if (millis() - smartStartTime > 30000) {
          Serial.println("Smart tuning cancelled - timeout waiting for arm switch");
          smartTuningEnabled = false;
          smartWaitingForArm = false;
        }
        return;
      }
    }
    
    // Check if system is still armed during tuning
    int armswitch = crsf.getChannel(5);
    if (armswitch < 1700) {
      Serial.println("Smart tuning stopped: System disarmed during tuning");
      stopSmartTuning();
      return;
    }
    
    // Check for timeout (5 minutes max)
    if (millis() - smartStartTime > 300000) {
      Serial.println("Smart tuning timeout (5 minutes), stopping...");
      stopSmartTuning();
    } else if (stepActive) {
      analyzeStepResponse();
    }
  }
}

// Check if motor is currently moving
bool checkIfMoving() {
  // Add current position to history
  positionHistory[historyIndex] = pos_deg;
  historyIndex = (historyIndex + 1) % POSITION_HISTORY_SIZE;
  
  if (!historyFull && historyIndex == 0) {
    historyFull = true;
  }
  
  // Need full history to make determination
  if (!historyFull) {
    return true;  // Assume moving until we have enough data
  }
  
  // Find min and max in the last 100 readings
  double minPos = positionHistory[0];
  double maxPos = positionHistory[0];
  
  for (int i = 1; i < POSITION_HISTORY_SIZE; i++) {
    if (positionHistory[i] < minPos) minPos = positionHistory[i];
    if (positionHistory[i] > maxPos) maxPos = positionHistory[i];
  }
  
  double range = maxPos - minPos;
  bool settled = (range <= SETTLE_RANGE);
  
  static bool lastSettled = false;
  if (settled != lastSettled) {
    if (settled) {
      Serial.printf("Motor settled: position range %.3f° over last %d readings\n", 
                   range, POSITION_HISTORY_SIZE);
    } else {
      Serial.printf("Motor moving: position range %.3f° (pos: %.1f°)\n", range, pos_deg);
    }
    lastSettled = settled;
  }
  
  return !settled;
}

// Wait for motor to settle before starting new step test
void waitForMotorToSettle() {
  Serial.printf("Waiting for motor to settle (current pos: %.1f°)...\n", pos_deg);
  Serial.printf("Checking for %.1f° range over last %d position readings\n", 
               SETTLE_RANGE, POSITION_HISTORY_SIZE);
  
  unsigned long settleStart = millis();
  
  // Check as often as possible - no artificial delays
  while (checkIfMoving() && millis() - settleStart < 10000) {  // 10 second timeout
    // No delay - check as fast as possible
  }
  
  if (millis() - settleStart >= 10000) {
    Serial.println("Timeout waiting for motor to settle (10s), proceeding anyway");
    Serial.printf("Final position: %.1f°\n", pos_deg);
  } else {
    Serial.printf("Motor settled after %.2fs at position %.1f°\n", 
                 (millis() - settleStart) / 1000.0, pos_deg);
  }
}
