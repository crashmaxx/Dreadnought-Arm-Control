#include "pid_autotune.h"
#include "config_manager.h"

// Start PID AutoTune
void startAutoTune() {
  if (!autoTuneEnabled) {
    Serial.println("AutoTune requested - waiting for system to be armed...");
    Serial.println("Please enable arm switch (Channel 5 > 1700) to begin AutoTune");
    
    autoTuneEnabled = true;
    tuningComplete = false;
    waitingForArm = true;
    tuneStartTime = millis();
    
    // Don't switch to manual mode yet - wait until armed
  }
}

// Stop AutoTune and apply results
void stopAutoTune() {
  if (autoTuneEnabled) {
    autoTuneEnabled = false;
    waitingForArm = false;
    
    if (tuningComplete) {
      // Get tuned parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi(); 
      Kd = aTune.GetKd();
      
      // Update PID controller
      myPID.SetTunings(Kp, Ki, Kd);
      
      Serial.println("AutoTune complete! New parameters:");
      Serial.printf("Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
      
      // Save to EEPROM
      saveConfig();
    } else {
      Serial.println("AutoTune cancelled");
    }
    
    // Switch back to automatic mode
    myPID.SetMode(AUTOMATIC);
  }
}

// Handle AutoTune process (call this in main loop)
void handleAutoTune() {
  if (autoTuneEnabled) {
    // Check if we're waiting for the arm switch to be enabled
    if (waitingForArm) {
      int armswitch = crsf.getChannel(5);
      if (armswitch >= 1700) {
        // System is now armed, start the actual autotune
        Serial.println("System armed - Starting PID AutoTune...");
        
        aTune.SetNoiseBand(3.0);  // Larger noise band for more aggressive tuning
        aTune.SetOutputStep(75);  // Very large step for strong oscillations
        aTune.SetLookbackSec(3);  // Short lookback for faster analysis
        aTune.SetControlType(1);  // PID control type
        
        waitingForArm = false;
        tuneStartTime = millis(); // Reset timer for actual autotune
        
        // Keep PID in automatic mode so it can chase targets
        myPID.SetMode(AUTOMATIC);
      } else {
        // Still waiting - check for timeout (30 seconds to arm)
        if (millis() - tuneStartTime > 30000) {
          Serial.println("AutoTune cancelled - timeout waiting for arm switch");
          autoTuneEnabled = false;
          waitingForArm = false;
        }
        return;
      }
    }
    
    // Check if system is still armed during autotune
    int armswitch = crsf.getChannel(5);
    if (armswitch < 1700) {
      Serial.println("AutoTune stopped: System disarmed during autotune");
      stopAutoTune();
      return;
    }
    
    // Check for timeout (3 minutes max) - reduced from 5 minutes
    if (millis() - tuneStartTime > 180000) {
      Serial.println("AutoTune timeout (3 minutes), stopping...");
      stopAutoTune();
    } else {
      // Run autotune - let it adjust target position
      static double lastOutput = 0;
      static unsigned long lastLogTime = 0;
      static double baseTarget = 90.0;  // Fixed center target for autotune
      
      int val = aTune.Runtime();
      
      // Apply AutoTune output as target position offset
      target_deg = baseTarget + autoTuneOutput;
      
      // Ensure target stays within limits
      if (target_deg < min_deg) target_deg = min_deg;
      if (target_deg > max_deg) target_deg = max_deg;
      
      // Log output changes and periodic status
      if (autoTuneOutput != lastOutput || (millis() - lastLogTime > 10000)) {
        if (autoTuneOutput > lastOutput) {
          Serial.printf("AutoTune: Step target UP, Target: %.1f + %.1f = %.1f deg (Runtime: %lds, Error: %.1f)\n", 
                       baseTarget, autoTuneOutput, target_deg, (millis() - tuneStartTime) / 1000, target_deg - pos_deg);
        } else if (autoTuneOutput < lastOutput) {
          Serial.printf("AutoTune: Step target DOWN, Target: %.1f + %.1f = %.1f deg (Runtime: %lds, Error: %.1f)\n", 
                       baseTarget, autoTuneOutput, target_deg, (millis() - tuneStartTime) / 1000, target_deg - pos_deg);
        } else {
          Serial.printf("AutoTune: Holding target, Target: %.1f deg, Position: %.1f deg (Runtime: %lds, Error: %.1f)\n", 
                       target_deg, pos_deg, (millis() - tuneStartTime) / 1000, target_deg - pos_deg);
        }
        lastOutput = autoTuneOutput;
        lastLogTime = millis();
      }
      
      if (val != 0) {
        tuningComplete = true;
        Serial.printf("AutoTune phase complete (val=%d)\n", val);
        Serial.printf("Current settings: Noise=%.1f, Step=%.1f, Lookback=%ds\n", 
                     3.0, 75.0, 3);  // Updated to match new aggressive settings
        if (val == 1) {
          // Autotune finished successfully
          Serial.println("AutoTune successfully completed!");
          stopAutoTune();
        } else {
          // Autotune failed or was cancelled
          Serial.println("AutoTune failed or cancelled");
          stopAutoTune();
        }
      }
    }
  }
}
