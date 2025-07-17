#include "pid_autotune.h"
#include "config_manager.h"

// Start PID AutoTune
void startAutoTune() {
  if (!autoTuneEnabled) {
    Serial.println("Starting PID AutoTune...");
    aTune.SetNoiseBand(2.0);  // Noise band for autotune
    aTune.SetOutputStep(100); // Output step size
    aTune.SetLookbackSec(10); // Lookback time in seconds
    aTune.SetControlType(1);  // PID control type
    
    autoTuneEnabled = true;
    tuningComplete = false;
    tuneStartTime = millis();
    
    // Switch to manual mode during tuning
    myPID.SetMode(MANUAL);
  }
}

// Stop AutoTune and apply results
void stopAutoTune() {
  if (autoTuneEnabled) {
    autoTuneEnabled = false;
    
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
    // Check for timeout (5 minutes max)
    if (millis() - tuneStartTime > 300000) {
      Serial.println("AutoTune timeout, stopping...");
      stopAutoTune();
    } else {
      // Run autotune
      int val = aTune.Runtime();
      if (val != 0) {
        tuningComplete = true;
        Serial.println("AutoTune phase complete");
        if (val == 1) {
          // Autotune finished successfully
          stopAutoTune();
        }
      }
    }
  } else {
    // Normal PID operation
    myPID.Compute();
  }
}
