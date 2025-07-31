#include "pid_autotune.h"

// sTune AutoTune variables
bool autoTuneRunning = false;
unsigned long autoTuneStartTime = 0;
float tuneSetpoint = 90.0;  // Center position for tuning
bool sTuneActive = false;

// Start PID AutoTune using sTune
void startAutoTune() {
  if (!autoTuneEnabled) {
    Serial.println("sTune AutoTune requested - waiting for system to be armed...");
    Serial.println("Please enable arm switch (Channel 5 > 1700) to begin sTune AutoTune");
    
    autoTuneEnabled = true;
    tuningComplete = false;
    waitingForArm = true;
    tuneStartTime = millis();
    autoTuneRunning = false;
    sTuneActive = false;
  }
}

// Stop AutoTune and apply results
void stopAutoTune() {
  if (autoTuneEnabled) {
    autoTuneEnabled = false;
    waitingForArm = false;
    autoTuneRunning = false;
    sTuneActive = false;
    
    if (tuningComplete) {
      Serial.println("sTune AutoTune complete! New parameters:");
      Serial.printf("Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
    } else {
      Serial.println("sTune AutoTune cancelled");
    }
    
    // Switch back to automatic mode
    myPID.SetMode(1);  // 1 = AUTOMATIC mode in QuickPID
  }
}

// Handle sTune AutoTune process (call this in main loop)
void handleAutoTune() {
  if (autoTuneEnabled) {
    // Check if we're waiting for the arm switch to be enabled
    if (waitingForArm) {
      int armswitch = crsf.getChannel(5);
      if (armswitch >= 1700) {
        // System is now armed, start the actual autotune
        Serial.println("System armed - Starting sTune AutoTune...");
        Serial.println("sTune will automatically determine optimal PID parameters");
        
        waitingForArm = false;
        autoTuneRunning = true;
        autoTuneStartTime = millis();
        tuneStartTime = millis(); // Reset timer for actual autotune
        
        // Configure sTune for our system
        tuner.Configure(pos_deg,        // input
                       pid_output,      // output  
                       tuneSetpoint,    // setpoint
                       2.0,             // outputStep (2 degree step)
                       300,             // testTimeSec (5 minutes max)
                       150,             // settleTimeSec (2.5 minutes to settle)
                       100);            // samples (100ms sample time)
        
        // Set the target for tuning
        target_deg = tuneSetpoint;
        
        // Switch PID to manual mode for sTune
        myPID.SetMode(0);  // 0 = MANUAL mode
        sTuneActive = true;
        
        Serial.printf("sTune configured for setpoint: %.1f degrees\n", tuneSetpoint);
        Serial.println("Tuning process will take 3-5 minutes...");
        
      } else {
        // Still waiting - check for timeout (30 seconds to arm)
        if (millis() - tuneStartTime > 30000) {
          Serial.println("sTune AutoTune cancelled - timeout waiting for arm switch");
          autoTuneEnabled = false;
          waitingForArm = false;
        }
        return;
      }
    }
    
    // Check if system is still armed during autotune
    int armswitch = crsf.getChannel(5);
    if (armswitch < 1700) {
      Serial.println("sTune AutoTune stopped: System disarmed during autotune");
      stopAutoTune();
      return;
    }
    
    // Check for timeout (10 minutes max)
    if (millis() - tuneStartTime > 600000) {
      Serial.println("sTune AutoTune timeout (10 minutes), stopping...");
      stopAutoTune();
    } else if (autoTuneRunning && sTuneActive) {
      // Run sTune autotune process
      static unsigned long lastLogTime = 0;
      
      // Update sTune with current position
      switch (tuner.Run()) {
        case tuner.sample:
          // sTune is sampling - use its output
          // sTune handles the output directly
          break;
          
        case tuner.tunings:
          // sTune has completed and calculated new tunings
          tuningComplete = true;
          autoTuneRunning = false;
          sTuneActive = false;
          
          // Get the new tuned parameters from sTune
          Kp = tuner.GetKp();
          Ki = tuner.GetKi(); 
          Kd = tuner.GetKd();
          
          Serial.println("sTune AutoTune successfully completed!");
          Serial.printf("New PID values: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", Kp, Ki, Kd);
          
          // Apply the new parameters to QuickPID
          myPID.SetTunings(Kp, Ki, Kd);
          myPID.SetMode(1);  // Switch back to automatic mode
          
          Serial.println("New PID parameters applied and QuickPID re-enabled");
          break;
          
        case tuner.runPid:
          // sTune wants us to run normal PID control
          myPID.SetMode(1);  // Automatic mode
          if (myPID.Compute()) {
            // PID computed new output
          }
          break;
          
        default:
          // Continue tuning process
          if (millis() - lastLogTime > 15000) {
            Serial.printf("sTune running... Position: %.1f deg, Target: %.1f deg (Runtime: %lds)\n", 
                         pos_deg, tuneSetpoint, (millis() - tuneStartTime) / 1000);
            lastLogTime = millis();
          }
          break;
      }
    }
  }
}
