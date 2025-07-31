#include "serial_commands.h"
#include "pid_autotune.h"
#include "manual_tuning.h"

// Print all CRSF channel values
void printChannels() {
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

// Show available commands
void showAvailableCommands() {
  Serial.println("Unknown command. Available commands:");
  Serial.println("  autotune - Start/Stop sTune AutoTune (waits for arm switch)");
  Serial.println("  manual - Start manual step-response tuning");
  Serial.println("  kp+/kp- - Increase/decrease Kp during manual tuning");
  Serial.println("  ki+/ki- - Increase/decrease Ki during manual tuning");  
  Serial.println("  kd+/kd- - Increase/decrease Kd during manual tuning");
  Serial.println("  test - Run new step test during manual tuning");
  Serial.println("  apply - Apply current tuning values");
  Serial.println("  cancel - Cancel active tuning");
  Serial.println("  debug - Toggle debug output on/off");
  Serial.println("  status - Show current configuration");
}

// Handle serial commands
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "autotune") {
      if (autoTuneEnabled) {
        Serial.println("Stopping AutoTune...");
        stopAutoTune();
      } else {
        Serial.println("Starting AutoTune...");
        startAutoTune();
      }
    } else if (command == "manual") {
      if (manualTuningEnabled) {
        Serial.println("Manual tuning already active");
        printTuningStatus();
      } else {
        startManualTuning();
      }
    } else if (command == "kp+") {
      adjustKp(true);
    } else if (command == "kp-") {
      adjustKp(false);
    } else if (command == "ki+") {
      adjustKi(true);
    } else if (command == "ki-") {
      adjustKi(false);
    } else if (command == "kd+") {
      adjustKd(true);
    } else if (command == "kd-") {
      adjustKd(false);
    } else if (command == "test") {
      runStepTest();
    } else if (command == "apply") {
      if (manualTuningEnabled) {
        applyManualTuning();
      } else {
        Serial.println("No active tuning to apply");
      }
    } else if (command == "cancel") {
      if (manualTuningEnabled) {
        stopManualTuning();
      } else {
        Serial.println("No active tuning to cancel");
      }
    } else if (command == "debug") {
      debugEnabled = !debugEnabled;
      Serial.printf("Debug output %s\n", debugEnabled ? "ENABLED" : "DISABLED");
    } else if (command == "status") {
      Serial.println("=== CURRENT CONFIGURATION ===");
      Serial.printf("Board: %s\n", board_name.c_str());
      Serial.printf("Channel: %d\n", CONTROL_CHANNEL);
      Serial.printf("PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
      Serial.printf("Angle Range: %d to %d degrees\n", min_deg, max_deg);
      Serial.printf("Current Position: %.1f degrees\n", pos_deg);
      Serial.printf("Target Position: %.1f degrees\n", target_deg);
      Serial.printf("AutoTune: %s\n", autoTuneEnabled ? (waitingForArm ? "WAITING FOR ARM" : "RUNNING") : "STOPPED");
      Serial.printf("Debug Output: %s\n", debugEnabled ? "ENABLED" : "DISABLED");
    } else if (command.length() > 0) {
      showAvailableCommands();
    }
  }
}

// Print debug information
void printDebugInfo(int ppm_out) {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  
  if (debugEnabled && now - lastPrint >= 1000) {
    Serial.print("Encoder angle: ");
    Serial.print(pos_deg);
    Serial.print(" deg, PWM angle: ");
    Serial.print(pos_deg_PWM);
    Serial.print(" deg, Target angle: ");
    Serial.print(target_deg);
    Serial.print(" deg, PPM output: ");
    Serial.println(ppm_out);
    
    if (autoTuneEnabled) {
      if (waitingForArm) {
        Serial.print("AutoTune waiting for arm switch... Time: ");
        Serial.print((now - tuneStartTime) / 1000);
        Serial.println("s");
      } else {
        Serial.print("AutoTune running... Time: ");
        Serial.print((now - tuneStartTime) / 1000);
        Serial.println("s");
      }
    } else {
      Serial.printf("PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
    }
    
    printChannels();
    lastPrint = now;
  }
}
