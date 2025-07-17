#include "serial_commands.h"
#include "config_manager.h"
#include "pid_autotune.h"

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
  Serial.println("  autotune - Start/Stop PID AutoTune");
  Serial.println("  save - Save config to EEPROM");
  Serial.println("  load - Load config from EEPROM");
  Serial.println("  debug - Toggle debug output on/off");
  Serial.println("  export - Export config to JSON");
  Serial.println("  import - Import config from JSON");
  Serial.println("  status - Show current configuration");
}

// Handle serial commands
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "export") {
      exportConfigJSON();
    } else if (command == "import") {
      importConfigJSON();
    } else if (command == "autotune") {
      if (autoTuneEnabled) {
        stopAutoTune();
      } else {
        startAutoTune();
      }
    } else if (command == "save") {
      saveConfig();
    } else if (command == "load") {
      if (loadConfig()) {
        myPID.SetTunings(Kp, Ki, Kd);
      }
    } else if (command == "debug") {
      debugEnabled = !debugEnabled;
      Serial.printf("Debug output %s\n", debugEnabled ? "ENABLED" : "DISABLED");
    } else if (command == "status") {
      Serial.println("=== CURRENT CONFIGURATION ===");
      Serial.printf("Board: %s\n", board_name.c_str());
      Serial.printf("Channel: %d\n", channel_number);
      Serial.printf("PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
      Serial.printf("Angle Range: %d to %d degrees\n", min_deg, max_deg);
      Serial.printf("Current Position: %.1f degrees\n", pos_deg);
      Serial.printf("Target Position: %.1f degrees\n", target_deg);
      Serial.printf("AutoTune: %s\n", autoTuneEnabled ? "RUNNING" : "STOPPED");
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
    Serial.print("Board: ");
    Serial.print(board_name);
    Serial.print(", Channel: ");
    Serial.println(channel_number);
    Serial.print("Encoder angle: ");
    Serial.print(pos_deg);
    Serial.print(" deg, PWM angle: ");
    Serial.print(pos_deg_PWM);
    Serial.print(" deg, Target angle: ");
    Serial.print(target_deg);
    Serial.print(" deg, PPM output: ");
    Serial.println(ppm_out);
    
    if (autoTuneEnabled) {
      Serial.print("AutoTune running... Time: ");
      Serial.print((now - tuneStartTime) / 1000);
      Serial.println("s");
    } else {
      Serial.printf("PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", Kp, Ki, Kd);
    }
    
    printChannels();
    lastPrint = now;
  }
}
