#include <Arduino.h>
#include <SimpleFOC.h>
#include <AlfredoCRSF.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <esp_now_telemetry.h>
#include "board_config.h"

// ------------------- Pin Definitions -------------------
#define PIN_RX 1
#define PIN_TX 2
#define VESC_PPM_PIN 8
#define ENCODER_I_PIN 4
#define ENCODER_PWM_PIN 5
#define ENCODER_A_PIN 7
#define ENCODER_B_PIN 6
#define ENCODER_PPR 2048

// ------------------- Global Variables -------------------
// EEPROM Configuration
#define EEPROM_SIZE 512
#define CONFIG_VERSION 2  // Incremented for new features
#define CONFIG_START 0

struct Config {
  int version;
  double Kp;
  double Ki; 
  double Kd;
  char board_name[32];
  int channel_number;
  int min_angle;
  int max_angle;
  byte checksum;
};

Config config;

// PID tuning parameters (use board config defaults)
#define min_deg MIN_ANGLE
#define max_deg MAX_ANGLE
double Kp = DEFAULT_KP, Ki = DEFAULT_KI, Kd = DEFAULT_KD;

// Board configuration
String board_name = BOARD_NAME;
int channel_number = CONTROL_CHANNEL;

// Encoder angles and positions
float angleAB = 0;        // Quadrature encoder angle (radians)
float anglePWM = 0;       // PWM sensor angle (radians)
double pos_deg = 0;       // Combined position (degrees)
double pos_deg_PWM = 0;   // PWM sensor position (degrees)

double target_deg = 0;    // Target position (degrees)
double pid_output = 0;    // PID output

// AutoTune variables
bool autoTuneEnabled = false;
bool tuningComplete = false;
unsigned long tuneStartTime = 0;
PID_ATune aTune(&pos_deg, &pid_output);

// Debug output control
bool debugEnabled = false;

// ------------------- Hardware Objects -------------------
Servo vescPPM; // VESC PPM output

// PWM sensor (1us = 0 deg, 1060us = 359 deg)
MagneticSensorPWM encoderPWM = MagneticSensorPWM(ENCODER_PWM_PIN, 1, 1060);
void doPWM() { encoderPWM.handlePWM(); }

// Quadrature encoder
Encoder encoder = Encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_PPR, 4);
void doEncoderA() { encoder.handleA(); }
void doEncoderB() { encoder.handleB(); }

// CRSF receiver
HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

// ESP-NOW peer MAC address (from board config)
const uint8_t peer_addr[6] = PEER_MAC;

// ------------------- PID Controller -------------------
PID myPID(&pos_deg, &pid_output, &target_deg, Kp, Ki, Kd, DIRECT);

// ------------------- Utility Functions -------------------
// Calculate checksum for config validation
byte calculateChecksum(Config* cfg) {
  byte checksum = 0;
  byte* ptr = (byte*)cfg;
  for (int i = 0; i < sizeof(Config) - 1; i++) {
    checksum += ptr[i];
  }
  return checksum;
}

// Save configuration to EEPROM
void saveConfig() {
  config.version = CONFIG_VERSION;
  config.Kp = Kp;
  config.Ki = Ki;
  config.Kd = Kd;
  strncpy(config.board_name, board_name.c_str(), sizeof(config.board_name) - 1);
  config.board_name[sizeof(config.board_name) - 1] = '\0';
  config.channel_number = channel_number;
  config.min_angle = min_deg;
  config.max_angle = max_deg;
  config.checksum = calculateChecksum(&config);
  
  EEPROM.put(CONFIG_START, config);
  EEPROM.commit();
  Serial.println("Configuration saved to EEPROM");
}

// Load configuration from EEPROM
bool loadConfig() {
  EEPROM.get(CONFIG_START, config);
  
  // Validate config
  if (config.version != CONFIG_VERSION || 
      config.checksum != calculateChecksum(&config)) {
    Serial.println("Invalid or corrupted config, using defaults");
    return false;
  }
  
  // Load values
  Kp = config.Kp;
  Ki = config.Ki;
  Kd = config.Kd;
  board_name = String(config.board_name);
  channel_number = config.channel_number;
  
  Serial.println("Configuration loaded from EEPROM");
  Serial.printf("Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
  return true;
}

// Export configuration to JSON format
void exportConfigJSON() {
  JsonDocument doc;
  
  doc["board_name"] = board_name;
  doc["firmware_version"] = "AutoTune_v1.0";
  doc["config_version"] = CONFIG_VERSION;
  doc["timestamp"] = millis();
  
  JsonObject pid = doc["pid"];
  pid["Kp"] = Kp;
  pid["Ki"] = Ki;
  pid["Kd"] = Kd;
  
  JsonObject settings = doc["settings"];
  settings["channel_number"] = channel_number;
  settings["min_angle"] = min_deg;
  settings["max_angle"] = max_deg;
  
  JsonObject hardware = doc["hardware"];
  hardware["encoder_ppr"] = ENCODER_PPR;
  hardware["pwm_min"] = 1;
  hardware["pwm_max"] = 1060;
  
  Serial.println("=== CONFIG EXPORT START ===");
  serializeJsonPretty(doc, Serial);
  Serial.println();
  Serial.println("=== CONFIG EXPORT END ===");
  Serial.println("Copy the JSON above to save your configuration");
}

// Import configuration from JSON (paste into Serial Monitor)
void importConfigJSON() {
  Serial.println("Paste your JSON configuration and press Enter:");
  Serial.println("(Waiting for input...)");
  
  // Wait for serial input
  String jsonInput = "";
  unsigned long startTime = millis();
  
  while (millis() - startTime < 30000) { // 30 second timeout
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (jsonInput.length() > 10) break; // Got some data
      } else {
        jsonInput += c;
      }
    }
  }
  
  if (jsonInput.length() < 10) {
    Serial.println("Import timeout or no data received");
    return;
  }
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonInput);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Import values
  if (doc.containsKey("pid")) {
    if (doc["pid"].containsKey("Kp")) Kp = doc["pid"]["Kp"];
    if (doc["pid"].containsKey("Ki")) Ki = doc["pid"]["Ki"];
    if (doc["pid"].containsKey("Kd")) Kd = doc["pid"]["Kd"];
  }
  
  if (doc.containsKey("settings")) {
    if (doc["settings"].containsKey("channel_number")) 
      channel_number = doc["settings"]["channel_number"];
  }
  
  if (doc.containsKey("board_name")) {
    board_name = doc["board_name"].as<String>();
  }
  
  // Update PID controller
  myPID.SetTunings(Kp, Ki, Kd);
  
  // Save to EEPROM
  saveConfig();
  
  Serial.println("Configuration imported successfully!");
  Serial.printf("New PID values - Kp: %.3f, Ki: %.3f, Kd: %.3f\n", Kp, Ki, Kd);
}

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

// Print all CRSF channel values
void printChannels() {
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++) {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
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
      Serial.println("Unknown command. Available commands:");
      Serial.println("  autotune - Start/Stop PID AutoTune");
      Serial.println("  save - Save config to EEPROM");
      Serial.println("  load - Load config from EEPROM");
      Serial.println("  debug - Toggle debug output on/off");
      Serial.println("  export - Export config to JSON");
      Serial.println("  import - Import config from JSON");
      Serial.println("  status - Show current configuration");
    }
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load configuration from EEPROM
  if (!loadConfig()) {
    // Save default configuration if loading failed
    saveConfig();
  }

  // Initialize PWM sensor
  encoderPWM.init();
  encoderPWM.enableInterrupt(doPWM);

  // Initialize quadrature encoder
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doEncoderA, doEncoderB);

  // Initialize CRSF receiver
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
  crsf.begin(crsfSerial);
  Serial.println("CRSF initialized. Waiting for receiver...");

  // Initialize VESC PPM output
  vescPPM.attach(VESC_PPM_PIN);
  vescPPM.setPeriodHertz(50);

  // Initialize PID with loaded parameters
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-500, 500);

  // Initialize ESP-NOW telemetry
  setTelemetryPeer(peer_addr);

  // Read initial PWM sensor angle
  encoderPWM.update();
  anglePWM = encoderPWM.getAngle();
  pos_deg_PWM = anglePWM * 180.0 / PI;
  Serial.print("PWM Sensor initialized. Angle: ");
  Serial.println(pos_deg_PWM);
  
  Serial.println("Setup complete. Serial Commands:");
  Serial.println("  'autotune' - Start/Stop PID AutoTune");
  Serial.println("  'save' - Save config to EEPROM");
  Serial.println("  'load' - Load config from EEPROM");
  Serial.println("  'debug' - Toggle debug output on/off");
  Serial.println("  'export' - Export config to JSON");
  Serial.println("  'import' - Import config from JSON");
  Serial.println("  'status' - Show current configuration");
  Serial.printf("Board: %s, Channel: %d\n", board_name.c_str(), channel_number);
}

// ------------------- Main Loop -------------------
void loop() {
  // Handle serial commands
  handleSerialCommands();

  // Update encoders
  encoder.update();
  angleAB = encoder.getAngle();
  angleAB = angleAB + anglePWM; // Combine quadrature and PWM if needed
  pos_deg = angleAB * 180.0 / PI;

  // Update CRSF
  crsf.update();

  // Map channel (1000-2000us) to claw range
  int target_ppm = crsf.getChannel(channel_number);
  target_deg = map(target_ppm, 1000, 2000, min_deg, max_deg);
  if (target_deg < min_deg) target_deg = min_deg;
  if (target_deg > max_deg) target_deg = max_deg;

  // Handle AutoTune
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

  // Read arming channel
  int armswitch = crsf.getChannel(5);
  
  // Calculate PPM output for VESC
  int ppm_out;
  if (armswitch >= 1700) { // Channel 5 armed
    ppm_out = 1500 + (int)pid_output;
    if (ppm_out < 1000) ppm_out = 1000;
    if (ppm_out > 2000) ppm_out = 2000;
  } else {
    ppm_out = 1500; // Disarmed
  }
  vescPPM.writeMicroseconds(ppm_out);

  // Print debug info every second (only if debug is enabled)
  // This is non-blocking and uses millis() to avoid delays
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

  // Send telemetry every tenth of a second
  static unsigned long lastSend = 0;
  if (now - lastSend > 100) {
    sendTelemetry(board_name.c_str(), pos_deg, target_deg, ppm_out);
    lastSend = now;
  }
  // No blocking delay here
}